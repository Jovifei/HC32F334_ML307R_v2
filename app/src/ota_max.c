#include "ota_max.h"
#include "sub1g.h"
#include "mmi.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>

// ================= 全局变量定义 =================
ota_manager_t g_ota_manager = {0};

// ================= 内部函数声明 =================
static void ota_device_list_init(void);
static bool ota_start_next_device(void);
static void ota_process_current_device(void);
static void ota_handle_timeout(void);
static void ota_complete_device(uint8_t device_idx, bool success);
static void ota_request_firmware_from_wifi(uint32_t address, uint16_t length);
static void ota_send_finish_result_to_wifi(ota_result_t result);
static void ota_retry_failed_devices(void);

/*---------------------------------------------------------------------------
 Name        : void ota_manager_init(void)
 Input       : 无
 Output      : 无
 Description : 初始化OTA管理器
---------------------------------------------------------------------------*/
void ota_manager_init(void)
{
    memset(&g_ota_manager, 0, sizeof(ota_manager_t));
    g_ota_manager.ota_in_progress = false;
    g_ota_manager.disable_broadcast = false;
    g_ota_manager.disable_property_report = false;
    g_ota_manager.waiting_wifi_data = false;
}

/*---------------------------------------------------------------------------
 Name        : void ota_manager_task(void)
 Input       : 无
 Output      : 无
 Description : OTA管理器主任务（在主循环中调用，1ms周期）
---------------------------------------------------------------------------*/
void ota_manager_task(void)
{
    if (sys_param.flags.timer_1ms_flag)
    {
        sys_param.flags.timer_1ms_flag = 0;

        if (!g_ota_manager.ota_in_progress)
        {
            return;
        }

        // 增加超时计数器
        g_ota_manager.last_activity_ms++;

        // 处理当前设备
        ota_process_current_device();

        // 检查超时
        ota_handle_timeout();
    }
}

/*---------------------------------------------------------------------------
 Name        : ota_start_result_t ota_check_start_status(void)
 Input       : 无
 Output      : OTA_START_READY(0)-成功准备升级
               OTA_START_BUSY(1)-繁忙
               OTA_START_ERROR(2)-错误
 Description : 检查OTA启动状态并初始化升级流程，在parse_ota_start_data解析完成后调用
---------------------------------------------------------------------------*/
ota_start_result_t ota_check_start_status(void)
{
    DEBUG_PRINTF("[OTA] Check start status: fw_type=%d, length=%lu, version=%s, crc=0x%08lX\r\n",
                 g_ota_manager.fw_type, g_ota_manager.fw_length, g_ota_manager.fw_version, g_ota_manager.fw_crc);

    // 检查是否已上报slave版本
    if (!sys_param.slave_version.slave_version_reported)
    {
        DEBUG_PRINTF("[OTA] Necessary all versions not collected yet, return error\r\n");
        return OTA_START_WAIT; // 版本未收集完成，不允许OTA
    }

    // 检查是否正在进行OTA
    if (g_ota_manager.ota_in_progress)
    {
        DEBUG_PRINTF("[OTA] Already in progress, return busy\r\n");
        return OTA_START_BUSY; // 繁忙
    }

    // 验证固件类型
    if (g_ota_manager.fw_type < FW_TYPE_CT_SUB1G || g_ota_manager.fw_type > FW_TYPE_INV_2500W)
    {
        DEBUG_PRINTF("[OTA] Invalid fw_type=%d\r\n", g_ota_manager.fw_type);
        return OTA_START_ERROR; // 错误
    }

    // 重置OTA管理器状态
    g_ota_manager.ota_in_progress = true;
    g_ota_manager.current_device_index = 0;
    g_ota_manager.success_count = 0;
    g_ota_manager.failed_count = 0;
    g_ota_manager.failed_device_count = 0;
    g_ota_manager.firmware_retry_count = 0;

    // 转换固件类型为Sub1G固件升级协议类型
    g_ota_manager.sub1g_type = ota_fw_type_to_sub1g_type(g_ota_manager.fw_type);

    // 计算Sub1G协议总包数（按64字节分包）
    g_ota_manager.total_packets_sub1g = (g_ota_manager.fw_length + OTA_SUB1G_PACKET_SIZE - 1) / OTA_SUB1G_PACKET_SIZE;

    // 初始化设备列表
    ota_device_list_init();

    if (g_ota_manager.need_ota_device_count == 0)
    {
        DEBUG_PRINTF("[OTA] All devices version matched, no need to upgrade\r\n");
        g_ota_manager.ota_in_progress = false;

        // 版本一致，视为全部成功，发送完成消息给WiFi
        ota_send_finish_result_to_wifi(OTA_RESULT_SUCCESS_ALL);

        // update_slave_versions();
        // tx_flag.slave_version = true;

        return OTA_START_READY; // 返回READY表示已处理(版本已是最新)
    }

    // 设置状态标志
    g_ota_manager.disable_broadcast = true;
    g_ota_manager.disable_property_report = true;
    g_ota_manager.retry_round = 0;
    g_ota_manager.failed_device_count = 0;

    DEBUG_PRINTF("[OTA] Initialized: device_count=%d, total_packets=%d\r\n", g_ota_manager.need_ota_device_count, g_ota_manager.total_packets_sub1g);

    // 开始第一个设备的升级
    if (!ota_start_next_device())
    {
        DEBUG_PRINTF("[OTA] Failed to start first device\r\n");
        g_ota_manager.ota_in_progress = false;
        return OTA_START_ERROR; // 错误
    }

    return OTA_START_READY; // 准备就绪
}

/*---------------------------------------------------------------------------
 Name        : static void ota_device_list_init(void)
 Input       : 无
 Output      : 无
 Description : 初始化待升级设备列表
               固件类型3: device_count=1 (CT Sub1G)
               固件类型4/5/6: device_count=已绑定微逆个数
---------------------------------------------------------------------------*/
static void ota_device_list_init(void)
{
    g_ota_manager.need_ota_device_count = 0;

    // 固件类型3：CT Sub1G固件，只需要升级1个设备（CT自己的Sub1G，约定默认使用111111）
    if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
    {
        // 版本判断：如果版本相同，跳过升级
        if (strcmp(sys_param.sub1g.sw_version, g_ota_manager.fw_version) == 0)
        {
            DEBUG_PRINTF("[OTA] CT Sub1G version matched (%s), skip upgrade\r\n", sys_param.sub1g.sw_version);
            g_ota_manager.need_ota_device_count = 0;
            return;
        }

        g_ota_manager.devices[0].sub1g_addr = 0x111111; // 默认地址
        g_ota_manager.devices[0].device_index = 0;
        g_ota_manager.devices[0].state = OTA_STATE_IDLE;
        g_ota_manager.devices[0].current_packet = 0;
        g_ota_manager.devices[0].total_packets = g_ota_manager.total_packets_sub1g;
        g_ota_manager.devices[0].retry_count = 0;
        g_ota_manager.devices[0].device_retry_count = 0;
        g_ota_manager.devices[0].version_query_interval_counter = 0;
        g_ota_manager.devices[0].ota_send_success = false;

        g_ota_manager.need_ota_device_count = 1;
        g_ota_manager.original_device_count = 1; // 保存原始设备数

        DEBUG_PRINTF("[OTA] CT Sub1G upgrade, device_count=1\r\n");
        return;
    }

    // 固件类型4/5/6：微逆Sub1G或MCU固件，需要遍历已配对的微逆设备
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // 检查设备是否已绑定
        if (sys_param.paired_inv_info[i].siid < SIID_MIN || sys_param.paired_inv_info[i].siid > SIID_MAX)
        {
            continue;
        }

        // 检查设备是否在线（只升级在线设备，离线设备不处理）
        if (sys_param.paired_inv_info[i].online_state != CT_STATUS_ONLINE)
        {
            DEBUG_PRINTF("[OTA] INV[%d] is offline (online_state=%d), skip\r\n", i, sys_param.paired_inv_info[i].online_state);
            continue;
        }

        bool version_matched = false;
        if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G)
        {
            // 微逆的Sub1G固件
            if (strcmp(sys_param.paired_inv_info[i].sub1g_version, g_ota_manager.fw_version) == 0)
            {
                DEBUG_PRINTF("[OTA] INV[%d] Sub1G version matched (%s), skip\r\n", i, sys_param.paired_inv_info[i].sub1g_version);
                version_matched = true;
            }
            else
            {
                DEBUG_PRINTF("[OTA] INV[%d] Sub1G version mismatched (%s->%s), need upgrade\r\n", i, sys_param.paired_inv_info[i].sub1g_version, g_ota_manager.fw_version);
            }
        }
        else if (g_ota_manager.fw_type == FW_TYPE_INV_800W || g_ota_manager.fw_type == FW_TYPE_INV_2500W)
        {
            // 微逆的MCU固件
            if (strcmp(sys_param.paired_inv_info[i].sw_version, g_ota_manager.fw_version) == 0)
            {
                DEBUG_PRINTF("[OTA] INV[%d] MCU version matched (%s), skip\r\n", i, sys_param.paired_inv_info[i].sw_version);
                version_matched = true;
            }
        }

        // 如果版本匹配，跳过该设备
        if (version_matched)
        {
            continue;
        }

        uint8_t idx = g_ota_manager.need_ota_device_count;
        g_ota_manager.devices[idx].sub1g_addr = sys_param.paired_inv_info[i].sub1g_addr;
        g_ota_manager.devices[idx].device_index = i;
        g_ota_manager.devices[idx].state = OTA_STATE_IDLE;
        g_ota_manager.devices[idx].current_packet = 0;
        g_ota_manager.devices[idx].total_packets = g_ota_manager.total_packets_sub1g;
        g_ota_manager.devices[idx].retry_count = 0;
        g_ota_manager.devices[idx].device_retry_count = 0;
        g_ota_manager.devices[idx].version_query_interval_counter = 0;
        g_ota_manager.devices[idx].ota_send_success = false;

        g_ota_manager.need_ota_device_count++;

        DEBUG_PRINTF("[OTA] Add device[%d]: sub1g_addr=0x%06lX, inv_index=%d\r\n", idx, sys_param.paired_inv_info[i].sub1g_addr, i);
    }

    // 保存原始设备数
    g_ota_manager.original_device_count = g_ota_manager.need_ota_device_count;

    DEBUG_PRINTF("[OTA] Total devices to upgrade: %d\r\n", g_ota_manager.need_ota_device_count);
}

/*---------------------------------------------------------------------------
 Name        : static bool ota_start_next_device(void)
 Input       : 无
 Output      : true-成功启动, false-无设备可启动
 Description : 执行从一个设备到下一个设备的升级切换和初始化操作
---------------------------------------------------------------------------*/
static bool ota_start_next_device(void)
{
    if (g_ota_manager.current_device_index >= g_ota_manager.need_ota_device_count)
    {
        DEBUG_PRINTF("[OTA] All devices processed: finish_ota_device=%d, need_ota_device_count=%d\r\n", g_ota_manager.current_device_index, g_ota_manager.need_ota_device_count);
        return false;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    DEBUG_PRINTF("[OTA] Start device[%d]: sub1g_addr=0x%06lX\r\n", g_ota_manager.current_device_index, dev->sub1g_addr);

    // 重置设备状态（用于重试）
    dev->state = OTA_STATE_IDLE;
    dev->current_packet = 0;
    dev->retry_count = 0;
    dev->version_query_interval_counter = 0;

    // 发送初始化命令
    ota_send_sub1g_init_cmd(dev->sub1g_addr, g_ota_manager.sub1g_type, g_ota_manager.fw_length);

    // 更新状态
    dev->state = OTA_STATE_INIT_WAIT_ACK;
    dev->timeout_counter = 0;
    g_ota_manager.last_activity_ms = 0;

    // 重置WiFi缓冲区，确保在开始升级新设备时，系统准备好从固件的起始地址（地址 0）请求数据，如果获取到了wifi数据后会更新
    g_ota_manager.fw_current_address = 0;
    g_ota_manager.fw_buffer_valid_len = 0;
    g_ota_manager.waiting_wifi_data = false;

    return true;
}

/*---------------------------------------------------------------------------
 Name        : static void ota_retry_failed_devices(void)
 Input       : 无
 Output      : 无
 Description : 如果某个设备在通信中发生命令超时或初始化应答失败，该设备会被标记为失败，并被记录到失败设备列表中。当所有设备的第一轮尝试结束后，此函数将根据预设的设备级重试限制，决定是否对这些失败的设备进行新一轮的尝试。
---------------------------------------------------------------------------*/
static void ota_retry_failed_devices(void)
{
    g_ota_manager.retry_round++;

    DEBUG_PRINTF("[OTA] Starting retry round %d, failed_device_count=%d\r\n", g_ota_manager.retry_round, g_ota_manager.failed_device_count);

    // 重建设备列表，只包含失败的设备
    ota_device_info_t temp_devices[INV_DEVICE_MAX_NUM];
    uint8_t temp_count = 0;

    for (uint8_t i = 0; i < g_ota_manager.failed_device_count; i++)
    {
        uint8_t failed_idx = g_ota_manager.failed_device_list[i];

        // 检查设备是否已经达到最大重试次数
        if (g_ota_manager.devices[failed_idx].device_retry_count >= OTA_DEVICE_RETRY_MAX)
        {
            DEBUG_PRINTF("[OTA] Device[%d] reached max device retries, skip\r\n", failed_idx);
            continue;
        }

        if (g_ota_manager.fw_type > FW_TYPE_CT_SUB1G && g_ota_manager.fw_type <= FW_TYPE_INV_2500W)
        {
            // 检查设备是否仍然在线
            uint8_t dev_idx = g_ota_manager.devices[failed_idx].device_index;
            if (sys_param.paired_inv_info[dev_idx].online_state != CT_STATUS_ONLINE)
            {
                DEBUG_PRINTF("[OTA] Device[%d] is now offline (online_state=%d), skip retry\r\n", dev_idx, sys_param.paired_inv_info[dev_idx].online_state);
                continue;
            }
        }

        memcpy(&temp_devices[temp_count], &g_ota_manager.devices[failed_idx], sizeof(ota_device_info_t));
        temp_devices[temp_count].device_retry_count++;
        temp_count++;

        DEBUG_PRINTF("[OTA] Add device[%d] to retry list: sub1g_addr=0x%06lX, retry_count=%d\r\n",
                     temp_count - 1, temp_devices[temp_count - 1].sub1g_addr,
                     temp_devices[temp_count - 1].device_retry_count);
    }

    if (temp_count == 0)
    {
        DEBUG_PRINTF("[OTA] No devices need retry\r\n");
        g_ota_manager.ota_in_progress = false;
        g_ota_manager.disable_broadcast = false;
        g_ota_manager.disable_property_report = false;
        return;
    }

    // 如果 temp_count > 0，则需要启动新一轮的重试，那么更新设备列表
    memcpy(g_ota_manager.devices, temp_devices, sizeof(ota_device_info_t) * temp_count);
    g_ota_manager.need_ota_device_count = temp_count;
    g_ota_manager.current_device_index = 0;
    g_ota_manager.failed_device_count = 0;

    // 开始第一个设备的重试
    ota_start_next_device();
}

/*---------------------------------------------------------------------------
 Name        : static void ota_process_current_device(void)
 Input       : 无
 Output      : 无
 Description : 负责根据当前设备的 OTA 状态来推进或结束其固件传输流程。
---------------------------------------------------------------------------*/
static void ota_process_current_device(void)
{
    if (g_ota_manager.current_device_index >= g_ota_manager.need_ota_device_count)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    switch (dev->state)
    {
    case OTA_STATE_IDLE:

        break;

    case OTA_STATE_INIT_WAIT_ACK:
        // 等待初始化应答，在ota_handle_timeout中处理
        break;

    case OTA_STATE_TRANSMITTING: // 从 WiFi 接收的 128 字节缓冲区中，取出当前需要发送的 64 字节 Sub1G 数据包，并发送给目标设备。
    {
        // 等待WiFi数据时检查超时
        if (g_ota_manager.waiting_wifi_data)
        {
            // WiFi数据请求超时检测（2秒超时）
            if (g_ota_manager.last_activity_ms > 2000)
            {
                DEBUG_PRINTF("[OTA] WiFi data request timeout, retry...\r\n");

                // 重新请求WiFi数据
                uint32_t packet_address = dev->current_packet * OTA_SUB1G_PACKET_SIZE;
                uint32_t request_address = (packet_address / OTA_FIRMWARE_BUFFER_SIZE) * OTA_FIRMWARE_BUFFER_SIZE;
                uint32_t remaining = g_ota_manager.fw_length - request_address;
                uint16_t request_length = (remaining >= OTA_FIRMWARE_BUFFER_SIZE) ? OTA_FIRMWARE_BUFFER_SIZE : (uint16_t)remaining;

                g_ota_manager.last_activity_ms = 0;

                ota_request_firmware_from_wifi(request_address, request_length);

                // 增加重试计数
                dev->retry_count++;
                if (dev->retry_count >= OTA_RETRY_MAX)
                {
                    DEBUG_PRINTF("[OTA] WiFi data request max retries reached\r\n");
                    ota_complete_device(g_ota_manager.current_device_index, false);
                }
            }
            break;
        }

        // 准备发送下一个数据包
        if (dev->current_packet < dev->total_packets)
        {
            // 计算当前包的固件地址和长度
            uint32_t packet_address = dev->current_packet * OTA_SUB1G_PACKET_SIZE;
            uint16_t packet_len = OTA_SUB1G_PACKET_SIZE;

            // 最后一包可能不足64字节
            if (packet_address + packet_len > g_ota_manager.fw_length)
            {
                packet_len = g_ota_manager.fw_length - packet_address;
            }

            // 检查缓冲区是否需要刷新
            if (packet_address < g_ota_manager.fw_current_address ||
                packet_address >= (g_ota_manager.fw_current_address + g_ota_manager.fw_buffer_valid_len))
            {
                // 需要从WiFi请求新的固件数据块
                // 计算实际需要的长度:优先请求128字节，如果剩余固件不足128字节,则请求实际剩余长度
                uint32_t request_address = (packet_address / OTA_FIRMWARE_BUFFER_SIZE) * OTA_FIRMWARE_BUFFER_SIZE;
                uint32_t remaining = g_ota_manager.fw_length - request_address;
                uint16_t request_length = (remaining >= OTA_FIRMWARE_BUFFER_SIZE) ? OTA_FIRMWARE_BUFFER_SIZE : (uint16_t)remaining;

                // 记录请求地址,WiFi回复时会用到
                g_ota_manager.fw_current_address = request_address;
                g_ota_manager.waiting_wifi_data = true;

                // 打印进度
                DEBUG_PRINTF("[OTA] progress: %d/%d addr=0x%06lX len=%d\r\n", dev->current_packet + 1, dev->total_packets, request_address, request_length);

                ota_request_firmware_from_wifi(request_address, request_length);
                break;
            }

            // 计算在缓冲区中的偏移
            uint16_t offset = packet_address - g_ota_manager.fw_current_address;

            // 发送数据包（packet_num从1开始）
            ota_send_sub1g_data_packet(dev->sub1g_addr, g_ota_manager.sub1g_type,
                                       dev->total_packets, dev->current_packet + 1,
                                       &g_ota_manager.fw_buffer[offset], packet_len);

            // 更新状态
            dev->state = OTA_STATE_WAIT_DATA_ACK;
            dev->timeout_counter = 0;
            g_ota_manager.last_activity_ms = 0;

            DEBUG_PRINTF("[OTA] Send packet[%d/%d] addr=0x%08lX len=%d\r\n",
                         dev->current_packet + 1, dev->total_packets, packet_address, packet_len);
        }
        else
        {
            // 所有包发送完成
            DEBUG_PRINTF("[OTA] Device[%d] all packets sent\r\n", g_ota_manager.current_device_index);

            // 根据固件类型决定后续流程
            if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
            {
                // CT Sub1G升级：等待设备重启后通过CMD_CT_SUB1G_VERSION上报版本
                DEBUG_PRINTF("[OTA] Wait CT Sub1G version report\r\n");
                dev->state = OTA_STATE_WAIT_CT_VERSION;
                dev->timeout_counter = 0;
                g_ota_manager.last_activity_ms = 0;
            }
            else
            {
                // 微逆升级：主动询问版本
                DEBUG_PRINTF("[OTA] Start query inverter version\r\n");
                dev->state = OTA_STATE_QUERY_VERSION;
                dev->timeout_counter = 0;
                dev->version_query_interval_counter = 0;
                g_ota_manager.last_activity_ms = 0;

                // 立即发送第一次版本询问
                ota_send_sub1g_query_version_cmd(dev->sub1g_addr);
            }
        }
        break;
    }

    case OTA_STATE_WAIT_DATA_ACK:
        // 等待数据应答，超时处理在ota_handle_timeout中
        break;

    case OTA_STATE_WAIT_CT_VERSION:
        // 等待CT Sub1G上报版本，超时处理在ota_handle_timeout中
        break;

    case OTA_STATE_QUERY_VERSION:
        // 主动询问微逆版本，每100ms询问一次
        dev->version_query_interval_counter++;
        if (dev->version_query_interval_counter >= OTA_VERSION_QUERY_INTERVAL_MS)
        {
            dev->version_query_interval_counter = 0;
            ota_send_sub1g_query_version_cmd(dev->sub1g_addr);
            DEBUG_PRINTF("[OTA] Query version: device[%d], timeout=%lums\r\n", g_ota_manager.current_device_index, g_ota_manager.last_activity_ms);
        }
        break;

    case OTA_STATE_COMPLETED:
    case OTA_STATE_FAILED:
    case OTA_STATE_CANCELLED:
        // 移动到下一个设备
        g_ota_manager.current_device_index++;
        if (g_ota_manager.current_device_index < g_ota_manager.need_ota_device_count)
        {
            ota_start_next_device();
        }
        else
        {
            // 当前轮次所有设备完成
            DEBUG_PRINTF("[OTA] Round %d completed: success=%d, failed=%d\r\n", g_ota_manager.retry_round, g_ota_manager.success_count, g_ota_manager.failed_count);

            // 检查是否有失败设备需要重试
            if (g_ota_manager.failed_device_count > 0 && g_ota_manager.retry_round < OTA_DEVICE_RETRY_MAX)
            {
                // 获取当前状态
                ota_result_t current_status = ota_get_finish_status();

                // 如果是部分成功，递增固件级重试计数
                if (current_status == OTA_RESULT_PARTIAL_SUCCESS)
                {
                    g_ota_manager.firmware_retry_count++;
                    DEBUG_PRINTF("[OTA] Partial success, firmware_retry_count=%d/%d\r\n",
                                 g_ota_manager.firmware_retry_count, OTA_FIRMWARE_RETRY_MAX);

                    // 检查是否达到固件级重试上限
                    if (g_ota_manager.firmware_retry_count >= OTA_FIRMWARE_RETRY_MAX)
                    {
                        DEBUG_PRINTF("[OTA] Firmware retry limit reached, force fail all remaining devices\r\n");

                        // 停止OTA进程
                        g_ota_manager.ota_in_progress = false;
                        g_ota_manager.disable_broadcast = false;
                        g_ota_manager.disable_property_report = false;

                        // 发送完全失败状态给WiFi
                        ota_send_finish_result_to_wifi(OTA_RESULT_FAILED);

                        // 不执行版本更新（因为不是真正的成功）
                        return; // 直接返回，不继续后续流程
                    }
                }

                // 未达到上限，继续重试
                ota_retry_failed_devices();
            }
            else
            {
                // 全部完成或达到最大设备重试次数
                DEBUG_PRINTF("\r\n[OTA] All upgrade attempts completed: total_success=%d, total_failed=%d\r\n", g_ota_manager.success_count, g_ota_manager.failed_count);
                g_ota_manager.ota_in_progress = false;
                g_ota_manager.disable_broadcast = false;
                g_ota_manager.disable_property_report = false;

                // 发送完成状态给WiFi
                ota_result_t finish_status = ota_get_finish_status();
                ota_send_finish_result_to_wifi(finish_status);

                // 如果有设备升级成功，更新版本并上报
                if (g_ota_manager.success_count > 0)
                {
                    if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
                    {
                        // CT Sub1G升级：版本已通过CMD_CT_SUB1G_VERSION更新
                        DEBUG_PRINTF("[OTA] CT Sub1G OTA success, version already updated\r\n");
                    }
                    else if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G)
                    {
                        DEBUG_PRINTF("[OTA] Inv Sub1G OTA success, slave version updated\r\n");
                        // update_slave_versions();
                        // tx_flag.slave_version = true;
                    }
                    else if (g_ota_manager.fw_type == FW_TYPE_INV_800W)
                    {
                        DEBUG_PRINTF("[OTA] 800W_Inv OTA success, slave version updated\r\n");
                        // update_slave_versions();
                        // tx_flag.slave_version = true;
                    }
                    else if (g_ota_manager.fw_type == FW_TYPE_INV_2500W)
                    {
                        DEBUG_PRINTF("[OTA] 2500W_Inv OTA success, slave version updated\r\n");
                        // update_slave_versions();
                        // tx_flag.slave_version = true;
                    }
                }
            }
        }
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : static void ota_handle_timeout(void)
 Input       : 无
 Output      : 无
 Description : 在 CT 设备向 Sub1G 设备发送命令（如初始化或数据包）后，处理因目标设备无应答而导致的超时和重试逻辑
---------------------------------------------------------------------------*/
static void ota_handle_timeout(void)
{
    static uint8_t timeout_print_counter = 30; // 添加静态计数器

    if (g_ota_manager.current_device_index >= g_ota_manager.need_ota_device_count)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    // 检查是否超时（根据状态使用不同的超时时间）
    uint32_t timeout_threshold = OTA_TIMEOUT_MS;
    if (dev->state == OTA_STATE_QUERY_VERSION || dev->state == OTA_STATE_WAIT_CT_VERSION)
    {
        timeout_threshold = OTA_VERSION_QUERY_TIMEOUT_MS;
    }

    if (g_ota_manager.last_activity_ms < timeout_threshold)
    {
        return;
    }

    // 超时处理
    timeout_print_counter++;
    if (timeout_print_counter >= 2) // 每10次超时打印一次
    {
        if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
        {
            DEBUG_PRINTF("[OTA] Timeout: CT Sub1G, State=%d\r\n", dev->state);
        }
        else if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G)
        {
            DEBUG_PRINTF("[OTA] Timeout: Inv Sub1G 0x%06X, State=%d\r\n", dev->sub1g_addr, dev->state);
        }
        else if (g_ota_manager.fw_type == FW_TYPE_INV_800W)
        {
            DEBUG_PRINTF("[OTA] Timeout: 800W_Inv 0x%06X, State=%d\r\n", dev->sub1g_addr, dev->state);
        }
        else if (g_ota_manager.fw_type == FW_TYPE_INV_2500W)
        {
            DEBUG_PRINTF("[OTA] Timeout: 2500W_Inv 0x%06X, State=%d\r\n", dev->sub1g_addr, dev->state);
        }

        timeout_print_counter = 0;
    }

    switch (dev->state)
    {
    case OTA_STATE_INIT_WAIT_ACK:
    case OTA_STATE_WAIT_DATA_ACK:
        dev->retry_count++;

        if (dev->retry_count >= OTA_RETRY_MAX)
        {
            DEBUG_PRINTF("[OTA] Max retries reached, device failed\r\n");
            ota_complete_device(g_ota_manager.current_device_index, false);
        }
        else
        {
            DEBUG_PRINTF("[OTA] Command retry %d/%d\r\n", dev->retry_count, OTA_RETRY_MAX);

            if (dev->state == OTA_STATE_INIT_WAIT_ACK)
            {
                // 重发初始化命令
                ota_send_sub1g_init_cmd(dev->sub1g_addr, g_ota_manager.sub1g_type, g_ota_manager.fw_length);
            }
            else
            {
                // 重发数据包
                dev->state = OTA_STATE_TRANSMITTING;
            }

            g_ota_manager.last_activity_ms = 0;
        }
        break;

    case OTA_STATE_QUERY_VERSION:
        // 微逆版本询问超时，认为升级失败
        DEBUG_PRINTF("[OTA] Inverter version query timeout, device upgrade failed\r\n");
        ota_complete_device(g_ota_manager.current_device_index, false);
        break;

    case OTA_STATE_WAIT_CT_VERSION:
        // CT Sub1G版本上报超时，认为升级失败
        DEBUG_PRINTF("[OTA] CT Sub1G version report timeout, device upgrade failed\r\n");
        ota_complete_device(g_ota_manager.current_device_index, false);
        break;

    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : static void ota_complete_device(uint8_t device_idx, bool success)
 Input       : device_idx - 设备索引
               success - 是否成功
 Output      : 无
 Description : 完成设备升级，记录成功/失败，失败设备加入重试列表
---------------------------------------------------------------------------*/
static void ota_complete_device(uint8_t device_idx, bool success)
{
    ota_device_info_t *dev = &g_ota_manager.devices[device_idx];

    if (success)
    {
        dev->state = OTA_STATE_COMPLETED;
        dev->ota_send_success = true;
        g_ota_manager.success_count++;
        DEBUG_PRINTF("[OTA] Device[%d] SUCCESS (sub1g_addr=0x%06lX)\r\n", device_idx, dev->sub1g_addr);
    }
    else
    {
        dev->state = OTA_STATE_FAILED;
        g_ota_manager.failed_count++;

        // 添加到失败列表
        if (g_ota_manager.failed_device_count < INV_DEVICE_MAX_NUM)
        {
            g_ota_manager.failed_device_list[g_ota_manager.failed_device_count++] = device_idx;
            DEBUG_PRINTF("[OTA] Device[%d] FAILED (sub1g_addr=0x%06lX), added to retry list\r\n", device_idx, dev->sub1g_addr);
        }
    }
}

// ================= OTA与wifi之间的互动函数 =================

/*---------------------------------------------------------------------------
 Name        : static void ota_request_firmware_from_wifi(uint32_t address, uint16_t length)
 Input       : address - 固件地址
                length - 请求长度(实际需要的长度,不强制128字节)
 Output      : 无
 Description : 向WiFi请求固件数据
---------------------------------------------------------------------------*/
static void ota_request_firmware_from_wifi(uint32_t address, uint16_t length)
{
    DEBUG_PRINTF("[OTA] Request firmware from WiFi: addr=0x%06lX, len=%d\r\n", address, length);

    // 构建serial_msg_t消息
    msg_output.type = 0; // 0-请求
    msg_output.cmd = 0x1001;
    msg_output.code = 0;
    msg_output.cmd_data_length = 6;

    msg_output.cmd_data[0] = (address >> 24) & 0xFF;
    msg_output.cmd_data[1] = (address >> 16) & 0xFF;
    msg_output.cmd_data[2] = (address >> 8) & 0xFF;
    msg_output.cmd_data[3] = address & 0xFF;
    msg_output.cmd_data[4] = (length >> 8) & 0xFF;
    msg_output.cmd_data[5] = length & 0xFF;

    // 发送消息
    serial_msg_send(&msg_output);
}

/*---------------------------------------------------------------------------
 Name        : static void ota_send_finish_result_to_wifi(uint8_t result)
 Input       : result - OTA完成结果
                        OTA_RESULT_SUCCESS_ALL - 全部成功
                        OTA_RESULT_PARTIAL_SUCCESS - 部分成功
                        OTA_RESULT_FAILED - 全部失败
 Output      : 无
 Description : 向WiFi发送OTA完成结果，发送CMD=1002，格式{1字节result}
---------------------------------------------------------------------------*/
static void ota_send_finish_result_to_wifi(ota_result_t result)
{
    DEBUG_PRINTF("[OTA] Send finish result to WiFi: result=%d\r\n", result);

    // 构建serial_msg_t消息
    msg_output.type = 0;
    msg_output.cmd = 0x1002;
    msg_output.code = 0;
    msg_output.cmd_data_length = 1;
    msg_output.cmd_data[0] = result;

    // 发送消息
    serial_msg_send(&msg_output);
}

/*---------------------------------------------------------------------------
 Name        : void ota_copy_wifi_fw_data(...)
 Input       : data - 固件数据
               length - 数据长度
 Output      : 无
 Description : 处理WiFi发来的固件数据(CMD=1001应答)
               在mmi.c的serial_msg_parse解析完成后,收到CMD=1001,TYPE=1时调用
               WiFi数据格式: {DATA:N*BYTE} - 纯二进制数据
---------------------------------------------------------------------------*/
void ota_copy_wifi_fw_data(const uint8_t *data, uint16_t length)
{
    if (!g_ota_manager.ota_in_progress)
    {
        DEBUG_PRINTF("[OTA] Not in progress, ignore data\r\n");
        return;
    }

    // 验证长度
    if (length > OTA_FIRMWARE_BUFFER_SIZE)
    {
        DEBUG_PRINTF("[OTA] Invalid length: len=%d\r\n", length);
        return;
    }

    // 保存到缓冲区
    g_ota_manager.fw_buffer_valid_len = length;
    g_ota_manager.waiting_wifi_data = false;
    memcpy(g_ota_manager.fw_buffer, data, length);

    // 重置当前设备的WiFi请求重试计数器
    if (g_ota_manager.current_device_index < g_ota_manager.need_ota_device_count)
    {
        g_ota_manager.devices[g_ota_manager.current_device_index].retry_count = 0;
    }

    DEBUG_PRINTF("[OTA] Received firmware data: addr=%lu, len=%d\r\n", g_ota_manager.fw_current_address, length);
}

/*---------------------------------------------------------------------------
 Name        : sub1g_ota_type_t ota_fw_type_to_sub1g_type(ota_fw_type_t fw_type)
 Input       : fw_type - 固件类型
 Output      : Sub1G OTA类型
 Description : 将固件类型转换为Sub1G OTA类型
---------------------------------------------------------------------------*/
sub1g_ota_type_t ota_fw_type_to_sub1g_type(ota_fw_type_t fw_type)
{
    switch (fw_type)
    {
    case FW_TYPE_CT_SUB1G:
        return SUB1G_OTA_TYPE_CT_SUB1G;
    case FW_TYPE_INV_SUB1G:
        return SUB1G_OTA_TYPE_INV_SUB1G;
    case FW_TYPE_INV_800W:
        return SUB1G_OTA_TYPE_INV_800W;
    case FW_TYPE_INV_2500W:
        return SUB1G_OTA_TYPE_INV_2500W;
    default:
        return SUB1G_OTA_TYPE_CT_SUB1G;
    }
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_init_request(void)
 Input       : 无
 Output      : 无
 Description : 获取OTA完成状态
---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 Name        : ota_result_t ota_get_finish_status(void)
 Input       : 无
 Output      : OTA结果状态
 Description : 获取OTA完成状态
               - 全部成功 (result=0)：所有原始需要升级的设备都成功
               - 部分成功 (result=1)：有部分设备成功，但不是全部
               - 全部失败 (result=2)：所有设备都失败
               注意：使用original_device_count而非need_ota_device_count，
                     因为后者在重试时会被更新
---------------------------------------------------------------------------*/
ota_result_t ota_get_finish_status(void)
{
    // 使用原始设备数判断结果（重试时need_ota_device_count会更新）
    if (g_ota_manager.success_count == g_ota_manager.original_device_count)
    {
        // 所有原始需要升级的设备都成功了
        return OTA_RESULT_SUCCESS_ALL;
    }
    else if (g_ota_manager.success_count > 0)
    {
        // 有部分设备成功，但不是全部
        return OTA_RESULT_PARTIAL_SUCCESS;
    }
    else
    {
        // 所有设备都失败了
        return OTA_RESULT_FAILED;
    }
}

// ================= OTA与sub1g之间的互动回复函数 =================

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_init_ack(uint32_t sub1g_addr, uint8_t status)
 Input       : sub1g_addr - Sub1G设备地址
               status - 初始化状态 (0-成功, 其他-失败)
 Output      : 无
 Description : 处理Sub1G初始化应答 (0x71)
---------------------------------------------------------------------------*/
void ota_handle_sub1g_init_ack(uint32_t sub1g_addr, uint8_t status)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    if (dev->sub1g_addr != sub1g_addr)
    {
        DEBUG_PRINTF("[OTA] Init ACK from unexpected device: addr=0x%06lX\r\n", sub1g_addr);
        return;
    }

    DEBUG_PRINTF("[OTA] CMD 0x70 ACK: device[%d] status=%d\r\n\r\n", g_ota_manager.current_device_index, status);

    if (status != 0)
    {
        DEBUG_PRINTF("[OTA] Init failed with status=%d\r\n", status);
        ota_complete_device(g_ota_manager.current_device_index, false);
        return;
    }

    // 初始化成功，开始传输
    dev->state = OTA_STATE_TRANSMITTING;
    dev->retry_count = 0;
    g_ota_manager.last_activity_ms = 0;
}

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_data_ack(uint32_t sub1g_addr, uint16_t packet_num, uint8_t status)
 Input       : sub1g_addr - Sub1G设备地址
               packet_num - 数据包序号（从1开始）
               status - 数据包状态 (0-成功, 其他-失败)
 Output      : 无
 Description : 处理Sub1G数据应答 (0x73)
---------------------------------------------------------------------------*/
void ota_handle_sub1g_data_ack(uint32_t sub1g_addr, uint16_t packet_num, uint8_t status)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    if (dev->sub1g_addr != sub1g_addr)
    {
        DEBUG_PRINTF("[OTA] Data ACK from unexpected device: addr=0x%06lX\r\n", sub1g_addr);
        return;
    }

    // packet_num从1开始，需要转换为从0开始的索引
    if (dev->current_packet != packet_num - 1)
    {
        DEBUG_PRINTF("[OTA] Packet mismatch: expected %d, got %d\r\n", dev->current_packet + 1, packet_num);
        return;
    }

    DEBUG_PRINTF("[OTA] CMD 0x73 ACK: packet=%d, status=%d\r\n", packet_num, status);

    if (status != 0)
    {
        DEBUG_PRINTF("[OTA] Data packet failed: status=%d\r\n", status);
        dev->retry_count++;

        if (dev->retry_count >= OTA_RETRY_MAX)
        {
            ota_complete_device(g_ota_manager.current_device_index, false);
        }
        else
        {
            DEBUG_PRINTF("[OTA] Packet retry %d/%d\r\n", dev->retry_count, OTA_RETRY_MAX);
            dev->state = OTA_STATE_TRANSMITTING;
        }
        DEBUG_PRINTF("\r\n");
        return;
    }

    DEBUG_PRINTF("\r\n");

    // 成功，移动到下一包
    dev->retry_count = 0;
    dev->current_packet++;
    dev->state = OTA_STATE_TRANSMITTING;
    g_ota_manager.last_activity_ms = 0;
}

// ================= OTA与sub1g之间的互动发送函数 =================

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_cancel(uint32_t sub1g_addr)
 Input       : sub1g_addr - Sub1G设备地址
 Output      : 无
 Description : 取消Sub1G设备的升级 (0x75)
---------------------------------------------------------------------------*/
void ota_handle_sub1g_cancel(uint32_t sub1g_addr)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    if (dev->sub1g_addr == sub1g_addr)
    {
        DEBUG_PRINTF("[OTA] Device[%d] cancelled upgrade\r\n", g_ota_manager.current_device_index);
        dev->state = OTA_STATE_CANCELLED;
        ota_complete_device(g_ota_manager.current_device_index, false);
    }
}

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_version_report(uint32_t sub1g_addr, const char *sub1g_version, const char *mcu_version)
 Input       : sub1g_addr - Sub1G设备地址
               sub1g_version - Sub1G版本号
               mcu_version - MCU版本号
 Output      : 无
 Description : 处理微逆设备上报版本号 (0x77)，用于OTA升级完成后确认设备版本
               判断版本号是否与目标版本一致，一致则升级成功并更新版本信息，不一致则失败
               注意：此函数只处理微逆设备，不处理CT Sub1G
---------------------------------------------------------------------------*/
void ota_handle_sub1g_version_report(uint32_t sub1g_addr, const char *sub1g_version, const char *mcu_version)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    // 检查是否是当前正在升级的设备
    if (dev->sub1g_addr != sub1g_addr)
    {
        return;
    }

    // 检查是否处于询问版本状态（只处理微逆）
    if (dev->state != OTA_STATE_QUERY_VERSION)
    {
        return;
    }

    DEBUG_PRINTF("[OTA] Device[%d] version report: sub1g=%s, mcu=%s\r\n",
                 g_ota_manager.current_device_index,
                 sub1g_version ? sub1g_version : "NULL",
                 mcu_version ? mcu_version : "NULL");

    // 判断版本号是否一致
    bool version_match = false;
    const char *reported_version = NULL;

    if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G)
    {
        // 微逆Sub1G固件升级，比对Sub1G版本
        reported_version = sub1g_version;
    }
    else if (g_ota_manager.fw_type == FW_TYPE_INV_800W || g_ota_manager.fw_type == FW_TYPE_INV_2500W)
    {
        // 微逆MCU固件升级，比对MCU版本
        reported_version = mcu_version;
    }
    else
    {
        // 不应该到这里（FW_TYPE_CT_SUB1G不使用0x77）
        DEBUG_PRINTF("[OTA] ERROR: Unexpected fw_type=%d in version report\r\n", g_ota_manager.fw_type);
        return;
    }

    if (reported_version && strcmp(reported_version, g_ota_manager.fw_version) == 0)
    {
        version_match = true;
        DEBUG_PRINTF("[OTA] Version match! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version, reported_version);

        // 版本匹配，更新sys_param中的微逆版本信息
        uint8_t dev_idx = dev->device_index;

        if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G && sub1g_version)
        {
            // 更新微逆Sub1G版本
            strncpy(sys_param.paired_inv_info[dev_idx].sub1g_version, sub1g_version, VERSION_STRING_MAX_LEN);
            sys_param.paired_inv_info[dev_idx].sub1g_version[VERSION_STRING_MAX_LEN] = '\0';
            DEBUG_PRINTF("[OTA] Update Inv[%d] Sub1G version: %s\r\n", dev_idx, sys_param.paired_inv_info[dev_idx].sub1g_version);
        }
        else if ((g_ota_manager.fw_type == FW_TYPE_INV_800W || g_ota_manager.fw_type == FW_TYPE_INV_2500W) && mcu_version)
        {
            // 更新微逆MCU版本
            strncpy(sys_param.paired_inv_info[dev_idx].sw_version, mcu_version, VERSION_STRING_MAX_LEN);
            sys_param.paired_inv_info[dev_idx].sw_version[VERSION_STRING_MAX_LEN] = '\0';
            DEBUG_PRINTF("[OTA] Update Inv[%d] MCU version: %s\r\n", dev_idx, sys_param.paired_inv_info[dev_idx].sw_version);
        }
    }
    else
    {
        DEBUG_PRINTF("[OTA] Version mismatch! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version,
                     reported_version ? reported_version : "NULL");
    }

    // 根据版本比对结果完成升级
    ota_complete_device(g_ota_manager.current_device_index, version_match);
}

/*---------------------------------------------------------------------------
 Name        : void ota_handle_ct_sub1g_version_report(const char *ct_sub1g_version)
 Input       : ct_sub1g_version - CT Sub1G版本号
 Output      : 无
 Description : 处理CT Sub1G上报版本号，用于OTA升级完成后确认版本
               判断版本号是否与目标版本一致，一致则升级成功，不一致则失败
               注意：CT Sub1G通过CMD_CT_SUB1G_VERSION (0x41)上报版本，而非0x77
---------------------------------------------------------------------------*/
void ota_handle_ct_sub1g_version_report(const char *ct_sub1g_version)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    // 只有CT Sub1G升级时才处理
    if (g_ota_manager.fw_type != FW_TYPE_CT_SUB1G)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    // 检查是否处于等待CT版本状态
    if (dev->state != OTA_STATE_WAIT_CT_VERSION)
    {
        return;
    }

    DEBUG_PRINTF("[OTA] CT Sub1G version report: %s\r\n", ct_sub1g_version ? ct_sub1g_version : "NULL");

    // 判断版本号是否一致
    bool version_match = false;
    if (ct_sub1g_version && strcmp(ct_sub1g_version, g_ota_manager.fw_version) == 0)
    {
        version_match = true;
        DEBUG_PRINTF("[OTA] CT Sub1G version match! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version, ct_sub1g_version);

        // 版本匹配，更新sys_param中的CT Sub1G版本信息
        strncpy(sys_param.sub1g.sw_version, ct_sub1g_version, VERSION_STRING_MAX_LEN);
        sys_param.sub1g.sw_version[VERSION_STRING_MAX_LEN] = '\0';
        DEBUG_PRINTF("[OTA] Update CT Sub1G version: %s\r\n", sys_param.sub1g.sw_version);
    }
    else
    {
        DEBUG_PRINTF("[OTA] CT Sub1G version mismatch! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version, ct_sub1g_version ? ct_sub1g_version : "NULL");
    }

    // 根据版本比对结果完成升级
    ota_complete_device(g_ota_manager.current_device_index, version_match);
}

/*---------------------------------------------------------------------------
 Name        : void ota_force_cancel(void)
 Input       : 无
 Output      : 无
 Description : 向Sub1G设备发送取消升级命令
---------------------------------------------------------------------------*/
void ota_force_cancel(void)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    DEBUG_PRINTF("[OTA] Force cancel all upgrades\r\n");

    // 向所有设备发送取消命令
    for (uint8_t i = 0; i < g_ota_manager.need_ota_device_count; i++)
    {
        if (g_ota_manager.devices[i].state != OTA_STATE_COMPLETED &&
            g_ota_manager.devices[i].state != OTA_STATE_FAILED &&
            g_ota_manager.devices[i].state != OTA_STATE_CANCELLED)
        {
            ota_send_sub1g_cancel_cmd(g_ota_manager.devices[i].sub1g_addr,
                                      g_ota_manager.sub1g_type);
            g_ota_manager.devices[i].state = OTA_STATE_CANCELLED;
        }
    }

    g_ota_manager.ota_in_progress = false;
    g_ota_manager.disable_broadcast = false;
    g_ota_manager.disable_property_report = false;
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_init_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type, uint32_t length)
 Input       : sub1g_addr - Sub1G设备地址
               type - OTA类型
               length - 固件长度
 Output      :
 Description : 发送Sub1G OTA初始化命令 (0x70)
---------------------------------------------------------------------------*/
void ota_send_sub1g_init_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type, uint32_t length)
{
    uint8_t buffer[16];
    uint16_t index = 0;

    // 帧头
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G地址（3字节）
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // 长度（命令码1B + 数据8B  = 9B）
    buffer[index++] = 9;

    // 命令码
    buffer[index++] = SUB1G_OTA_CMD_INIT;

    // 类型（1字节）
    buffer[index++] = type;

    // Sub1G地址（3字节）
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // 固件长度（4字节）
    buffer[index++] = (length >> 24) & 0xFF;
    buffer[index++] = (length >> 16) & 0xFF;
    buffer[index++] = (length >> 8) & 0xFF;
    buffer[index++] = length & 0xFF;

    // 发送到UART1队列
    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] Send Sub1g init cmd: addr=0x%06lX, type=0x%02X, len=%lu\r\n", sub1g_addr, type, length);
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_data_packet(...)
 Input       : sub1g_addr - 目标设备地址
               type - OTA类型
               total_packets - 总包数
               packet_num - 当前包序号（从1开始）
               data - 数据内容
               data_len - 数据长度
 Output      : 无
 Description : 发送Sub1G OTA数据包 (0x72)
               长度 = 命令码(1) + 类型(1) + 总片数(2) + 第几片(2) + 内容(64) + 校验(2) = 72
               校验码从长度项开始计算71字节
---------------------------------------------------------------------------*/
void ota_send_sub1g_data_packet(uint32_t sub1g_addr, sub1g_ota_type_t type, uint16_t total_packets, uint16_t packet_num, const uint8_t *data, uint8_t data_len)
{
    uint8_t buffer[UART1_TX_MSG_MAX_LEN - 1];
    uint16_t index = 0;

    // 帧头
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G地址（3字节）
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // CRC计算起点
    uint16_t checksum_start_pos = index;

    // 长度：72（命令码1+类型1+总片数2+第几片2+内容64+校验码2）
    buffer[index++] = 72;

    // 命令码
    buffer[index++] = SUB1G_OTA_CMD_DATA;

    // 类型（1字节）
    buffer[index++] = type;

    // 总片数（2字节）
    buffer[index++] = (total_packets >> 8) & 0xFF;
    buffer[index++] = total_packets & 0xFF;

    // 第几片（2字节）
    buffer[index++] = (packet_num >> 8) & 0xFF;
    buffer[index++] = packet_num & 0xFF;

    // 数据：固定64字节（不足填充0xFF）
    memcpy(&buffer[index], data, data_len);
    if (data_len < 64)
    {
        memset(&buffer[index + data_len], 0xFF, 64 - data_len);
    }
    index += 64;

    // CRC：从长度字段开始计算71字节
    uint16_t checksum = ota_calculate_checksum(&buffer[checksum_start_pos], 71);

    // CRC（2字节）
    buffer[index++] = (checksum >> 8) & 0xFF;
    buffer[index++] = checksum & 0xFF;

    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] TX: pkt=%d/%d, len=%d, crc=0x%04X\r\n",
                 packet_num, total_packets, data_len, checksum);
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_cancel_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type)
 Input       : sub1g_addr - 目标设备地址
               type - OTA类型
 Output      : 无
 Description : 发送Sub1G取消升级命令 (0x74)
---------------------------------------------------------------------------*/
void ota_send_sub1g_cancel_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type)
{
    uint8_t buffer[16];
    uint16_t index = 0;

    // 帧头
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G地址（3字节）
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // 长度（命令码1B + 类型1B + 地址3B = 5B）
    buffer[index++] = 5;

    // 命令码
    buffer[index++] = SUB1G_OTA_CMD_CANCEL_FROM_CT;

    // 类型（1字节）
    buffer[index++] = type;

    // Sub1G地址（3字节，重复）
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // 发送到UART1队列
    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] Send cancel cmd: addr=0x%06lX, type=0x%02X\r\n", sub1g_addr, type);
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr)
 Input       : sub1g_addr - 目标设备地址
 Output      : 无
 Description : 发送Sub1G询问版本命令 (0x76)
---------------------------------------------------------------------------*/
void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr)
{
    uint8_t buffer[16];
    uint16_t index = 0;

    // 帧头
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G地址（3字节）
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // 长度（命令码1B）
    buffer[index++] = 1;

    // 命令码
    buffer[index++] = SUB1G_OTA_CMD_QUERY_VERSION;

    // 发送到UART1队列
    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] Send query version cmd: addr=0x%06lX\r\n", sub1g_addr);
}

/*---------------------------------------------------------------------------
|| Name        : calculate_crc16
|| Input       : data - 数据指针, length - 数据长度
|| Output      : CRC16校验值
|| Description : 计算CRC16校验值（CRC-16/XMODEM）
---------------------------------------------------------------------------*/
uint16_t ota_calculate_checksum(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0x0000;
    uint16_t i;
    uint8_t j;

    for (i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}
