#include "ota_max.h"
#include "sub1g.h"
#include "mmi.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// ================= ȫ�ֱ������� =================
ota_manager_t g_ota_manager = {0};

// ================= �ڲ��������� =================
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
 Description : OTA管理器初始化。
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
 Description : OTA管理器任务，非阻塞状态机。
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

        // ���ӳ�ʱ������
        g_ota_manager.last_activity_ms++;

        // ������ǰ�豸
        ota_process_current_device();

        // ��鳬ʱ
        ota_handle_timeout();
    }
}

/*---------------------------------------------------------------------------
 Name        : ota_start_result_t ota_check_start_status(void)
 Input       : 无
 Output      : OTA启动结果
 Description : 检查OTA启动状态。
---------------------------------------------------------------------------*/
ota_start_result_t ota_check_start_status(void)
{
    DEBUG_PRINTF("[OTA] Check start status: fw_type=%d, length=%lu, version=%s, crc=0x%08lX\r\n",
                 g_ota_manager.fw_type, g_ota_manager.fw_length, g_ota_manager.fw_version, g_ota_manager.fw_crc);

    // ����Ƿ����ϱ�slave�汾
    if (!sys_param.slave_version.slave_version_reported)
    {
        DEBUG_PRINTF("[OTA] Necessary all versions not collected yet, return error\r\n");
        return OTA_START_WAIT; // �汾δ�ռ���ɣ�������OTA
    }

    // ����Ƿ����ڽ���OTA
    if (g_ota_manager.ota_in_progress)
    {
        DEBUG_PRINTF("[OTA] Already in progress, return busy\r\n");
        return OTA_START_BUSY; // ��æ
    }

    // ��֤�̼�����
    if (g_ota_manager.fw_type < FW_TYPE_CT_SUB1G || g_ota_manager.fw_type > FW_TYPE_INV_2500W)
    {
        DEBUG_PRINTF("[OTA] Invalid fw_type=%d\r\n", g_ota_manager.fw_type);
        return OTA_START_ERROR; // ����
    }

    // ����OTA������״̬
    g_ota_manager.ota_in_progress = true;
    g_ota_manager.current_device_index = 0;
    g_ota_manager.success_count = 0;
    g_ota_manager.failed_count = 0;
    g_ota_manager.failed_device_count = 0;
    g_ota_manager.firmware_retry_count = 0;

    // ת���̼�����ΪSub1G�̼�����Э������
    g_ota_manager.sub1g_type = ota_fw_type_to_sub1g_type(g_ota_manager.fw_type);

    // ����Sub1GЭ���ܰ�������64�ֽڷְ���
    g_ota_manager.total_packets_sub1g = (g_ota_manager.fw_length + OTA_SUB1G_PACKET_SIZE - 1) / OTA_SUB1G_PACKET_SIZE;

    // ��ʼ���豸�б�
    ota_device_list_init();

    if (g_ota_manager.need_ota_device_count == 0)
    {
        DEBUG_PRINTF("[OTA] All devices version matched, no need to upgrade\r\n");
        g_ota_manager.ota_in_progress = false;

        // �汾һ�£���Ϊȫ���ɹ������������Ϣ��WiFi
        ota_send_finish_result_to_wifi(OTA_RESULT_SUCCESS_ALL);

        // update_slave_versions();
        // tx_flag.slave_version = true;

        return OTA_START_READY; // ����READY��ʾ�Ѵ���(�汾��������)
    }

    // ����״̬��־
    g_ota_manager.disable_broadcast = true;
    g_ota_manager.disable_property_report = true;
    g_ota_manager.retry_round = 0;
    g_ota_manager.failed_device_count = 0;

    DEBUG_PRINTF("[OTA] Initialized: device_count=%d, total_packets=%d\r\n", g_ota_manager.need_ota_device_count, g_ota_manager.total_packets_sub1g);

    // ��ʼ��һ���豸������
    if (!ota_start_next_device())
    {
        DEBUG_PRINTF("[OTA] Failed to start first device\r\n");
        g_ota_manager.ota_in_progress = false;
        return OTA_START_ERROR; // ����
    }

    return OTA_START_READY; // ׼������
}

/*---------------------------------------------------------------------------
 Name        : static void ota_device_list_init(void)
 Input       : 无
 Output      : 无
 Description : OTA设备列表初始化。
---------------------------------------------------------------------------*/
static void ota_device_list_init(void)
{
    g_ota_manager.need_ota_device_count = 0;

    // �̼�����3��CT Sub1G�̼���ֻ��Ҫ����1���豸��CT�Լ���Sub1G��Լ��Ĭ��ʹ��111111��
    if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
    {
        // �汾�жϣ�����汾��ͬ����������
        if (strcmp(sys_param.sub1g.sw_version, g_ota_manager.fw_version) == 0)
        {
            DEBUG_PRINTF("[OTA] CT Sub1G version matched (%s), skip upgrade\r\n", sys_param.sub1g.sw_version);
            g_ota_manager.need_ota_device_count = 0;
            return;
        }

        g_ota_manager.devices[0].sub1g_addr = 0x111111; // Ĭ�ϵ�ַ
        g_ota_manager.devices[0].device_index = 0;
        g_ota_manager.devices[0].state = OTA_STATE_IDLE;
        g_ota_manager.devices[0].current_packet = 0;
        g_ota_manager.devices[0].total_packets = g_ota_manager.total_packets_sub1g;
        g_ota_manager.devices[0].retry_count = 0;
        g_ota_manager.devices[0].device_retry_count = 0;
        g_ota_manager.devices[0].version_query_interval_counter = 0;
        g_ota_manager.devices[0].ota_send_success = false;

        g_ota_manager.need_ota_device_count = 1;
        g_ota_manager.original_device_count = 1; // ����ԭʼ�豸��

        DEBUG_PRINTF("[OTA] CT Sub1G upgrade, device_count=1\r\n");
        return;
    }

    // �̼�����4/5/6��΢��Sub1G��MCU�̼�����Ҫ��������Ե�΢���豸
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // ����豸�Ƿ��Ѱ�
        if (sys_param.paired_inv_info[i].siid < SIID_MIN || sys_param.paired_inv_info[i].siid > SIID_MAX)
        {
            continue;
        }

        // ����豸�Ƿ����ߣ�ֻ���������豸�������豸��������
        if (sys_param.paired_inv_info[i].online_state != CT_STATUS_ONLINE)
        {
            DEBUG_PRINTF("[OTA] INV[%d] is offline (online_state=%d), skip\r\n", i, sys_param.paired_inv_info[i].online_state);
            continue;
        }

        bool version_matched = false;
        if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G)
        {
            // ΢���Sub1G�̼�
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
            // ΢���MCU�̼�
            if (strcmp(sys_param.paired_inv_info[i].sw_version, g_ota_manager.fw_version) == 0)
            {
                DEBUG_PRINTF("[OTA] INV[%d] MCU version matched (%s), skip\r\n", i, sys_param.paired_inv_info[i].sw_version);
                version_matched = true;
            }
        }

        // ����汾ƥ�䣬�������豸
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

    // ����ԭʼ�豸��
    g_ota_manager.original_device_count = g_ota_manager.need_ota_device_count;

    DEBUG_PRINTF("[OTA] Total devices to upgrade: %d\r\n", g_ota_manager.need_ota_device_count);
}

/*---------------------------------------------------------------------------
 Name        : static bool ota_start_next_device(void)
 Input       : 无
 Output      : 启动状态
 Description : 开始下一个设备的OTA升级。
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

    // �����豸״̬���������ԣ�
    dev->state = OTA_STATE_IDLE;
    dev->current_packet = 0;
    dev->retry_count = 0;
    dev->version_query_interval_counter = 0;

    // ���ͳ�ʼ������
    ota_send_sub1g_init_cmd(dev->sub1g_addr, g_ota_manager.sub1g_type, g_ota_manager.fw_length);

    // ����״̬
    dev->state = OTA_STATE_INIT_WAIT_ACK;
    dev->timeout_counter = 0;
    g_ota_manager.last_activity_ms = 0;

    // ����WiFi��������ȷ���ڿ�ʼ�������豸ʱ��ϵͳ׼���ôӹ̼�����ʼ��ַ����ַ 0���������ݣ������ȡ����wifi���ݺ�����
    g_ota_manager.fw_current_address = 0;
    g_ota_manager.fw_buffer_valid_len = 0;
    g_ota_manager.waiting_wifi_data = false;

    return true;
}

/*---------------------------------------------------------------------------
 Name        : static void ota_retry_failed_devices(void)
 Input       : 无
 Output      : 无
 Description : 重试失败的设备OTA升级。
---------------------------------------------------------------------------*/
static void ota_retry_failed_devices(void)
{
    g_ota_manager.retry_round++;

    DEBUG_PRINTF("[OTA] Starting retry round %d, failed_device_count=%d\r\n", g_ota_manager.retry_round, g_ota_manager.failed_device_count);

    // �ؽ��豸�б���ֻ����ʧ�ܵ��豸
    ota_device_info_t temp_devices[INV_DEVICE_MAX_NUM];
    uint8_t temp_count = 0;

    for (uint8_t i = 0; i < g_ota_manager.failed_device_count; i++)
    {
        uint8_t failed_idx = g_ota_manager.failed_device_list[i];

        // ����豸�Ƿ��Ѿ��ﵽ������Դ���
        if (g_ota_manager.devices[failed_idx].device_retry_count >= OTA_DEVICE_RETRY_MAX)
        {
            DEBUG_PRINTF("[OTA] Device[%d] reached max device retries, skip\r\n", failed_idx);
            continue;
        }

        if (g_ota_manager.fw_type > FW_TYPE_CT_SUB1G && g_ota_manager.fw_type <= FW_TYPE_INV_2500W)
        {
            // ����豸�Ƿ���Ȼ����
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

    // ��� temp_count > 0������Ҫ������һ�ֵ����ԣ���ô�����豸�б�
    memcpy(g_ota_manager.devices, temp_devices, sizeof(ota_device_info_t) * temp_count);
    g_ota_manager.need_ota_device_count = temp_count;
    g_ota_manager.current_device_index = 0;
    g_ota_manager.failed_device_count = 0;

    // ��ʼ��һ���豸������
    ota_start_next_device();
}

/*---------------------------------------------------------------------------
 Name        : static void ota_process_current_device(void)
 Input       : 无
 Output      : 无
 Description : 处理当前设备的OTA升级。
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
        // �ȴ���ʼ��Ӧ����ota_handle_timeout�д���
        break;

    case OTA_STATE_TRANSMITTING: // �� WiFi ���յ� 128 �ֽڻ������У�ȡ����ǰ��Ҫ���͵� 64 �ֽ� Sub1G ���ݰ��������͸�Ŀ���豸��
    {
        // �ȴ�WiFi����ʱ��鳬ʱ
        if (g_ota_manager.waiting_wifi_data)
        {
            // WiFi��������ʱ��⣨2�볬ʱ��
            if (g_ota_manager.last_activity_ms > 2000)
            {
                DEBUG_PRINTF("[OTA] WiFi data request timeout, retry...\r\n");

                // ��������WiFi����
                uint32_t packet_address = dev->current_packet * OTA_SUB1G_PACKET_SIZE;
                uint32_t request_address = (packet_address / OTA_FIRMWARE_BUFFER_SIZE) * OTA_FIRMWARE_BUFFER_SIZE;
                uint32_t remaining = g_ota_manager.fw_length - request_address;
                uint16_t request_length = (remaining >= OTA_FIRMWARE_BUFFER_SIZE) ? OTA_FIRMWARE_BUFFER_SIZE : (uint16_t)remaining;

                g_ota_manager.last_activity_ms = 0;

                ota_request_firmware_from_wifi(request_address, request_length);

                // �������Լ���
                dev->retry_count++;
                if (dev->retry_count >= OTA_RETRY_MAX)
                {
                    DEBUG_PRINTF("[OTA] WiFi data request max retries reached\r\n");
                    ota_complete_device(g_ota_manager.current_device_index, false);
                }
            }
            break;
        }

        // ׼��������һ�����ݰ�
        if (dev->current_packet < dev->total_packets)
        {
            // ���㵱ǰ���Ĺ̼���ַ�ͳ���
            uint32_t packet_address = dev->current_packet * OTA_SUB1G_PACKET_SIZE;
            uint16_t packet_len = OTA_SUB1G_PACKET_SIZE;

            // ���һ�����ܲ���64�ֽ�
            if (packet_address + packet_len > g_ota_manager.fw_length)
            {
                packet_len = g_ota_manager.fw_length - packet_address;
            }

            // ��黺�����Ƿ���Ҫˢ��
            if (packet_address < g_ota_manager.fw_current_address ||
                packet_address >= (g_ota_manager.fw_current_address + g_ota_manager.fw_buffer_valid_len))
            {
                // ��Ҫ��WiFi�����µĹ̼����ݿ�
                // ����ʵ����Ҫ�ĳ���:��������128�ֽڣ����ʣ��̼�����128�ֽ�,������ʵ��ʣ�೤��
                uint32_t request_address = (packet_address / OTA_FIRMWARE_BUFFER_SIZE) * OTA_FIRMWARE_BUFFER_SIZE;
                uint32_t remaining = g_ota_manager.fw_length - request_address;
                uint16_t request_length = (remaining >= OTA_FIRMWARE_BUFFER_SIZE) ? OTA_FIRMWARE_BUFFER_SIZE : (uint16_t)remaining;

                // ��¼�����ַ,WiFi�ظ�ʱ���õ�
                g_ota_manager.fw_current_address = request_address;
                g_ota_manager.waiting_wifi_data = true;

                // ��ӡ����
                DEBUG_PRINTF("[OTA] progress: %d/%d addr=0x%06lX len=%d\r\n", dev->current_packet + 1, dev->total_packets, request_address, request_length);

                ota_request_firmware_from_wifi(request_address, request_length);
                break;
            }

            // �����ڻ������е�ƫ��
            uint16_t offset = packet_address - g_ota_manager.fw_current_address;

            // �������ݰ���packet_num��1��ʼ��
            ota_send_sub1g_data_packet(dev->sub1g_addr, g_ota_manager.sub1g_type,
                                       dev->total_packets, dev->current_packet + 1,
                                       &g_ota_manager.fw_buffer[offset], packet_len);

            // ����״̬
            dev->state = OTA_STATE_WAIT_DATA_ACK;
            dev->timeout_counter = 0;
            g_ota_manager.last_activity_ms = 0;

            DEBUG_PRINTF("[OTA] Send packet[%d/%d] addr=0x%08lX len=%d\r\n",
                         dev->current_packet + 1, dev->total_packets, packet_address, packet_len);
        }
        else
        {
            // ���а��������
            DEBUG_PRINTF("[OTA] Device[%d] all packets sent\r\n", g_ota_manager.current_device_index);

            // ���ݹ̼����;�����������
            if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
            {
                // CT Sub1G�������ȴ��豸������ͨ��CMD_CT_SUB1G_VERSION�ϱ��汾
                DEBUG_PRINTF("[OTA] Wait CT Sub1G version report\r\n");
                dev->state = OTA_STATE_WAIT_CT_VERSION;
                dev->timeout_counter = 0;
                g_ota_manager.last_activity_ms = 0;
            }
            else
            {
                // ΢������������ѯ�ʰ汾
                DEBUG_PRINTF("[OTA] Start query inverter version\r\n");
                dev->state = OTA_STATE_QUERY_VERSION;
                dev->timeout_counter = 0;
                dev->version_query_interval_counter = 0;
                g_ota_manager.last_activity_ms = 0;

                // �������͵�һ�ΰ汾ѯ��
                ota_send_sub1g_query_version_cmd(dev->sub1g_addr);
            }
        }
        break;
    }

    case OTA_STATE_WAIT_DATA_ACK:
        // �ȴ�����Ӧ�𣬳�ʱ������ota_handle_timeout��
        break;

    case OTA_STATE_WAIT_CT_VERSION:
        // �ȴ�CT Sub1G�ϱ��汾����ʱ������ota_handle_timeout��
        break;

    case OTA_STATE_QUERY_VERSION:
        // ����ѯ��΢��汾��ÿ100msѯ��һ��
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
        // �ƶ�����һ���豸
        g_ota_manager.current_device_index++;
        if (g_ota_manager.current_device_index < g_ota_manager.need_ota_device_count)
        {
            ota_start_next_device();
        }
        else
        {
            // ��ǰ�ִ������豸���
            DEBUG_PRINTF("[OTA] Round %d completed: success=%d, failed=%d\r\n", g_ota_manager.retry_round, g_ota_manager.success_count, g_ota_manager.failed_count);

            // ����Ƿ���ʧ���豸��Ҫ����
            if (g_ota_manager.failed_device_count > 0 && g_ota_manager.retry_round < OTA_DEVICE_RETRY_MAX)
            {
                // ��ȡ��ǰ״̬
                ota_result_t current_status = ota_get_finish_status();

                // ����ǲ��ֳɹ��������̼������Լ���
                if (current_status == OTA_RESULT_PARTIAL_SUCCESS)
                {
                    g_ota_manager.firmware_retry_count++;
                    DEBUG_PRINTF("[OTA] Partial success, firmware_retry_count=%d/%d\r\n",
                                 g_ota_manager.firmware_retry_count, OTA_FIRMWARE_RETRY_MAX);

                    // ����Ƿ�ﵽ�̼�����������
                    if (g_ota_manager.firmware_retry_count >= OTA_FIRMWARE_RETRY_MAX)
                    {
                        DEBUG_PRINTF("[OTA] Firmware retry limit reached, force fail all remaining devices\r\n");

                        // ֹͣOTA����
                        g_ota_manager.ota_in_progress = false;
                        g_ota_manager.disable_broadcast = false;
                        g_ota_manager.disable_property_report = false;

                        // ������ȫʧ��״̬��WiFi
                        ota_send_finish_result_to_wifi(OTA_RESULT_FAILED);

                        // ��ִ�а汾���£���Ϊ���������ĳɹ���
                        return; // ֱ�ӷ��أ���������������
                    }
                }

                // δ�ﵽ���ޣ���������
                ota_retry_failed_devices();
            }
            else
            {
                // ȫ����ɻ�ﵽ����豸���Դ���
                DEBUG_PRINTF("\r\n[OTA] All upgrade attempts completed: total_success=%d, total_failed=%d\r\n", g_ota_manager.success_count, g_ota_manager.failed_count);
                g_ota_manager.ota_in_progress = false;
                g_ota_manager.disable_broadcast = false;
                g_ota_manager.disable_property_report = false;

                // �������״̬��WiFi
                ota_result_t finish_status = ota_get_finish_status();
                ota_send_finish_result_to_wifi(finish_status);

                // ������豸�����ɹ������°汾���ϱ�
                if (g_ota_manager.success_count > 0)
                {
                    if (g_ota_manager.fw_type == FW_TYPE_CT_SUB1G)
                    {
                        // CT Sub1G�������汾��ͨ��CMD_CT_SUB1G_VERSION����
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
 Description : 处理OTA超时。
---------------------------------------------------------------------------*/
static void ota_handle_timeout(void)
{
    static uint8_t timeout_print_counter = 30; // ���Ӿ�̬������

    if (g_ota_manager.current_device_index >= g_ota_manager.need_ota_device_count)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    // ����Ƿ�ʱ������״̬ʹ�ò�ͬ�ĳ�ʱʱ�䣩
    uint32_t timeout_threshold = OTA_TIMEOUT_MS;
    if (dev->state == OTA_STATE_QUERY_VERSION || dev->state == OTA_STATE_WAIT_CT_VERSION)
    {
        timeout_threshold = OTA_VERSION_QUERY_TIMEOUT_MS;
    }

    if (g_ota_manager.last_activity_ms < timeout_threshold)
    {
        return;
    }

    // ��ʱ����
    timeout_print_counter++;
    if (timeout_print_counter >= 2) // ÿ10�γ�ʱ��ӡһ��
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
                // �ط���ʼ������
                ota_send_sub1g_init_cmd(dev->sub1g_addr, g_ota_manager.sub1g_type, g_ota_manager.fw_length);
            }
            else
            {
                // �ط����ݰ�
                dev->state = OTA_STATE_TRANSMITTING;
            }

            g_ota_manager.last_activity_ms = 0;
        }
        break;

    case OTA_STATE_QUERY_VERSION:
        // ΢��汾ѯ�ʳ�ʱ����Ϊ����ʧ��
        DEBUG_PRINTF("[OTA] Inverter version query timeout, device upgrade failed\r\n");
        ota_complete_device(g_ota_manager.current_device_index, false);
        break;

    case OTA_STATE_WAIT_CT_VERSION:
        // CT Sub1G�汾�ϱ���ʱ����Ϊ����ʧ��
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
 Description : 完成设备OTA升级。
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

        // ���ӵ�ʧ���б�
        if (g_ota_manager.failed_device_count < INV_DEVICE_MAX_NUM)
        {
            g_ota_manager.failed_device_list[g_ota_manager.failed_device_count++] = device_idx;
            DEBUG_PRINTF("[OTA] Device[%d] FAILED (sub1g_addr=0x%06lX), added to retry list\r\n", device_idx, dev->sub1g_addr);
        }
    }
}

// ================= OTA��wifi֮��Ļ������� =================

/*---------------------------------------------------------------------------
 Name        : static void ota_request_firmware_from_wifi(uint32_t address, uint16_t length)
 Input       : address - 固件地址
               length - 固件长度
 Output      : 无
 Description : 从WiFi请求固件数据。
---------------------------------------------------------------------------*/
static void ota_request_firmware_from_wifi(uint32_t address, uint16_t length)
{
    DEBUG_PRINTF("[OTA] Request firmware from WiFi: addr=0x%06lX, len=%d\r\n", address, length);

    // ����serial_msg_t��Ϣ
    msg_output.type = 0; // 0-����
    msg_output.cmd = 0x1001;
    msg_output.code = 0;
    msg_output.cmd_data_length = 6;

    msg_output.cmd_data[0] = (address >> 24) & 0xFF;
    msg_output.cmd_data[1] = (address >> 16) & 0xFF;
    msg_output.cmd_data[2] = (address >> 8) & 0xFF;
    msg_output.cmd_data[3] = address & 0xFF;
    msg_output.cmd_data[4] = (length >> 8) & 0xFF;
    msg_output.cmd_data[5] = length & 0xFF;

    // ������Ϣ
    serial_msg_send(&msg_output);
}

/*---------------------------------------------------------------------------
 Name        : static void ota_send_finish_result_to_wifi(uint8_t result)
 Input       : result - 完成结果
 Output      : 无
 Description : 发送完成结果到WiFi。
---------------------------------------------------------------------------*/
static void ota_send_finish_result_to_wifi(ota_result_t result)
{
    DEBUG_PRINTF("[OTA] Send finish result to WiFi: result=%d\r\n", result);

    // ����serial_msg_t��Ϣ
    msg_output.type = 0;
    msg_output.cmd = 0x1002;
    msg_output.code = 0;
    msg_output.cmd_data_length = 1;
    msg_output.cmd_data[0] = result;

    // ������Ϣ
    serial_msg_send(&msg_output);
}

/*---------------------------------------------------------------------------
 Name        : void ota_copy_wifi_fw_data(...)
 Input       : 固件数据参数
 Output      : 无
 Description : 拷贝WiFi固件数据。
---------------------------------------------------------------------------*/
void ota_copy_wifi_fw_data(const uint8_t *data, uint16_t length)
{
    if (!g_ota_manager.ota_in_progress)
    {
        DEBUG_PRINTF("[OTA] Not in progress, ignore data\r\n");
        return;
    }

    // ��֤����
    if (length > OTA_FIRMWARE_BUFFER_SIZE)
    {
        DEBUG_PRINTF("[OTA] Invalid length: len=%d\r\n", length);
        return;
    }

    // ���浽������
    g_ota_manager.fw_buffer_valid_len = length;
    g_ota_manager.waiting_wifi_data = false;
    memcpy(g_ota_manager.fw_buffer, data, length);

    // ���õ�ǰ�豸��WiFi�������Լ�����
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
 Description : 固件类型转换为Sub1G类型。
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
 Description : 发送初始化请求。
---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 Name        : ota_result_t ota_get_finish_status(void)
 Input       : 无
 Output      : OTA完成状态
 Description : 获取OTA完成状态。
               - 全部成功 (result=0)：包括原始所有设备和新发现的设备都成功
               - 部分成功 (result=1)：包含部分设备成功，但不是全部
               - 全部失败 (result=2)：所有设备都失败
               注意：使用 original_device_count 和 need_ota_device_count
                     来判断在操作期间是否被更新
---------------------------------------------------------------------------*/
ota_result_t ota_get_finish_status(void)
{
    // ʹ��ԭʼ�豸���жϽ��������ʱneed_ota_device_count����£�
    if (g_ota_manager.success_count == g_ota_manager.original_device_count)
    {
        // ����ԭʼ��Ҫ�������豸���ɹ���
        return OTA_RESULT_SUCCESS_ALL;
    }
    else if (g_ota_manager.success_count > 0)
    {
        // �в����豸�ɹ���������ȫ��
        return OTA_RESULT_PARTIAL_SUCCESS;
    }
    else
    {
        // �����豸��ʧ����
        return OTA_RESULT_FAILED;
    }
}

// ================= OTA��sub1g֮��Ļ����ظ����� =================

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_init_ack(uint32_t sub1g_addr, uint8_t status)
 Input       : sub1g_addr - Sub1G地址
               status - 初始化状态
 Output      : 无
 Description : 处理Sub1G初始化应答。
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

    // ��ʼ���ɹ�����ʼ����
    dev->state = OTA_STATE_TRANSMITTING;
    dev->retry_count = 0;
    g_ota_manager.last_activity_ms = 0;
}

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_data_ack(uint32_t sub1g_addr, uint16_t packet_num, uint8_t status)
 Input       : sub1g_addr - Sub1G地址
               packet_num - 数据包编号
               status - 数据状态
 Output      : 无
 Description : 处理Sub1G数据应答。
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

    // packet_num��1��ʼ����Ҫת��Ϊ��0��ʼ������
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

    // �ɹ����ƶ�����һ��
    dev->retry_count = 0;
    dev->current_packet++;
    dev->state = OTA_STATE_TRANSMITTING;
    g_ota_manager.last_activity_ms = 0;
}

// ================= OTA��sub1g֮��Ļ������ͺ��� =================

/*---------------------------------------------------------------------------
 Name        : void ota_handle_sub1g_cancel(uint32_t sub1g_addr)
 Input       : sub1g_addr - Sub1G地址
 Output      : 无
 Description : 处理Sub1G取消请求。
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
 Input       : sub1g_addr - Sub1G地址
               sub1g_version - Sub1G版本号
               mcu_version - MCU版本号
 Output      : 无
 Description : 处理Sub1G版本报告。
---------------------------------------------------------------------------*/
void ota_handle_sub1g_version_report(uint32_t sub1g_addr, const char *sub1g_version, const char *mcu_version)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    // ����Ƿ��ǵ�ǰ�����������豸
    if (dev->sub1g_addr != sub1g_addr)
    {
        return;
    }

    // ����Ƿ���ѯ�ʰ汾״̬��ֻ����΢�棩
    if (dev->state != OTA_STATE_QUERY_VERSION)
    {
        return;
    }

    DEBUG_PRINTF("[OTA] Device[%d] version report: sub1g=%s, mcu=%s\r\n",
                 g_ota_manager.current_device_index,
                 sub1g_version ? sub1g_version : "NULL",
                 mcu_version ? mcu_version : "NULL");

    // �жϰ汾���Ƿ�һ��
    bool version_match = false;
    const char *reported_version = NULL;

    if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G)
    {
        // ΢��Sub1G�̼��������ȶ�Sub1G�汾
        reported_version = sub1g_version;
    }
    else if (g_ota_manager.fw_type == FW_TYPE_INV_800W || g_ota_manager.fw_type == FW_TYPE_INV_2500W)
    {
        // ΢��MCU�̼��������ȶ�MCU�汾
        reported_version = mcu_version;
    }
    else
    {
        // ��Ӧ�õ����FW_TYPE_CT_SUB1G��ʹ��0x77��
        DEBUG_PRINTF("[OTA] ERROR: Unexpected fw_type=%d in version report\r\n", g_ota_manager.fw_type);
        return;
    }

    if (reported_version && strcmp(reported_version, g_ota_manager.fw_version) == 0)
    {
        version_match = true;
        DEBUG_PRINTF("[OTA] Version match! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version, reported_version);

        // �汾ƥ�䣬����sys_param�е�΢��汾��Ϣ
        uint8_t dev_idx = dev->device_index;

        if (g_ota_manager.fw_type == FW_TYPE_INV_SUB1G && sub1g_version)
        {
            // ����΢��Sub1G�汾
            strncpy(sys_param.paired_inv_info[dev_idx].sub1g_version, sub1g_version, VERSION_STRING_MAX_LEN);
            sys_param.paired_inv_info[dev_idx].sub1g_version[VERSION_STRING_MAX_LEN] = '\0';
            DEBUG_PRINTF("[OTA] Update Inv[%d] Sub1G version: %s\r\n", dev_idx, sys_param.paired_inv_info[dev_idx].sub1g_version);
        }
        else if ((g_ota_manager.fw_type == FW_TYPE_INV_800W || g_ota_manager.fw_type == FW_TYPE_INV_2500W) && mcu_version)
        {
            // ����΢��MCU�汾
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

    // ���ݰ汾�ȶԽ���������
    ota_complete_device(g_ota_manager.current_device_index, version_match);
}

/*---------------------------------------------------------------------------
 Name        : void ota_handle_ct_sub1g_version_report(const char *ct_sub1g_version)
 Input       : ct_sub1g_version - CT Sub1G版本号
 Output      : 无
 Description : 处理CT Sub1G版本报告。
---------------------------------------------------------------------------*/
void ota_handle_ct_sub1g_version_report(const char *ct_sub1g_version)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    // ֻ��CT Sub1G����ʱ�Ŵ���
    if (g_ota_manager.fw_type != FW_TYPE_CT_SUB1G)
    {
        return;
    }

    ota_device_info_t *dev = &g_ota_manager.devices[g_ota_manager.current_device_index];

    // ����Ƿ��ڵȴ�CT�汾״̬
    if (dev->state != OTA_STATE_WAIT_CT_VERSION)
    {
        return;
    }

    DEBUG_PRINTF("[OTA] CT Sub1G version report: %s\r\n", ct_sub1g_version ? ct_sub1g_version : "NULL");

    // �жϰ汾���Ƿ�һ��
    bool version_match = false;
    if (ct_sub1g_version && strcmp(ct_sub1g_version, g_ota_manager.fw_version) == 0)
    {
        version_match = true;
        DEBUG_PRINTF("[OTA] CT Sub1G version match! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version, ct_sub1g_version);

        // �汾ƥ�䣬����sys_param�е�CT Sub1G�汾��Ϣ
        strncpy(sys_param.sub1g.sw_version, ct_sub1g_version, VERSION_STRING_MAX_LEN);
        sys_param.sub1g.sw_version[VERSION_STRING_MAX_LEN] = '\0';
        DEBUG_PRINTF("[OTA] Update CT Sub1G version: %s\r\n", sys_param.sub1g.sw_version);
    }
    else
    {
        DEBUG_PRINTF("[OTA] CT Sub1G version mismatch! target=%s, reported=%s\r\n",
                     g_ota_manager.fw_version, ct_sub1g_version ? ct_sub1g_version : "NULL");
    }

    // ���ݰ汾�ȶԽ���������
    ota_complete_device(g_ota_manager.current_device_index, version_match);
}

/*---------------------------------------------------------------------------
 Name        : void ota_force_cancel(void)
 Input       : 无
 Output      : 无
 Description : 强制取消OTA升级。
---------------------------------------------------------------------------*/
void ota_force_cancel(void)
{
    if (!g_ota_manager.ota_in_progress)
    {
        return;
    }

    DEBUG_PRINTF("[OTA] Force cancel all upgrades\r\n");

    // �������豸����ȡ������
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
 Input       : sub1g_addr - Sub1G地址
               type - OTA类型
               length - 固件长度
 Output      : 无
 Description : 发送Sub1G初始化命令。
---------------------------------------------------------------------------*/
void ota_send_sub1g_init_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type, uint32_t length)
{
    uint8_t buffer[16];
    uint16_t index = 0;

    // ֡ͷ
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G��ַ��3�ֽڣ�
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // ���ȣ�������1B + ����8B  = 9B��
    buffer[index++] = 9;

    // ������
    buffer[index++] = SUB1G_OTA_CMD_INIT;

    // ���ͣ�1�ֽڣ�
    buffer[index++] = type;

    // Sub1G��ַ��3�ֽڣ�
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // �̼����ȣ�4�ֽڣ�
    buffer[index++] = (length >> 24) & 0xFF;
    buffer[index++] = (length >> 16) & 0xFF;
    buffer[index++] = (length >> 8) & 0xFF;
    buffer[index++] = length & 0xFF;

    // ���͵�UART1����
    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] Send Sub1g init cmd: addr=0x%06lX, type=0x%02X, len=%lu\r\n", sub1g_addr, type, length);
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_data_packet(...)
 Input       : 数据包参数
 Output      : 无
 Description : 发送Sub1G数据包。
---------------------------------------------------------------------------*/
void ota_send_sub1g_data_packet(uint32_t sub1g_addr, sub1g_ota_type_t type, uint16_t total_packets, uint16_t packet_num, const uint8_t *data, uint8_t data_len)
{
    uint8_t buffer[UART1_TX_MSG_MAX_LEN - 1];
    uint16_t index = 0;

    // ֡ͷ
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G��ַ��3�ֽڣ�
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // CRC�������
    uint16_t checksum_start_pos = index;

    // ���ȣ�72��������1+����1+��Ƭ��2+�ڼ�Ƭ2+����64+У����2��
    buffer[index++] = 72;

    // ������
    buffer[index++] = SUB1G_OTA_CMD_DATA;

    // ���ͣ�1�ֽڣ�
    buffer[index++] = type;

    // ��Ƭ����2�ֽڣ�
    buffer[index++] = (total_packets >> 8) & 0xFF;
    buffer[index++] = total_packets & 0xFF;

    // �ڼ�Ƭ��2�ֽڣ�
    buffer[index++] = (packet_num >> 8) & 0xFF;
    buffer[index++] = packet_num & 0xFF;

    // ���ݣ��̶�64�ֽڣ��������0xFF��
    memcpy(&buffer[index], data, data_len);
    if (data_len < 64)
    {
        memset(&buffer[index + data_len], 0xFF, 64 - data_len);
    }
    index += 64;

    // CRC���ӳ����ֶο�ʼ����71�ֽ�
    uint16_t checksum = ota_calculate_checksum(&buffer[checksum_start_pos], 71);

    // CRC��2�ֽڣ�
    buffer[index++] = (checksum >> 8) & 0xFF;
    buffer[index++] = checksum & 0xFF;

    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] TX: pkt=%d/%d, len=%d, crc=0x%04X\r\n",
                 packet_num, total_packets, data_len, checksum);
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_cancel_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type)
 Input       : sub1g_addr - Sub1G地址
               type - OTA类型
 Output      : 无
 Description : 发送Sub1G取消命令。
---------------------------------------------------------------------------*/
void ota_send_sub1g_cancel_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type)
{
    uint8_t buffer[16];
    uint16_t index = 0;

    // ֡ͷ
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G��ַ��3�ֽڣ�
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // ���ȣ�������1B + ����1B + ��ַ3B = 5B��
    buffer[index++] = 5;

    // ������
    buffer[index++] = SUB1G_OTA_CMD_CANCEL_FROM_CT;

    // ���ͣ�1�ֽڣ�
    buffer[index++] = type;

    // Sub1G��ַ��3�ֽڣ��ظ���
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // ���͵�UART1����
    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] Send cancel cmd: addr=0x%06lX, type=0x%02X\r\n", sub1g_addr, type);
}

/*---------------------------------------------------------------------------
 Name        : void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr)
 Input       : sub1g_addr - Sub1G地址
 Output      : 无
 Description : 发送Sub1G版本查询命令。
---------------------------------------------------------------------------*/
void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr)
{
    uint8_t buffer[16];
    uint16_t index = 0;

    // ֡ͷ
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;

    // Sub1G��ַ��3�ֽڣ�
    buffer[index++] = (sub1g_addr >> 16) & 0xFF;
    buffer[index++] = (sub1g_addr >> 8) & 0xFF;
    buffer[index++] = sub1g_addr & 0xFF;

    // ���ȣ�������1B��
    buffer[index++] = 1;

    // ������
    buffer[index++] = SUB1G_OTA_CMD_QUERY_VERSION;

    // ���͵�UART1����
    uart1_tx_queue_push(buffer, index);

    DEBUG_PRINTF("[OTA] Send query version cmd: addr=0x%06lX\r\n", sub1g_addr);
}

/*---------------------------------------------------------------------------
 Name        : uint16_t calculate_crc16(...)
 Input       : 数据参数
 Output      : CRC16校验值
 Description : 计算CRC16校验和。
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
