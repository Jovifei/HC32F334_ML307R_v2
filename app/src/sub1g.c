//
// Included Files
//
#include <stdlib.h>
#include "board.h"
#include "sub1g.h"
#include "main.h"
#include "fft.h"
#include "wifi.h"
#include "string.h"
#include "eeprom.h"
#include "stdio.h"
#include "stdbool.h"
#include "ota_max.h"
#include "debug.h"

// ================= 全局变量定义 =================
sub1g_uart_com_rx_status_t sub1g_status;
sub1g_frame_t sub1g_rx_frame;

// ================= UART1发送队列 =================
//  UART1发送队列实例
static uart1_tx_queue_t g_uart1_tx_queue;

// 临时接收缓冲区（中断中使用）
uint8_t uart1_temp_buffer[SUB1G_MAX_FRAME_SIZE];
uint16_t uart1_temp_count = 0;
uint16_t uart1_expected_length = 0;

// 已绑定设备列表缓冲区、待配对设备列表缓冲区、用户配对列表缓冲区
static char paired_device_list_buffer[DEVICE_LIST_BUFFER_SIZE];
static char unpaired_device_list_buffer[DEVICE_LIST_BUFFER_SIZE];
static char user_pair_list_buffer[DEVICE_LIST_BUFFER_SIZE];

static void sub1g_send_record_channel(void);
void sub1g_reboot(void)
{
    GPIO_ResetPins(GPIO_PORT_A, GPIO_PIN_15);
    delay_us(1000);
    GPIO_SetPins(GPIO_PORT_A, GPIO_PIN_15);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_init(void)
 Input       : 无
 Output      : 无
 Description : 初始化SUB1G模块和队列
---------------------------------------------------------------------------*/
void sub1g_init(void)
{
    sub1g_reboot();

    memset(&sub1g_status, 0, sizeof(sub1g_uart_com_rx_status_t));
    memset(&sub1g_rx_frame, 0, sizeof(sub1g_frame_t));
    memset(uart1_temp_buffer, 0, sizeof(uart1_temp_buffer));
    memset(&g_uart1_tx_queue, 0, sizeof(uart1_tx_queue_t));
    uart1_temp_count = 0;
    uart1_expected_length = 0;
}

/*---------------------------------------------------------------------------
 Name        : void USART1_Handler(void)
 Input       : ??
 Output      : 无
 Description : USART1接收中断处理函数，按字节解析帧头，完整帧复制到接收缓冲区
               帧格式：帧头(0xFACE) + sub1g地址(3B) + 长度(1B) + 命令码(1B) + 数据(nB)
               接收完整帧后设置frame_received标志，由主循环任务处理
---------------------------------------------------------------------------*/
void USART1_Handler(void)
{
    uint8_t received_byte;

    if (USART_GetStatus(CM_USART1, USART_FLAG_RX_FULL))
    {
        received_byte = USART_ReadData(CM_USART1);

        switch (uart1_temp_count)
        {
        case 0:
            // 等待帧头第一字节 0xFA
            if (received_byte == 0xFA)
            {
                uart1_temp_buffer[uart1_temp_count++] = received_byte;
            }
            break;

        case 1:
            // 等待帧头第二字节 0xCE
            if (received_byte == 0xCE)
            {
                // 帧头完整 FA CE
                uart1_temp_buffer[uart1_temp_count++] = received_byte;
            }
            else if (received_byte == 0xFA)
            {
                // 收到0xFA，可能是新帧头开始，重置并保留
                uart1_temp_buffer[0] = 0xFA;
                uart1_temp_count = 1;
                uart1_expected_length = 0;
            }
            else
            {
                // 非法字节，重置接收状态
                uart1_temp_count = 0;
                uart1_expected_length = 0;
            }
            break;

        case 2:
        case 3:
        case 4:
            // 接收SUB1G源地址共3个字节
            uart1_temp_buffer[uart1_temp_count++] = received_byte;
            break;

        case 5:
            // 接收长度字段
            uart1_temp_buffer[uart1_temp_count++] = received_byte;

            // 计算完整帧长度 = 帧头(2) + 地址(3) + 长度(1) + 数据(length字节)
            uart1_expected_length = 6 + received_byte;

            break;

        default:
            // 继续接收数据字节直到帧完整
            uart1_temp_buffer[uart1_temp_count++] = received_byte;

            // 判断是否接收完整帧
            if (uart1_temp_count == uart1_expected_length)
            {
                // 验证帧头合法后处理数据
                if (uart1_temp_buffer[0] == 0xFA && uart1_temp_buffer[1] == 0xCE)
                {

                    if (g_ota_manager.ota_in_progress && uart1_temp_buffer[6] == 0x01)
                    {
                        uart1_temp_count = 0;
                        uart1_expected_length = 0;
                    }
                    else
                    {
                        // 复制完整帧到接收缓冲区并置标志
                        memcpy(sub1g_status.rx_buffer, uart1_temp_buffer, uart1_temp_count);
                        sub1g_status.rx_index = uart1_temp_count;
                        sub1g_status.frame_received = true;
                    }
                }

                // 接收完毕后无论校验成功与否均重置状态
                uart1_temp_count = 0;
                uart1_expected_length = 0;
            }
            break;
        }

        if (uart1_temp_count >= SUB1G_MAX_FRAME_SIZE)
        {
            uart1_temp_count = 0;
            uart1_expected_length = 0;
        }
    }

    USART_ClearStatus(CM_USART1, USART_FLAG_ALL);
    __DSB(); /* Arm Errata 838869 */
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_rx_task(void)
 Input       : ??
 Output      : 无
 Description : 主循环调用，检测帧接收完成标志并处理
---------------------------------------------------------------------------*/
void sub1g_rx_task(void)
{
    if (sub1g_status.frame_received)
    {
        // if (sub1g_status.rx_buffer[6] == 0x50)
        // {
        //     printf("[Sub1G] Uart_Rx[%dB]: ", sub1g_status.rx_index);
        //     for (uint8_t i = 0; i < sub1g_status.rx_index; i++)
        //     {
        //         printf("%02X ", sub1g_status.rx_buffer[i]);
        //     }
        //     printf("\r\n");
        // }

        // 处理接收到的帧
        sub1g_process_received_frame();

        sub1g_status.frame_received = false;
        sub1g_status.rx_index = 0;

        // 更新SUB1G通信状态
        if (sub1g_status.rx_buffer[6] != 0x41 && sub1g_status.rx_buffer[6] != 0x42 && sub1g_status.rx_buffer[6] != 0x43)
        {
            sys_param.sub1g.state = 4; // 4 = 已配对（通信正常）
            sys_param.sub1g.timeout_count = 0;
        }

        sys_param.sub1g.reboot_count = 0;
    }
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_process_received_frame(void)
 Input       : ??
 Output      : ??
 Description : 解析接收到的完整帧，根据命令码分发处理
               更新设备在线状态，触发属性上报等后续动作
---------------------------------------------------------------------------*/
void sub1g_process_received_frame(void)
{
    uint8_t *data = sub1g_status.rx_buffer;
    uint8_t data_content_length;
    uint8_t inv_index;
    int8_t unpaired_index;

    // 解析帧头各字段
    sub1g_rx_frame.frame_header = (data[0] << 8) | data[1];
    sub1g_rx_frame.sub1g_addr = ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | data[4];
    sub1g_rx_frame.data_length = data[5];
    sub1g_rx_frame.command_code = data[6];

    // if (sub1g_rx_frame.sub1g_addr != 0x111111)
    // {
    //     DEBUG_PRINTF("[SUB1G] Received frame: sub1g_addr: %06X: command_code: 0x%02X\r\n", sub1g_rx_frame.sub1g_addr, sub1g_rx_frame.command_code);
    // }

    // 校验帧头
    if (sub1g_rx_frame.frame_header != SUB1G_FRAME_HEADER)
    {
        return;
    }

    // 数据内容长度 = 数据长度字段 - 命令码(1B)
    data_content_length = sub1g_rx_frame.data_length - 1;

    // 将数据内容复制到帧结构体
    if (data_content_length > 0 && data_content_length <= SUB1G_MAX_DATA_SIZE)
    {
        memcpy(sub1g_rx_frame.data_content, &data[7], data_content_length);
    }

    // 根据命令码分发处理
    switch (sub1g_rx_frame.command_code)
    {
    case CMD_INV_PAIR_BROADCAST: // 0x01 - 微逆配对广播
    {
        // 解析data_content：产品型号(1B) + 设备SN(15B)，型号1:800W微逆
        uint8_t product_model = sub1g_rx_frame.data_content[0];

        char device_sn[SN_LENGTH + 1] = {0}; // 末尾初始化为0
        memcpy(device_sn, &sub1g_rx_frame.data_content[1], SN_LENGTH);

        // 过滤SN中非字母数字字符，截断到有效字符
        for (uint8_t i = 0; i < SN_LENGTH; i++)
        {
            char c = device_sn[i];
            if (!((c >= '0' && c <= '9') ||
                  (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z')))
            {
                device_sn[i] = '\0'; // 截断，后续字节无效
                break;
            }
        }
        device_sn[SN_LENGTH] = '\0'; // 确保结尾

        // printf("Received pairing broadcast from device: %s, model: %d\r\n", device_sn, product_model);

        // 检查是否为预分配设备（EEPROM中有此SN但sub1g_addr为0的记录）
        int8_t pre_alloc_index = eeprom_find_inv_index_by_sn(device_sn);
        if (pre_alloc_index >= 0 && sys_param.paired_inv_info[pre_alloc_index].sub1g_addr == 0)
        {
            // ===== 情况0: 预分配设备补全sub1g_addr =====
            // printf("Found pre-allocated device: SN=%s, updating sub1g_addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr);

            // 更新EEPROM记录中的sub1g_addr和型号
            if (eeprom_update_device_sub1g_addr(device_sn, sub1g_rx_frame.sub1g_addr, product_model) == 0)
            {
                // 加入INV请求配对列表并设置2秒确认窗口
                inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);
                if (unpaired_index >= 0)
                {
                    sys_param.inv_request_pair_list[unpaired_index].paired_unvalid_ms = 2000;
                }

                // 立即发送绑定命令
                sub1g_send_bind(sub1g_rx_frame.sub1g_addr);

                // printf("Pre-allocated device bound: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr);
            }
            break;
        }

        // 检查是否已绑定（已在paired_inv_info中）
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            // printf("Device already paired: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr);
            // ===== 情况1: 已绑定设备重新握手 =====
            unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);

            // 检查是否在2秒确认窗口内
            bool in_confirm_window = (unpaired_index >= 0 && sys_param.inv_request_pair_list[unpaired_index].paired_unvalid_ms > 0);

            if (in_confirm_window)
            {
                // 在确认窗口内，重发绑定命令
                sub1g_send_bind(sub1g_rx_frame.sub1g_addr);
            }
            else
            {
                // 不在窗口，检查是否在用户配对列表
                int8_t user_pair_index = user_pair_list_find_by_sn(device_sn);

                if (user_pair_index >= 0)
                {
                    // 在用户列表中，加入请求配对列表并触发重绑
                    inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                    // 设置2秒确认窗口
                    unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);
                    if (unpaired_index >= 0)
                    {
                        sys_param.inv_request_pair_list[unpaired_index].paired_unvalid_ms = 2000;
                    }
                    sub1g_send_bind(sub1g_rx_frame.sub1g_addr);
                    // printf("Device re-bind: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                }
                else
                {
                    // 不在用户列表，更新或添加INV请求配对列表
                    if (unpaired_index >= 0)
                    {
                        // 已在列表中，刷新上报时间
                        sys_param.inv_request_pair_list[unpaired_index].unpaired_updata_ms = 0;
                    }
                    else
                    {
                        // 不在列表，新增到请求配对列表
                        inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                    }
                }
            }
        }
        else
        {
            // ===== 情况2: 未绑定设备申请配对 =====
            // 检查是否在用户配对列表
            int8_t user_pair_index = user_pair_list_find_by_sn(device_sn);

            if (user_pair_index >= 0)
            {
                // 在用户列表中，加入请求配对列表并自动绑定
                inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                if (user_bind_device(sub1g_rx_frame.sub1g_addr))
                {
                    // // 打印日志
                    // printf("Device auto-paired: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                }
            }
            else
            {
                // 不在用户列表，更新或新增INV请求配对列表
                unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);

                if (unpaired_index >= 0)
                {
                    // printf("Device in unpaired list: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                    // 已在列表中，刷新上报时间
                    sys_param.inv_request_pair_list[unpaired_index].unpaired_updata_ms = 0;
                }
                else
                {
                    // printf("Add New device in unpaired list: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                    // 添加新设备到INV请求配对列表
                    inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                }
            }
        }
        break;
    }

    case CMD_INV_REPORT_NORMAL: // 0x50 - 微逆日常上报(3s周期)
    {
        // 根据sub1g_addr查找已绑定设备索引（仅查找已绑定的SUB1G地址）
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);
        // printf("Receive device:%06X id=%d updata \r\n", sub1g_rx_frame.sub1g_addr, inv_index);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            // 解析data_content数据并更新sys_param.paired_inv_info[]
            uint8_t *data = sub1g_rx_frame.data_content;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // 记录上次故障和防逆流状态用于变更检测
            uint32_t fault_param_last = inv->fault_param;
            uint8_t antiflow_enable_last = inv->antiflow_enable;

            // 首次收到数据时初始化统计
            if (!inv->stats_started)
            {
                inv->stats_started = true;
                inv->stats_time_sec = 0;
                inv->rx_0x50_count = 0;
                inv->rx_0x52_count = 0;
                inv->rx_0x54_count = 0;
                inv->rx_0x55_count = 0;
                inv->rx_0x56_count = 0;
                inv->rx_0x57_count = 0;
                inv->plr = 0;
            }

            inv->rx_0x50_count++;
            // 解析数据字段

            // Byte 0-1: 并网功率(W)
            inv->grid_power = ((int16_t)((data[0] << 8) | data[1]));

            // Byte 2: 工作状态
            inv->work_state = data[2];

            // Byte 3-6: 故障参数
            inv->fault_param = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6];

            // Byte 7: 防逆流开关状态
            inv->antiflow_enable = data[7];

            // Byte 8: PV路数
            uint8_t pv_num = data[8];
            inv->pv_num = pv_num;

            // 读取PV各路功率 (2B*n)
            uint16_t offset = 9;
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].power = ((int16_t)((data[offset] << 8) | data[offset + 1])) / 10.0f;
                offset += 2;
            }

            // DEBUG_PRINTF("0X50: inv_index = %d, grid_power = %.2f, work_state = %d, fault_param = %d, antiflow_enable = %d, pv_num = %d, pv0_power = %.2f, pv1_power = %.2f.\n", inv_index, inv->grid_power, inv->work_state, inv->fault_param, inv->antiflow_enable, inv->pv_num, inv->pv[0].power, inv->pv[1].power);

            // 检测属性变更
            if (fault_param_last != inv->fault_param || antiflow_enable_last != inv->antiflow_enable)
            {
                inv->prop_changed = true;
            }

            // 更新sub1g地址
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            // 在线状态变化时标记属性变更
            if (inv->online_state != 2)
            {
                DEBUG_PRINTF("inv_index = %d, online_state = %d, prop_changed = %d.\n", inv_index, inv->online_state, inv->prop_changed);
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;

            // 根据工作模式同步防逆流开关
            switch (sys_param.power_work_mode)
            {
            case 1:                                 // 防逆流模式（默认）
                sys_param.anti_backflow_switch = 1; // 开启防逆流
                break;

            case 2:                                 // 强制发电模式
                sys_param.anti_backflow_switch = 1; // 开启防逆流
                break;

            case 3:                                 // 自由发电模式
                sys_param.anti_backflow_switch = 0; // 关闭防逆流
                break;

            default:
                sys_param.anti_backflow_switch = 1;
                break;
            }
            if (sys_param.anti_backflow_switch != inv->antiflow_enable)
            {
                inv->antiflow_enable = sys_param.anti_backflow_switch;
                sub1g_send_set_antiflow(inv->sub1g_addr, inv->antiflow_enable);
                // DEBUG_PRINTF("sub1g_send_set_antiflow: mode = %d, addr=0x%06X, antiflow_enable = %d\n", sys_param.power_work_mode, inv->sub1g_addr, inv->antiflow_enable);
            }
        }
        else
        {
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_INV_REPORT_NORMAL_2: // 0x52 - 微逆日常上报2(10s周期)
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // 初始化统计
            if (!inv->stats_started)
            {
                inv->stats_started = true;
                inv->stats_time_sec = 0;
                inv->rx_0x50_count = 0;
                inv->rx_0x52_count = 0;
                inv->rx_0x54_count = 0;
                inv->rx_0x55_count = 0;
                inv->rx_0x56_count = 0;
                inv->rx_0x57_count = 0;
                inv->plr = 0;
            }

            // 0x52接收计数
            inv->rx_0x52_count++;
            //  解析数据字段

            // Byte 0: 今日发电时间(0.1h单位) - 除以10得小时
            inv->today_power_time = data[0] / 10.0f;

            // Byte 1-4: 今日发电量(float, kWh)
            sub1g_bytes_to_float(&data[1], &inv->today_energy);

            // Byte 5-8: 累计发电量(float, kWh)
            sub1g_bytes_to_float(&data[5], &inv->lifetime_energy);

            // Byte 9-10: 环境温度(×10整数编码转float)
            inv->ambient_temperature = ((int16_t)((data[9] << 8) | data[10])) / 10.0f;

            // Byte 11-12: 电网频率(×10整数编码转float)
            inv->grid_frequency = ((int16_t)((data[11] << 8) | data[12])) / 10.0f;

            // Byte 13-14: 电网电压(×10整数编码转float)
            inv->grid_voltage = ((int16_t)((data[13] << 8) | data[14])) / 10.0f;

            // printf("0x52: inv_index = %d, today_power_time = %f, today_energy = %f, lifetime_energy = %f, temperature = %f, fre = %f, vol = %f.\n",
            //        inv_index,
            //        sys_param.paired_inv_info[inv_index].today_power_time,
            //        sys_param.paired_inv_info[inv_index].today_energy,
            //        sys_param.paired_inv_info[inv_index].lifetime_energy,
            //        sys_param.paired_inv_info[inv_index].ambient_temperature,
            //        sys_param.paired_inv_info[inv_index].grid_frequency,
            //        sys_param.paired_inv_info[inv_index].grid_voltage);

            // 更新sub1g地址和在线状态
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            if (inv->online_state != 2)
            {
                DEBUG_PRINTF("inv_index = %d, online_state = %d, prop_changed = %d.\n", inv_index, inv->online_state, inv->prop_changed);
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;
        }
        else
        {
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_INV_REPORT_ALL: // 0x51 - 微逆上报全部数据
    {
        if (sys_param.inv_0x51_report.valid != 1)
        {
            // 根据sub1g_addr查找已绑定设备索引
            inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

            if (inv_index < INV_DEVICE_MAX_NUM)
            {
                // 记录0x51上报内容
                sys_param.inv_0x51_report.inv_index = inv_index;
                sys_param.inv_0x51_report.data_len = sub1g_rx_frame.data_length - 1; // 总长度-命令码长度

                // 长度超出缓冲区时截断
                if (sys_param.inv_0x51_report.data_len > INV_REPORT_51_SIZE)
                {
                    sys_param.inv_0x51_report.data_len = INV_REPORT_51_SIZE;
                }

                // 拷贝数据并添加结尾符
                memcpy(sys_param.inv_0x51_report.data, sub1g_rx_frame.data_content, sys_param.inv_0x51_report.data_len);
                sys_param.inv_0x51_report.data[sys_param.inv_0x51_report.data_len] = '\0';
                sys_param.inv_0x51_report.valid = 1;

                // 更新在线状态
                sys_param.paired_inv_info[inv_index].sub1g_addr = sub1g_rx_frame.sub1g_addr;
                sys_param.paired_inv_info[inv_index].online_state = 2;
                sys_param.paired_inv_info[inv_index].offline_updata_ms = 0;

                DEBUG_PRINTF("Received 0x51 from inv[%d], addr=0x%06X, len=%d\r\n", inv_index, sub1g_rx_frame.sub1g_addr, sys_param.inv_0x51_report.data_len);
            }
            else
            {
                // 未找到绑定设备，发送解绑
                sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
            }
        }
        break;
    }

    case CMD_INV_REPORT_NORMAL_3: // 0x54 - 微逆日常上报3(7s周期)
    {
        // 重新记录信道
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // 相序字段
            if (!inv->stats_started)
            {
                inv->stats_started = true;
                inv->stats_time_sec = 0;
                inv->rx_0x50_count = 0;
                inv->rx_0x52_count = 0;
                inv->rx_0x54_count = 0;
                inv->rx_0x55_count = 0;
                inv->rx_0x56_count = 0;
                inv->rx_0x57_count = 0;
                inv->plr = 0;
            }
            inv->rx_0x54_count++;

            // 记录上次发电开关状态用于变更检测
            uint8_t power_enable_last = inv->power_enable;

            // Byte 0: 发电开关状态
            inv->power_enable = data[0];

            // Byte 1: 微逆RSSI
            inv->inv_rssi = (int8_t)data[1];

            // Byte 2: PV路数
            uint8_t pv_num = data[2];
            inv->pv_num = pv_num;

            // 读取PV各路状态 (1B*n)
            uint16_t offset = 3;
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].state = data[offset + i];
            }
            offset += pv_num;

            // 读取PV各路电压 (2B*n)
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].voltage = ((int16_t)((data[offset] << 8) | data[offset + 1])) / 10.0f;
                offset += 2;
            }

            // 读取PV各路电流 (2B*n)
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].current = ((int16_t)((data[offset] << 8) | data[offset + 1])) / 10.0f;
                offset += 2;
            }

            // 信道索引变更检测
            if (power_enable_last != inv->power_enable)
            {
                inv->prop_changed = true;
            }

            // 更新sub1g地址和在线状态
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            if (inv->online_state != 2)
            {
                DEBUG_PRINTF("inv_index = %d, online_state = %d, prop_changed = %d.\n", inv_index, inv->online_state, inv->prop_changed);
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;
        }
        else
        {
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_INV_REPORT_SET_1: // 0x55 - 微逆设置上报1: 接入点 + 功率限制 + sub1g版本
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            uint8_t str_packet_len = 0;
            uint8_t safe_copy_len = 0;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // 初始化统计
            if (!inv->stats_started)
            {
                inv->stats_started = true;
                inv->stats_time_sec = 0;
                inv->rx_0x50_count = 0;
                inv->rx_0x52_count = 0;
                inv->rx_0x54_count = 0;
                inv->rx_0x55_count = 0;
                inv->rx_0x56_count = 0;
                inv->rx_0x57_count = 0;
                inv->plr = 0;
            }
            inv->rx_0x55_count++;

            // 记录旧值用于变更检测
            uint8_t old_connection_point = inv->connection_point;
            uint16_t old_power_limit = inv->power_limit;
            char old_sub1g_version[VERSION_STRING_MAX_LEN + 1];
            strncpy(old_sub1g_version, inv->sub1g_version, VERSION_STRING_MAX_LEN);
            old_sub1g_version[VERSION_STRING_MAX_LEN] = '\0';

            // Byte 0: 接入点
            inv->connection_point = data[index++];

            // Byte 1-2: 功率限制(W)
            inv->power_limit = ((uint16_t)data[index] << 8) | data[index + 1];
            index += 2;

            // sub1g版本 (长度前缀 + 字符串)
            str_packet_len = data[index++];
            safe_copy_len = (str_packet_len > VERSION_STRING_MAX_LEN) ? VERSION_STRING_MAX_LEN : str_packet_len;
            memcpy(inv->sub1g_version, &data[index], safe_copy_len);
            inv->sub1g_version[safe_copy_len] = '\0';
            index += str_packet_len;

            // DEBUG_PRINTF("0X55: inv_index = %d, connection_point = %d, power_limit = %d, sub1g_version = %s.\n", inv_index, inv->connection_point, inv->power_limit, inv->sub1g_version);

            // 检测变更
            if (old_connection_point != inv->connection_point ||
                old_power_limit != inv->power_limit ||
                strncmp(old_sub1g_version, inv->sub1g_version, VERSION_STRING_MAX_LEN) != 0)
            {
                inv->settings_changed = true;

                if (strncmp(old_sub1g_version, inv->sub1g_version, VERSION_STRING_MAX_LEN) != 0)
                {
                    if (old_sub1g_version[0] != '\0')
                    {
                        DEBUG_PRINTF("  Sub1G: %s -> %s\r\n", old_sub1g_version, inv->sub1g_version);
                    }
                    else
                    {
                        DEBUG_PRINTF("  Sub1G: First collected -> %s\r\n", inv->sub1g_version);
                    }
                    sys_param.slave_version.slave_version_reported = false;
                    DEBUG_PRINTF("[Version] Trigger slave version re-report due to sub1g version change\r\n");
                }
            }

            // 更新sub1g地址和在线状态，在线状态变化时标记属性变更
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            if (inv->online_state != 2)
            {
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;
        }
        else
        {
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_INV_REPORT_SET_2: // 0x56 - 微逆设置上报2: MCU版本 + 相序
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            uint8_t str_packet_len = 0;
            uint8_t safe_copy_len = 0;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // 命令码
            if (!inv->stats_started)
            {
                inv->stats_started = true;
                inv->stats_time_sec = 0;
                inv->rx_0x50_count = 0;
                inv->rx_0x52_count = 0;
                inv->rx_0x54_count = 0;
                inv->rx_0x55_count = 0;
                inv->rx_0x56_count = 0;
                inv->rx_0x57_count = 0;
                inv->plr = 0;
            }

            inv->rx_0x56_count++;

            // 检测属性变更
            char old_sw_version[VERSION_STRING_MAX_LEN + 1];
            strncpy(old_sw_version, inv->sw_version, VERSION_STRING_MAX_LEN);
            old_sw_version[VERSION_STRING_MAX_LEN] = '\0';

            // MCU版本 (长度前缀 + 字符串)
            str_packet_len = data[index++];
            safe_copy_len = (str_packet_len > VERSION_STRING_MAX_LEN) ? VERSION_STRING_MAX_LEN : str_packet_len;
            memcpy(inv->sw_version, &data[index], safe_copy_len);
            inv->sw_version[safe_copy_len] = '\0';
            index += str_packet_len;

            // 帧头
            uint8_t inv_phase = data[index++];
            uint8_t local_phase = inv->phase;

            // DEBUG_PRINTF("0X56: inv_index = %d, sw_version = %s, phase = %d.\n", inv_index, inv->sw_version, inv_phase);

            // #ifdef FFT_DEBUG_PRINT
            //             printf("Device: 0x%06X, Report phase=%d, local phase=%d\r\n",
            //                    sub1g_rx_frame.sub1g_addr, inv_phase, local_phase);
            // #endif

            // 本地相序有效时进行校验
            if (local_phase > 0 && local_phase <= 3)
            {
                if (inv_phase != local_phase)
                {
                    // 相序不一致，下发正确相序
                    sub1g_send_set_inv_phase(sub1g_rx_frame.sub1g_addr, local_phase);

                    DEBUG_PRINTF("Inv[%d] phase mismatch! Correct: %d -> %d\r\n", inv_index, inv_phase, local_phase);
                }
            }
            else if ((inv_phase == 0 || local_phase != inv_phase) && !sys_param.fft_identify.is_ffting)
            {
                // 触发FFT识别
                sys_param.fft_identify.resend_cmd = true;
                sys_param.fft_identify.sub1g_addr = sub1g_rx_frame.sub1g_addr;
#ifdef FFT_DEBUG_PRINT
                printf("0x%06X need FFT identify! \r\n", sub1g_rx_frame.sub1g_addr);
#endif
            }
            else if (sys_param.fft_identify.is_ffting)
            {
#ifdef FFT_DEBUG_PRINT
                printf("[FFT Waiting]: 0x%06X\r\n", sub1g_rx_frame.sub1g_addr);
#endif
            }

            // 版本变更检测
            if (strncmp(old_sw_version, inv->sw_version, VERSION_STRING_MAX_LEN) != 0)
            {
                inv->settings_changed = true;

                if (old_sw_version[0] != '\0')
                {
                    DEBUG_PRINTF("  SW: %s -> %s\r\n", old_sw_version, inv->sw_version);
                }
                else
                {
                    DEBUG_PRINTF("  SW: First collected -> %s\r\n", inv->sw_version);
                }
                sys_param.slave_version.slave_version_reported = false;
                DEBUG_PRINTF("[Version] Trigger slave version re-report due to sw version change\r\n");
            }

            // 首次收到数据时初始化统计
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            // 在线状态变化时标记属性变更
            if (inv->online_state != 2)
            {
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;
        }
        else
        {
            // 未找到绑定设备，发送解绑
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_INV_REPORT_SET_3: // 0x57 - 微逆设置上报3: 型号 + 信道索引
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            uint8_t str_packet_len = 0;
            uint8_t safe_copy_len = 0;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // 命令码
            if (!inv->stats_started)
            {
                inv->stats_started = true;
                inv->stats_time_sec = 0;
                inv->rx_0x50_count = 0;
                inv->rx_0x52_count = 0;
                inv->rx_0x54_count = 0;
                inv->rx_0x55_count = 0;
                inv->rx_0x56_count = 0;
                inv->rx_0x57_count = 0;
                inv->plr = 0;
            }

            inv->rx_0x57_count++;

            // 清空内存槽位，检测变更
            char old_product_model[PRODUCT_MODEL_MAX_LEN + 1];
            strncpy(old_product_model, inv->product_model, PRODUCT_MODEL_MAX_LEN);
            old_product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

            // 型号字符串 (长度前缀 + 字符串)
            str_packet_len = data[index++];
            safe_copy_len = (str_packet_len > PRODUCT_MODEL_MAX_LEN) ? PRODUCT_MODEL_MAX_LEN : str_packet_len;
            memcpy(inv->product_model, &data[index], safe_copy_len);
            inv->product_model[safe_copy_len] = '\0';
            index += str_packet_len;

            // 信道索引变更检测并触发属性上报
            if (inv->channel_index != data[index])
            {
                inv->settings_changed = true;
            }

            // 重新记录信道
            inv->channel_index = data[index++];
            // DEBUG_PRINTF("0X57: inv_index = %d, product_model = %s, channel_index = %d.\n", inv_index, inv->product_model, inv->channel_index);

            // 型号变更检测
            if (strncmp(old_product_model, inv->product_model, PRODUCT_MODEL_MAX_LEN) != 0)
            {
                inv->settings_changed = true;
                DEBUG_PRINTF("[INV] [0x%06X] updata: Sub1g=%s, SW=%s, Model=%s\r\n",
                             inv->sub1g_addr,
                             inv->sub1g_version,
                             inv->sw_version,
                             inv->product_model);
            }

            // 更新sub1g地址和在线状态
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            if (inv->online_state != 2)
            {
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;
        }
        else
        {
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_CT_SUB1G_VERSION: // 收到CT模块的sub1g版本回复
    {
        uint8_t *data = sub1g_rx_frame.data_content;

        // 解析3字节的sub1g地址
        sys_param.sub1g.ct_sub1g_addr = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

        // 保存旧版本用于比较
        char old_version[VERSION_STRING_MAX_LEN + 1];
        strncpy(old_version, sys_param.sub1g.sw_version, VERSION_STRING_MAX_LEN);
        old_version[VERSION_STRING_MAX_LEN] = '\0';

        // 解析版本字符串
        uint8_t str_version_len = data_content_length - 3; // 去掉前3字节地址
        if (str_version_len > VERSION_STRING_MAX_LEN)
        {
            str_version_len = VERSION_STRING_MAX_LEN;
        }
        memcpy(sys_param.sub1g.sw_version, &data[3], str_version_len);
        sys_param.sub1g.sw_version[str_version_len] = '\0';

        DEBUG_PRINTF("[SUB1G] CT Sub1G: Src_addr=0x%06lX, Version=%s\r\n", sys_param.sub1g.ct_sub1g_addr, sys_param.sub1g.sw_version);

        // 版本发生变化时触发版本重新上报
        if (strcmp(old_version, sys_param.sub1g.sw_version) != 0 && old_version[0] != '\0')
        {
            DEBUG_PRINTF("[SUB1G] CT Sub1G changed: %s -> %s\r\n", old_version, sys_param.sub1g.sw_version);
            sys_param.slave_version.slave_version_reported = false;
     
        }

        // 通知OTA模块CT Sub1G版本上报
        ota_handle_ct_sub1g_version_report(sys_param.sub1g.sw_version);

        break;
    }

    case CMD_CT_SUB1G_RSSI: // ???CT??sub1g RSSI???
    {
        uint8_t *data = sub1g_rx_frame.data_content;

        // RSSI
        sys_param.sub1g.rssi = (int8_t)data[0];

        // CT当前信道索引
        if (data_content_length >= 2)
        {
            sys_param.sub1g.channel_index = data[1];
        }
        // DEBUG_PRINTF("[SubG]CT report RSSI=%d, Channel=%d\r\n", sys_param.sub1g.rssi, sys_param.sub1g.channel_index);
        break;
    }

    case CMD_CT_SUB1G_RESTART: // SUB1G????(OTA???)
    {
        // 清除版本字符串
        sys_param.sub1g.sw_version[0] = '\0';
        sys_param.sub1g.version_timer_ms = 2000;

        // 拷贝消息
        sub1g_send_record_channel();

        // 版本已变化，触发版本重新上报
        sys_param.slave_version.slave_version_reported = false;
      
        break;
    }

    case SUB1G_OTA_CMD_INIT_ACK: // OTA????????
    {
        if (sub1g_rx_frame.data_length >= 1)
        {
            uint8_t status = sub1g_rx_frame.data_content[0];
            ota_handle_sub1g_init_ack(sub1g_rx_frame.sub1g_addr, status);
        }
        break;
    }

    case SUB1G_OTA_CMD_DATA_ACK: // OTA???????
    {
        if (sub1g_rx_frame.data_length >= 3)
        {
            uint16_t packet_num = ((uint16_t)sub1g_rx_frame.data_content[0] << 8) | sub1g_rx_frame.data_content[1];
            uint8_t status = sub1g_rx_frame.data_content[2];
            ota_handle_sub1g_data_ack(sub1g_rx_frame.sub1g_addr, packet_num, status);
        }
        break;
    }

    case SUB1G_OTA_CMD_CANCEL_FROM_DEV: // ??????????
    {
        DEBUG_PRINTF("[SUB1G] OTA cancel from device: 0x%06lX\r\n", sub1g_rx_frame.sub1g_addr);
        ota_handle_sub1g_cancel(sub1g_rx_frame.sub1g_addr);
        break;
    }

    case SUB1G_OTA_CMD_VERSION_REPORT: // 0x77 - ???????·?
    {
        if (sub1g_rx_frame.data_length >= 2)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            char sub1g_version[VERSION_STRING_MAX_LEN + 1] = {0};
            char mcu_version[VERSION_STRING_MAX_LEN + 1] = {0};

            // 解析Sub1G版本 (长度前缀 + 字符串)
            uint8_t sub1g_ver_len = data[index++];
            if (sub1g_ver_len > 0 && sub1g_ver_len <= VERSION_STRING_MAX_LEN && (index + sub1g_ver_len) <= sub1g_rx_frame.data_length)
            {
                memcpy(sub1g_version, &data[index], sub1g_ver_len);
                sub1g_version[sub1g_ver_len] = '\0';
                index += sub1g_ver_len;
            }

            // 解析MCU版本 (长度前缀 + 字符串)
            if (index < sub1g_rx_frame.data_length)
            {
                uint8_t mcu_ver_len = data[index++];
                if (mcu_ver_len > 0 && mcu_ver_len <= VERSION_STRING_MAX_LEN && (index + mcu_ver_len) <= sub1g_rx_frame.data_length)
                {
                    memcpy(mcu_version, &data[index], mcu_ver_len);
                    mcu_version[mcu_ver_len] = '\0';
                }
            }

            DEBUG_PRINTF("[SUB1G] Version report from 0x%06lX: Sub1G=%s, MCU=%s\r\n", sub1g_rx_frame.sub1g_addr, sub1g_version, mcu_version);

            // 通知OTA管理器处理版本上报
            ota_handle_sub1g_version_report(sub1g_rx_frame.sub1g_addr, sub1g_version, mcu_version);
        }
        break;
    }

    default:
        // 未知命令码忽略
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : uint8_t inv_request_pair_list_add(...)
 Input       :
 Output      : INV请求配对列表索引，失败返回UNPAIRED_DEVICE_MAX_NUM
 Description : 将设备添加到INV请求配对列表
---------------------------------------------------------------------------*/
uint8_t inv_request_pair_list_add(uint32_t sub1g_addr, const char *device_sn, uint8_t product_model)
{
    int8_t existing_index;
    uint8_t free_slot = UNPAIRED_DEVICE_MAX_NUM;

    // 检查是否已存在
    existing_index = inv_request_pair_list_find_by_addr(sub1g_addr);
    if (existing_index >= 0)
    {
        // 已存在则刷新10秒超时计时器并返回
        sys_param.inv_request_pair_list[existing_index].unpaired_updata_ms = 0;
        return existing_index;
    }

    // 查找空闲槽位
    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (!sys_param.inv_request_pair_list[i].is_valid)
        {
            free_slot = i;
            break;
        }
    }

    if (free_slot >= UNPAIRED_DEVICE_MAX_NUM)
    {
        return UNPAIRED_DEVICE_MAX_NUM; // 列表已满
    }

    // 填入设备信息
    sys_param.inv_request_pair_list[free_slot].sub1g_addr = sub1g_addr;
    strncpy(sys_param.inv_request_pair_list[free_slot].device_sn, device_sn, SN_LENGTH);
    sys_param.inv_request_pair_list[free_slot].device_sn[SN_LENGTH] = '\0';
    sys_param.inv_request_pair_list[free_slot].product_model = product_model;
    sys_param.inv_request_pair_list[free_slot].unpaired_updata_ms = 0;
    sys_param.inv_request_pair_list[free_slot].is_valid = true;

    return free_slot;
}

/*---------------------------------------------------------------------------
 Name        : void inv_request_pair_list_remove(uint32_t sub1g_addr)
 Input       : sub1g_addr - SUB1G地址
 Output      : ??
 Description : 从INV请求配对列表移除设备
---------------------------------------------------------------------------*/
void inv_request_pair_list_remove(uint32_t sub1g_addr)
{
    int8_t index = inv_request_pair_list_find_by_addr(sub1g_addr);
    if (index >= 0)
    {
        sys_param.inv_request_pair_list[index].is_valid = false;
        sys_param.inv_request_pair_list[index].sub1g_addr = 0;
    }
}

/*---------------------------------------------------------------------------
    Name        : int8_t inv_request_pair_list_find_by_addr(uint32_t sub1g_addr)
    Input       :
    Output      : 找到返回索引，未找到返回-1
    Description : 按地址查找INV请求配对设备
---------------------------------------------------------------------------*/
int8_t inv_request_pair_list_find_by_addr(uint32_t sub1g_addr)
{
    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.inv_request_pair_list[i].is_valid && sys_param.inv_request_pair_list[i].sub1g_addr == sub1g_addr)
        {
            return i;
        }
    }

    return -1;
}

// ================== 用户配对列表和绑定/解绑操作函数 ==================

/*---------------------------------------------------------------------------
 Name        : bool user_pair_list_add(const char *device_sn)
 Input       : device_sn - 设备SN
 Output      : true=成功, false=失败
 Description : 将设备SN加入用户配对列表，同时预分配SIID到paired_inv_info
               以便设备上线时不经过扫描直接完成绑定（sub1g_addr后续补全）
---------------------------------------------------------------------------*/
bool user_pair_list_add(const char *device_sn)
{
    if (device_sn == NULL || strlen(device_sn) == 0 || strlen(device_sn) > SN_LENGTH)
    {
        return false;
    }

    int8_t existing_index = user_pair_list_find_by_sn(device_sn);
    if (existing_index >= 0)
    {
        // printf("User pair SN already exists: %s\r\n", device_sn);
        return true;
    }

    // 查找空闲槽位
    uint8_t free_slot = USER_PAIR_LIST_MAX_NUM;
    for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
    {
        if (!sys_param.user_pair_list[i].is_valid)
        {
            free_slot = i;
            break;
        }
    }

    if (free_slot >= USER_PAIR_LIST_MAX_NUM)
    {
        // printf("User pair list full\r\n");
        return false; // 列表已满
    }

    // 写入设备SN并标记有效
    strncpy(sys_param.user_pair_list[free_slot].device_sn, device_sn, SN_LENGTH);
    sys_param.user_pair_list[free_slot].device_sn[SN_LENGTH] = '\0';
    sys_param.user_pair_list[free_slot].is_valid = true;

    // printf("WIFI ADD user_pair: %s, id=%d, is_valid =%d\r\n", device_sn, free_slot, sys_param.user_pair_list[free_slot].is_valid);

    // 同步写入EEPROM
    if (eeprom_save_user_pair_device_by_sn(device_sn) != 0)
    {
        // // EEPROM写入失败则回滚内存
        // printf("Warning: Failed to save user pair to EEPROM: %s\r\n", device_sn);
        sys_param.user_pair_list[free_slot].is_valid = false;
        sys_param.user_pair_list[free_slot].device_sn[0] = '\0';
        return false;
    }

    // 预分配SIID到paired_inv_info，使设备上电后能自动恢复绑定
    uint8_t siid = eeprom_add_device_by_sn_only(device_sn);
    if (siid == 0)
    {
        // SIID分配失败不阻止添加（仅预分配，非必须）
        // 设备上线后会正常走绑定流程
        // printf("Warning: Failed to pre-allocate SIID for SN: %s\r\n", device_sn);
    }
    // else
    // {
    //     printf("Pre-allocated SIID %d for SN: %s\r\n", siid, device_sn);
    // }

    return true;
}

/*---------------------------------------------------------------------------
 Name        : int8_t user_pair_list_find_by_sn(const char *device_sn)
 Input       : device_sn - 设备SN
 Output      : 找到返回索引，未找到返回-1
 Description : 按SN查找用户配对列表
---------------------------------------------------------------------------*/
int8_t user_pair_list_find_by_sn(const char *device_sn)
{
    if (device_sn == NULL || strlen(device_sn) == 0)
    {
        return -1;
    }

    for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
    {
        if (sys_param.user_pair_list[i].is_valid)
        {
            // 按实际SN长度比较，避免截断匹配错误
            size_t len = strlen(sys_param.user_pair_list[i].device_sn);
            if (strncmp(sys_param.user_pair_list[i].device_sn, device_sn, len) == 0)
            {
                return i;
            }
        }
    }

    return -1;
}

/*---------------------------------------------------------------------------
 Name        : void user_pair_list_remove_by_sn(const char *device_sn)
 Input       : device_sn - ???SN
 Output      : ??
 Description : 按SN从用户配对列表删除，并清除EEPROM记录
---------------------------------------------------------------------------*/
void user_pair_list_remove_by_sn(const char *device_sn)
{
    if (device_sn == NULL || strlen(device_sn) == 0)
    {
        return;
    }

    int8_t index = user_pair_list_find_by_sn(device_sn);
    if (index < 0)
    {
        return;
    }

    // 清除EEPROM记录
    if (eeprom_clear_user_pair_list_device_by_sn(device_sn) != 0)
    {
        // printf("Warning: Failed to clear user pair from EEPROM: %s\r\n", device_sn);
    }

    // 清空缓冲区
    sys_param.user_pair_list[index].is_valid = false;
    sys_param.user_pair_list[index].device_sn[0] = '\0';

    // printf("User pair removed: SN=%s\r\n", device_sn);
}

/*---------------------------------------------------------------------------
Name        : bool user_bind_device(uint32_t sub1g_addr)
Input       : sub1g_addr - ????SUB1G???
Output      : true=??????false=?????
Description : ??????APP?????????????
            1. 从未配对列表查找设备信息
            2. 调用eeprom_add_device()写入EEPROM (自动分配SIID并更新sys_param)
            3. 设置配对确认窗口期 (期间重复收到广播仍回应绑定)
            4. 从未配对列表移除设备
            5. 发送绑定命令到设备
---------------------------------------------------------------------------*/
bool user_bind_device(uint32_t sub1g_addr)
{
    int8_t unpaired_index;
    uint8_t siid;
    uint8_t inv_index;

    // 检查是否已绑定
    if (find_inv_index_by_sub1g_addr(sub1g_addr) < INV_DEVICE_MAX_NUM)
    {
        return false; // 已绑定
    }

    // 从未配对列表查找设备信息
    unpaired_index = inv_request_pair_list_find_by_addr(sub1g_addr);
    if (unpaired_index < 0)
    {
        return false; // 未找到设备
    }

    unpaired_device_t *device = &sys_param.inv_request_pair_list[unpaired_index];
    device->paired_unvalid_ms = 2000; // 2秒内重复收到广播仍回应绑定命令

    // 调用eeprom_add_device写入设备
    siid = eeprom_add_device(device->device_sn, sub1g_addr, device->product_model);
    if (siid == 0)
    {
        return false; // 写入失败 (EEPROM已满或其他错误)
    }

    inv_index = find_inv_index_by_sub1g_addr(sub1g_addr);

    // 更新在线状态
    sys_param.paired_inv_info[inv_index].online_state = 2; // 设备在线

    // 发送绑定命令到设备
    sub1g_send_bind(sub1g_addr);

    // 保留用户配对列表中的设备SN（不删除）
    // user_pair_list_remove_by_sn(device->device_sn);

    return true;
}

/*---------------------------------------------------------------------------
Name        : bool user_unbind_device_by_sub1g_addr(uint32_t sub1g_addr)
Input       : sub1g_addr - ?????SUB1G???
Output      : true=???????false=??????
Description : ??????APP?????????????
---------------------------------------------------------------------------*/
bool user_unbind_device_by_sub1g_addr(uint32_t sub1g_addr)
{
    uint8_t inv_index = find_inv_index_by_sub1g_addr(sub1g_addr);

    if (inv_index >= INV_DEVICE_MAX_NUM)
    {
        return false;
    }

    // 获取设备SN用于后续清理用户配对列表
    char device_sn[SN_LENGTH + 1];
    strncpy(device_sn, sys_param.paired_inv_info[inv_index].device_sn, SN_LENGTH);
    device_sn[SN_LENGTH] = '\0';

    // 从EEPROM删除设备记录
    if (eeprom_remove_device_by_sub1g_addr(sub1g_addr) != 0)
    {
        return false;
    }

    // 清除内存中的设备记录
    memset(&sys_param.paired_inv_info[inv_index], 0, sizeof(inv_device_t));
    sys_param.paired_inv_info[inv_index].is_valid = false;

    // 同时清除用户配对列表中的SN
    eeprom_clear_user_pair_list_device_by_sn(device_sn);

    // 发送解绑命令到设备
    sub1g_send_unbind(sub1g_addr);

    user_pair_list_remove_by_sn(device_sn);

    return true;
}

bool user_unbind_device_by_sn(const char *device_sn)
{
    if (device_sn == NULL || strlen(device_sn) == 0)
    {
        return false;
    }

    // 在paired_inv_info中查找匹配设备
    for (uint8_t i = 0; i < 8; i++)
    {
        if (strcmp(sys_param.paired_inv_info[i].device_sn, device_sn) == 0 && sys_param.paired_inv_info[i].is_valid)
        {
            // 预分配状态（EEPROM有记录但sub1g_addr为0）
            if (sys_param.paired_inv_info[i].sub1g_addr == 0)
            {
                // 通过SIID从EEPROM删除记录
                uint8_t siid = sys_param.paired_inv_info[i].siid;

                // 清除EEPROM中的设备记录
                eeprom_remove_device_by_siid(siid);

                // 清空内存
                memset(&sys_param.paired_inv_info[i], 0, sizeof(inv_device_t));
                sys_param.paired_inv_info[i].is_valid = false;

                // 清除用户配对列表中的SN
                eeprom_clear_user_pair_list_device_by_sn(device_sn);
                user_pair_list_remove_by_sn(device_sn);

                return true;
            }
            else
            {
                // 正常已绑定设备按addr解绑
                return user_unbind_device_by_sub1g_addr(sys_param.paired_inv_info[i].sub1g_addr);
            }
        }
    }

    // 设备不在paired_inv_info中，尝试清理user_pair_list
    for (uint8_t i = 0; i < 8; i++)
    {
        if (strcmp(sys_param.user_pair_list[i].device_sn, device_sn) == 0)
        {
            // 清除用户配对列表中的SN
            eeprom_clear_user_pair_list_device_by_sn(device_sn);
            user_pair_list_remove_by_sn(device_sn);
            return true;
        }
    }

    return false;
}

const char *get_paired_device_list_string(void)
{
    uint16_t offset = 0;
    bool first_device = true;
    uint8_t device_count = 0;

    // 逐字节发送
    memset(paired_device_list_buffer, 0, DEVICE_LIST_BUFFER_SIZE);

    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid &&
            sys_param.paired_inv_info[i].siid >= SIID_MIN &&
            sys_param.paired_inv_info[i].siid <= SIID_MAX)
        {
            // 检查缓冲区是否有足够空间
            // 每条格式: siid(1) + "," + SN(15) + ";" = 最多18字节
            if (offset + 18 >= DEVICE_LIST_BUFFER_SIZE)
            {
                break; // 缓冲区空间不足，停止添加
            }

            // 非第一条记录前加分隔符
            if (!first_device)
            {
                paired_device_list_buffer[offset++] = ';';
            }
            first_device = false;

            // 格式: "SIID,SN;SIID,SN;SIID,SN"
            offset += sprintf(paired_device_list_buffer + offset, "%d,%s",
                              sys_param.paired_inv_info[i].siid,
                              sys_param.paired_inv_info[i].device_sn);
            device_count++;
        }
    }
    // 追加结尾符确保字符串合法
    if (device_count == 0)
    {
        paired_device_list_buffer[0] = '\0';
    }

    return paired_device_list_buffer;
}

/*---------------------------------------------------------------------------
 Name        : const char *get_inv_request_pair_list_string(void)
 Output      : INV???????????§???????
 Description : ???INV???????????§??????????????ADDR,MODEL;ADDR,MODEL;...??
---------------------------------------------------------------------------*/
const char *get_inv_request_pair_list_string(void)
{
    uint16_t offset = 0;
    bool first_device = true;
    uint8_t device_count = 0;

    // 清空缓冲区
    memset(unpaired_device_list_buffer, 0, DEVICE_LIST_BUFFER_SIZE);

    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.inv_request_pair_list[i].is_valid)
        {
            // 检查缓冲区是否有足够空间
            if (offset + 19 >= DEVICE_LIST_BUFFER_SIZE)
            {
                break; // 缓冲区空间不足，停止添加
            }

            // 非第一条记录前加分隔符
            if (!first_device)
            {
                unpaired_device_list_buffer[offset++] = ';';
            }
            first_device = false;

            // 获取产品型号字符串
            const char *model_str = product_model_code_to_string(sys_param.inv_request_pair_list[i].product_model);

            // 格式: "XXXXXX,型号"
            offset += sprintf(unpaired_device_list_buffer + offset, "%s,%s", sys_param.inv_request_pair_list[i].device_sn, model_str);
            device_count++;
        }
    }

    // 无设备时确保字符串为空
    if (device_count == 0)
    {
        unpaired_device_list_buffer[0] = '\0';
    }

    return unpaired_device_list_buffer;
}

/*---------------------------------------------------------------------------
 Name        : const char *get_user_pair_list_string(void)
 Output      : 用户配对列表字符串
 Description : 将用户配对列表序列化为字符串，格式"SN1,SN2,SN3"
---------------------------------------------------------------------------*/
const char *get_user_pair_list_string(void)
{
    uint16_t offset = 0;
    bool first_device = true;
    uint8_t device_count = 0;

    // 清空缓冲区
    memset(user_pair_list_buffer, 0, DEVICE_LIST_BUFFER_SIZE);

    for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
    {
        if (sys_param.user_pair_list[i].is_valid)
        {
            // 检查缓冲区是否有足够空间
            if (offset + SN_LENGTH + 2 >= DEVICE_LIST_BUFFER_SIZE)
            {
                break;
            }

            // 非第一条记录前加逗号分隔
            if (!first_device)
            {
                user_pair_list_buffer[offset++] = ',';
            }
            first_device = false;

            // 写入SN
            offset += sprintf(user_pair_list_buffer + offset, "%s", sys_param.user_pair_list[i].device_sn);
            device_count++;
        }
    }

    // 无设备时确保字符串为空
    if (device_count == 0)
    {
        user_pair_list_buffer[0] = '\0';
    }

    return user_pair_list_buffer;
}

// ================= 以下为 CT 主动发送命令到 sub1g_ct 的函数 =================

/*---------------------------------------------------------------------------
 Name        : bool uart1_tx_queue_push(const uint8_t *data, uint16_t len)
 Input       : data - 待发送的数据指针
               len  - 数据长度
 Output      : true=入队成功, false=队列已满或参数错误
 Description : 将消息加入发送队列
---------------------------------------------------------------------------*/
bool uart1_tx_queue_push(const uint8_t *data, uint16_t len)
{
    // 参数校验
    if (data == NULL || len == 0 || len > UART1_TX_MSG_MAX_LEN)
    {
        DEBUG_PRINTF("uart1_tx_queue_push: error\n");
        return false;
    }

    // 检查队列是否已满
    if (g_uart1_tx_queue.count >= UART1_TX_QUEUE_SIZE)
    {
        // 队列已满，丢弃此次发送
        // sys_param.sub1g.state = 5;  // 5 = 队列满（调试用）
        DEBUG_PRINTF("uart1_tx_queue_push: queue full\n");
        return false;
    }

    // 将数据写入当前空闲槽位
    uart1_tx_msg_t *slot = &g_uart1_tx_queue.buffer[g_uart1_tx_queue.write_index];
    memcpy(slot->data, data, len);
    slot->length = len;

    // 写指针前移（环形）
    g_uart1_tx_queue.write_index = (g_uart1_tx_queue.write_index + 1) % UART1_TX_QUEUE_SIZE;

    // 消息计数加一
    g_uart1_tx_queue.count++;

    return true;
}

/*---------------------------------------------------------------------------
 Name        : static bool uart1_tx_queue_pop(uart1_tx_msg_t *msg)
 Input       : msg - 出队消息的目标缓冲区
 Output      : true=出队成功, false=队列空
 Description : 从队列取出一条消息，使用memcpy复制到调用者缓冲区
               避免直接操作buffer指针，防移指针提前推进导致CPU乱序访问
---------------------------------------------------------------------------*/
static bool uart1_tx_queue_pop(uart1_tx_msg_t *msg)
{
    // 队列为空时直接返回
    if (g_uart1_tx_queue.count == 0)
    {
        return false;
    }

    // 入队发送
    memcpy(msg, &g_uart1_tx_queue.buffer[g_uart1_tx_queue.read_index], sizeof(uart1_tx_msg_t));

    // 读指针前移（环形）
    g_uart1_tx_queue.read_index = (g_uart1_tx_queue.read_index + 1) % UART1_TX_QUEUE_SIZE;

    // 消息计数减一
    g_uart1_tx_queue.count--;

    return true;
}

/*---------------------------------------------------------------------------
 Name        : void uart1_tx_queue_process(void)
 Input       : ??
 Output      : ??
 Description : 主循环调用，从队列取出消息并通过UART1逐字节发送
---------------------------------------------------------------------------*/
void uart1_tx_queue_process(void)
{
    uart1_tx_msg_t msg;

    // 正在发送 或 队列为空时直接返回
    if (g_uart1_tx_queue.is_sending || g_uart1_tx_queue.count == 0)
    {
        return;
    }

    // 从队列取出一条消息，失败则返回
    if (!uart1_tx_queue_pop(&msg))
    {
        return;
    }

    // 加发送锁
    g_uart1_tx_queue.is_sending = true;

    // printf("uart1_tx: %d bytes: ", msg.length);
    // for (uint16_t i = 0; i < msg.length; i++)
    // {
    //     printf("%02X ", msg.data[i]);
    // }
    // printf("\r\n");

    // 逐字节发送
    for (uint16_t i = 0; i < msg.length; i++)
    {
        USART_WriteData(CM_USART1, msg.data[i]);

        // 等待发送寄存器空
        while (RESET == USART_GetStatus(CM_USART1, USART_FLAG_TX_EMPTY))
        {
            // 轮询等待发送完成
        }
    }

    // 发送完成，释放发送锁
    g_uart1_tx_queue.is_sending = false;
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_bind(uint32_t target_addr)
 Input       : target_addr - 目标设备SUB1G地址
 Output      : ??
 Description : 发送绑定命令0x02，帧格式：帧头 + 目标地址 + 长度(1) + 命令码
---------------------------------------------------------------------------*/
void sub1g_send_bind(uint32_t target_addr)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    // 帧头
    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    // 目标SUB1G地址
    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    // 长度字段=1（仅命令码1B）
    temp_buffer[5] = 1;

    // 命令码
    temp_buffer[6] = CMD_CT_BIND;

    // 入队发送
    if (!uart1_tx_queue_push(temp_buffer, 7))
    {
        // 队列满时丢弃
        // sys_param.sub1g.state = 5;  // 5 = 队列满（调试用）
    }

    // printf("[Bind]sub1g_send_bind: %08X", target_addr);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_unbind(uint32_t target_addr)
 Input       :
 Output      : ??
 Description : ??????????0x03?? ??????? + ????? + ????(1) + ??????
---------------------------------------------------------------------------*/
void sub1g_send_unbind(uint32_t target_addr)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;
    temp_buffer[5] = 1;
    temp_buffer[6] = CMD_CT_UNBIND;

    uart1_tx_queue_push(temp_buffer, 7);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_broadcast_unbind_by_sn(const char *device_sn)
 Input       : device_sn - ???SN
 Output      : ??
 Description : ?????????0x03?????SN
               ??????? + ?????(000000) + ????(n+1) + ??????(0x03) + SN(n???)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_unbind_by_sn(const char *device_sn)
{
    if (device_sn == NULL)
        return;

    uint8_t sn_len = strlen(device_sn);
    if (sn_len == 0 || sn_len > SN_LENGTH)
        return;

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];
    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x00; // 广播地址000000
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;
    temp_buffer[5] = sn_len + 1; // 长度 = SN字节数 + 命令码占1字节

    temp_buffer[6] = CMD_CT_BROADCAST_UNBIND_SN; // 0x14

    // 填入SN
    memcpy(&temp_buffer[7], device_sn, sn_len);

    uart1_tx_queue_push(temp_buffer, 7 + sn_len);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_broadcast_single_phase_power(float power, uint8_t inv_count, uint32_t report_addr)
 Input       : power - 单相并网功率(W)
               inv_count - 当前相已在线微逆数量
               report_addr - 指定微逆的子地址（广播给谁）
 Output      : ??
 Description : 帧格式：帧头 + 广播地址 + 长度(7) + 命令码 + 功率(2B) + 设备数(1B) + 子地址(3B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_single_phase_power(float power, uint8_t inv_count, uint32_t report_addr)
{
    if (g_ota_manager.disable_broadcast) // OTA期间禁止广播
    {
        return;
    }

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];
    int16_t power_int = (int16_t)power;

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x00; // 广播地址
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;

    temp_buffer[5] = 7;

    temp_buffer[6] = CMD_CT_BROADCAST_SINGLE_PHASE_POWER;

    temp_buffer[7] = (power_int >> 8) & 0xFF;
    temp_buffer[8] = power_int & 0xFF;

    temp_buffer[9] = inv_count;
    temp_buffer[10] = (report_addr >> 16) & 0xFF;
    temp_buffer[11] = (report_addr >> 8) & 0xFF;
    temp_buffer[12] = report_addr & 0xFF;

    uart1_tx_queue_push(temp_buffer, 13);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_broadcast_three_phase_power(...)
 Input       : power_ct1/ct2/ct3 - 三相功率(W)，带符号整数
               report_addr - 目标子地址
 Output      : ??
 Description : 帧格式：帧头 + 广播地址 + 长度(10) + 命令码 + A相功率(2B) + B相功率(2B) + C相功率(2B) + 子地址(3B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_three_phase_power(int16_t power_ct1, int16_t power_ct2, int16_t power_ct3, uint32_t report_addr)
{
    if (g_ota_manager.disable_broadcast) // OTA期间禁止广播
    {
        return;
    }
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x00; // 广播地址
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;

    temp_buffer[5] = 10;

    temp_buffer[6] = CMD_CT_BROADCAST_THREE_PHASE_POWER;

    temp_buffer[7] = (power_ct1 >> 8) & 0xFF;
    temp_buffer[8] = power_ct1 & 0xFF;
    temp_buffer[9] = (power_ct2 >> 8) & 0xFF;
    temp_buffer[10] = power_ct2 & 0xFF;
    temp_buffer[11] = (power_ct3 >> 8) & 0xFF;
    temp_buffer[12] = power_ct3 & 0xFF;

    temp_buffer[13] = (report_addr >> 16) & 0xFF;
    temp_buffer[14] = (report_addr >> 8) & 0xFF;
    temp_buffer[15] = report_addr & 0xFF;

    uart1_tx_queue_push(temp_buffer, 16);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_broadcast_date(const char *date)
 Input       : date - 日期字符串 "YYYY-MM-DD" (10字节)
 Output      : ??
 Description : 帧格式：帧头 + 广播地址 + 长度(5) + 命令码(1B) + 年(2B) + 月(1B) + 日(1B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_date(const char *date)
{
    if (g_ota_manager.disable_broadcast) // OTA期间禁止广播
    {
        return;
    }

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    // 参数校验，date不能为空且至少10字节
    if (date == NULL || strlen(date) < 10)
    {
        return;
    }

    // 解析年月日
    char year_str[5], month_str[3], day_str[3];
    uint16_t year;
    uint8_t month, day;

    // 按"YYYY-MM-DD"格式分别截取年月日字符串
    strncpy(year_str, date, 4);
    year_str[4] = '\0';
    strncpy(month_str, date + 5, 2);
    month_str[2] = '\0';
    strncpy(day_str, date + 8, 2);
    day_str[2] = '\0';

    // 字符串转整数
    year = (uint16_t)atoi(year_str);
    month = (uint8_t)atoi(month_str);
    day = (uint8_t)atoi(day_str);
    // 帧头
    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    // 0x0000广播地址
    temp_buffer[2] = 0x00;
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;

    // 数据长度=命令码1B + 年2B + 月1B + 日1B = 5B
    temp_buffer[5] = 5;

    // 命令码
    temp_buffer[6] = CMD_CT_BROADCAST_DATE;

    // 年份2字节大端
    temp_buffer[7] = (year >> 8) & 0xFF;
    temp_buffer[8] = year & 0xFF;

    // 月份1字节
    temp_buffer[9] = month;

    // 日期1字节
    temp_buffer[10] = day;

    uart1_tx_queue_push(temp_buffer, 11); // 帧头2 + 地址3 + 长度1 + 命令码1 + 日期4 = 11
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_broadcast_phase_inv_count(uint8_t count_ct1, uint8_t count_ct2, uint8_t count_ct3)
 Input       : count_a/b/c - A/B/C????????
 Output      : ??
 Description : ??????? + ????? + ????(4) + ?????? + A?????(1B) + B?????(1B) + C?????(1B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_phase_inv_count(uint8_t count_ct1, uint8_t count_ct2, uint8_t count_ct3)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = 0x00; // 广播地址
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;

    temp_buffer[5] = 4;

    temp_buffer[6] = CMD_CT_BROADCAST_PHASE_INV_COUNT;

    temp_buffer[7] = count_ct1;
    temp_buffer[8] = count_ct2;
    temp_buffer[9] = count_ct3;

    uart1_tx_queue_push(temp_buffer, 10);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_set_antiflow(uint32_t target_addr, bool enable)
 Input       : target_addr - 目标设备SUB1G地址
               enable - ??????????????0=????1=??????
 Output      : ??
 Description : ??????? + ????? + ????(2) + ?????? + ????(1B)
---------------------------------------------------------------------------*/
void sub1g_send_set_antiflow(uint32_t target_addr, bool enable)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;
    temp_buffer[5] = 2;
    temp_buffer[6] = CMD_CT_SET_ANTIFLOW;
    temp_buffer[7] = (uint8_t)enable;

    uart1_tx_queue_push(temp_buffer, 8);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_set_power_switch(uint32_t target_addr, bool enable)
 Input       : target_addr - ???SUB1G???
 Output      : ??
 Description : ????????????C??????0x21????????? + ????? + ????(2) + ?????? + ????(1B)
---------------------------------------------------------------------------*/
void sub1g_send_set_power_switch(uint32_t target_addr, bool enable)
{
    if (g_ota_manager.disable_broadcast) // OTA期间禁止广播
    {
        return;
    }

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    temp_buffer[5] = 2;

    temp_buffer[6] = CMD_CT_SET_POWER_SWITCH;

    temp_buffer[7] = (uint8_t)enable;

    uart1_tx_queue_push(temp_buffer, 8);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_set_inv_phase(uint32_t target_addr, uint8_t phase)
 Input       : target_addr - ???SUB1G???
               phase - ???? (0=?????, 1=CT1??, 2=CT2??, 3=CT3??)
 Output      : ??
 Description : ??????? + ????? + ????(2) + ?????? + ????(1B)
---------------------------------------------------------------------------*/
void sub1g_send_set_inv_phase(uint32_t target_addr, uint8_t phase)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    temp_buffer[5] = 2;

    temp_buffer[6] = CMD_CT_SET_INV_PHASE;

    temp_buffer[7] = phase;

    uart1_tx_queue_push(temp_buffer, 8);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_enable_phase_identify(uint32_t target_addr,uint8_t time, uint16_t power, uint8_t power_interval)
 Input       : target_addr - ???SUB1G???
               time - 相序识别持续时间 (单位: s)
               power - 识别功率阈值(W)
               power_interval - 识别功率间隔 (单位: 10ms，n个周期开，n个周期关)
 Output      : ??
 Description : 帧格式：帧头 + 目标地址 + 长度(5) + 命令码(0x23) + 识别时间(1B) + 功率(2B) + 间隔×10ms(1B)
---------------------------------------------------------------------------*/
void sub1g_send_enable_phase_identify(uint32_t target_addr, uint8_t time, uint16_t power, uint8_t power_interval)
{
    // 已有识别任务进行中，拒绝新请求
    if (sys_param.fft_identify.is_ffting == 1)
    {
#ifdef FFT_DEBUG_PRINT
        printf("Busy: Device 0x%06X is identifying\r\n", sys_param.fft_identify.sub1g_addr);
#endif
        return;
    }

    // // 查找设备索引
    // uint8_t inv_index = INV_DEVICE_MAX_NUM;
    // inv_index = find_inv_index_by_sub1g_addr(target_addr);
    // if (inv_index >= INV_DEVICE_MAX_NUM)
    // {
    //     // printf("Device not found: 0x%06X\r\n", target_addr);
    //     return;
    // }

    // // 检查设备在线状态
    // if (sys_param.paired_inv_info[inv_index].online_state != 2)
    // {
    //     // printf("Device offline: 0x%06X\r\n", target_addr);
    //     return;
    // }

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    temp_buffer[5] = 5;

    temp_buffer[6] = CMD_CT_ENABLE_PHASE_IDENTIFY; // 0x23

    temp_buffer[7] = time;

    temp_buffer[8] = power >> 8;
    temp_buffer[9] = power & 0xFF;

    temp_buffer[10] = (power_interval - 1);

    uart1_tx_queue_push(temp_buffer, 11);

    // 更新 fft_identify 状态
    sys_param.fft_identify.is_ffting = 1; // 标记识别中
    sys_param.fft_identify.power = power;

    fft_reset_all_channels();

#ifdef FFT_DEBUG_PRINT
    printf("Start identify: Device 0x%06X, power=%d, power_interval=%d\r\n", target_addr, power, power_interval);
#endif
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_debug_mode(uint32_t target_addr)
 Input       : target_addr - ???SUB1G???
 Output      : ??
 Description : ?????????????????0x29?? ??????? + ????? + ????(1) + ??????
---------------------------------------------------------------------------*/
void sub1g_send_debug_mode(uint32_t target_addr)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;
    temp_buffer[5] = 1;
    temp_buffer[6] = CMD_CT_DEBUG_MODE;

    uart1_tx_queue_push(temp_buffer, 7);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_get_version(void)
 Input       : ??
 Output      : ??
 Description : ??????CT??SUB1G?·????????0x41?? ??????? + ?????(0x111111) + ????(1) + ??????
---------------------------------------------------------------------------*/
void sub1g_send_get_version(void)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x11; // 广播地址0x111111(CT专用的SUB1G地址)
    temp_buffer[3] = 0x11;
    temp_buffer[4] = 0x11;
    temp_buffer[5] = 1;
    temp_buffer[6] = CMD_CT_SUB1G_VERSION;

    uart1_tx_queue_push(temp_buffer, 7);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_get_rssi(void)
 Input       : ??
 Output      : ??
 Description : ??????CT??SUB1G RSSI????0x42?? ??????? + ?????(0x111111) + ????(1) + ??????
---------------------------------------------------------------------------*/
void sub1g_send_get_rssi(void)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x11; // 广播地址0x111111(CT专用的SUB1G地址)
    temp_buffer[3] = 0x11;
    temp_buffer[4] = 0x11;
    temp_buffer[5] = 1;
    temp_buffer[6] = CMD_CT_SUB1G_RSSI;

    uart1_tx_queue_push(temp_buffer, 7);
}

/*---------------------------------------------------------------------------
 Name        : static void sub1g_send_record_channel(void)
 Input       : ??
 Output      : ??
 Description : ??????CT??SUB1G RSSI????0x44?? ??????? + ?????(0x111111) + ????(2) + ?????? + CT???????
---------------------------------------------------------------------------*/
static void sub1g_send_record_channel(void)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x11; // 广播地址0x111111(CT专用的SUB1G地址)
    temp_buffer[3] = 0x11;
    temp_buffer[4] = 0x11;
    temp_buffer[5] = 2;
    temp_buffer[6] = CMD_CT_SUB1G_CHANNEL_REWORD;
    temp_buffer[7] = sys_param.sub1g.channel_index;

    uart1_tx_queue_push(temp_buffer, 8);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_force_channel(void)
 Input       : ??
 Output      : ??
 Description : ??????? + ?????(0x111111) + ????(2) + ?????? + CT???????
---------------------------------------------------------------------------*/
void sub1g_send_force_channel(void)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x11; // 广播地址0x111111(CT专用的SUB1G地址)
    temp_buffer[3] = 0x11;
    temp_buffer[4] = 0x11;
    temp_buffer[5] = 2;
    temp_buffer[6] = CMD_CT_SUB1G_CHANNEL_FORCE;
    temp_buffer[7] = sys_param.sub1g.channel_index;

    uart1_tx_queue_push(temp_buffer, 8);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_set_connection_point(uint32_t target_addr, uint8_t connection_point)
 Input       : target_addr - ???SUB1G???
               connection_point - ????? (0=?????, 1=??????)
 Output      : ??
 Description : ????????????????0x24?? ??????? + ????? + ????(2) + ?????? + ?????(1B)
---------------------------------------------------------------------------*/
void sub1g_send_set_connection_point(uint32_t target_addr, uint8_t connection_point)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    temp_buffer[5] = 2;

    temp_buffer[6] = CMD_CT_SET_CONNECTION_POINT;

    temp_buffer[7] = connection_point;

    uart1_tx_queue_push(temp_buffer, 8);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_set_power_limit(uint32_t target_addr, uint16_t power_limit)
 Input       : target_addr - ???SUB1G???
               power_limit - ???????? (W)
 Output      : ??
 Description : ???????¨????????????0x25?? ??????? + ????? + ????(3) + ?????? + ????????(2B?????)
---------------------------------------------------------------------------*/
void sub1g_send_set_power_limit(uint32_t target_addr, uint16_t power_limit)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    temp_buffer[5] = 3;

    temp_buffer[6] = CMD_CT_SET_POWER_LIMIT;

    temp_buffer[7] = (power_limit >> 8) & 0xFF;
    temp_buffer[8] = power_limit & 0xFF;

    uart1_tx_queue_push(temp_buffer, 9);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_clear_data(uint32_t target_addr)
 Input       : target_addr - ???SUB1G???
 Output      : ??
 Description : ???????????????????0x26?? ??????? + ????? + ????(1) + ??????
---------------------------------------------------------------------------*/
void sub1g_send_clear_data(uint32_t target_addr)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    temp_buffer[5] = 1;
    temp_buffer[6] = CMD_CT_CLEAR_DATA;

    uart1_tx_queue_push(temp_buffer, 7);
}

//  ================= 以下为工具函数  =================

/*---------------------------------------------------------------------------
 Name        : uint8_t find_inv_index_by_sub1g_addr(uint32_t sub1g_addr)
 Input       :
 Output      : 找到返回索引0-7，未找到返回INV_DEVICE_MAX_NUM
 Description : 按SUB1G地址在已绑定设备列表中查找索引
---------------------------------------------------------------------------*/
uint8_t find_inv_index_by_sub1g_addr(uint32_t sub1g_addr)
{
    // 遍历已绑定设备列表
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // 找到匹配且有效的记录
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr)
        {
            return i;
        }
    }

    // 未找到，返回无效索引
    return INV_DEVICE_MAX_NUM;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t find_free_inv_slot(void)
 Input       : ??
 Output      : 空闲槽位索引0-7，无空闲槽位返回INV_DEVICE_MAX_NUM
 Description : 查找未使用的设备槽位索引
---------------------------------------------------------------------------*/
uint8_t find_free_inv_slot(void)
{
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (!sys_param.paired_inv_info[i].is_valid)
        {
            return i;
        }
    }

    return INV_DEVICE_MAX_NUM;
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_bytes_to_float(const uint8_t *bytes, float *value)
 Input       : bytes - 4字节大端浮点数据
 Output      : value - 解析后的浮点值
 Description : 将4字节大端数组转换为float（IEEE754）
---------------------------------------------------------------------------*/
void sub1g_bytes_to_float(const uint8_t *bytes, float *value)
{
    union
    {
        float f;
        uint8_t b[4];
    } converter;

    // 大端转小端字节序映射
    converter.b[3] = bytes[0];
    converter.b[2] = bytes[1];
    converter.b[1] = bytes[2];
    converter.b[0] = bytes[3];

    *value = converter.f;
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_float_to_bytes(float value, uint8_t *bytes)
 Input       : value - 浮点值
 Output      : bytes - 4字节大端输出缓冲区
 Description : 将float以大端字节序写入4字节缓冲区
---------------------------------------------------------------------------*/
void sub1g_float_to_bytes(float value, uint8_t *bytes)
{
    union
    {
        float f;
        uint8_t b[4];
    } converter;

    converter.f = value;

    // 小端转大端字节序映射
    bytes[0] = converter.b[3];
    bytes[1] = converter.b[2];
    bytes[2] = converter.b[1];
    bytes[3] = converter.b[0];
}
