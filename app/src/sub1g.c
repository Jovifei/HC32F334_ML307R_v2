//
// Included Files
//
#include <stdlib.h>
#include "board.h"
#include "sub1g.h"
#include "main.h"
#include "fft.h"
#include "string.h"
#include "eeprom.h"
#include "stdio.h"
#include "stdbool.h"
#include "ota_max.h"
#include "debug.h"

// ================= ȫ�ֱ������� =================
sub1g_uart_com_rx_status_t sub1g_status;
sub1g_frame_t sub1g_rx_frame;

// ================= UART1���Ͷ��� =================
//  UART1���Ͷ���ʵ��
static uart1_tx_queue_t g_uart1_tx_queue;

// ��ʱ���ջ��������ж���ʹ�ã�
uint8_t uart1_temp_buffer[SUB1G_MAX_FRAME_SIZE];
uint16_t uart1_temp_count = 0;
uint16_t uart1_expected_length = 0;

// �Ѱ��豸�б���������������豸�б����������û�����б�������
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
 Input       : ��
 Output      : ��
 Description : ��ʼ��SUB1Gģ��Ͷ���
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
 Output      : ��
 Description : USART1�����жϴ������������ֽڽ���֡ͷ������֡���Ƶ����ջ�����
               ֡��ʽ��֡ͷ(0xFACE) + sub1g��ַ(3B) + ����(1B) + ������(1B) + ����(nB)
               ��������֡������frame_received��־������ѭ��������
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
            // �ȴ�֡ͷ��һ�ֽ� 0xFA
            if (received_byte == 0xFA)
            {
                uart1_temp_buffer[uart1_temp_count++] = received_byte;
            }
            break;

        case 1:
            // �ȴ�֡ͷ�ڶ��ֽ� 0xCE
            if (received_byte == 0xCE)
            {
                // ֡ͷ���� FA CE
                uart1_temp_buffer[uart1_temp_count++] = received_byte;
            }
            else if (received_byte == 0xFA)
            {
                // �յ�0xFA����������֡ͷ��ʼ�����ò�����
                uart1_temp_buffer[0] = 0xFA;
                uart1_temp_count = 1;
                uart1_expected_length = 0;
            }
            else
            {
                // �Ƿ��ֽڣ����ý���״̬
                uart1_temp_count = 0;
                uart1_expected_length = 0;
            }
            break;

        case 2:
        case 3:
        case 4:
            // ����SUB1GԴ��ַ��3���ֽ�
            uart1_temp_buffer[uart1_temp_count++] = received_byte;
            break;

        case 5:
            // ���ճ����ֶ�
            uart1_temp_buffer[uart1_temp_count++] = received_byte;

            // ��������֡���� = ֡ͷ(2) + ��ַ(3) + ����(1) + ����(length�ֽ�)
            uart1_expected_length = 6 + received_byte;

            break;

        default:
            // �������������ֽ�ֱ��֡����
            uart1_temp_buffer[uart1_temp_count++] = received_byte;

            // �ж��Ƿ��������֡
            if (uart1_temp_count == uart1_expected_length)
            {
                // ��֤֡ͷ�Ϸ���������
                if (uart1_temp_buffer[0] == 0xFA && uart1_temp_buffer[1] == 0xCE)
                {

                    if (g_ota_manager.ota_in_progress && uart1_temp_buffer[6] == 0x01)
                    {
                        uart1_temp_count = 0;
                        uart1_expected_length = 0;
                    }
                    else
                    {
                        // ��������֡�����ջ��������ñ�־
                        memcpy(sub1g_status.rx_buffer, uart1_temp_buffer, uart1_temp_count);
                        sub1g_status.rx_index = uart1_temp_count;
                        sub1g_status.frame_received = true;
                    }
                }

                // ������Ϻ�����У��ɹ���������״̬
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
 Output      : ��
 Description : ��ѭ�����ã����֡������ɱ�־������
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

        // �������յ���֡
        sub1g_process_received_frame();

        sub1g_status.frame_received = false;
        sub1g_status.rx_index = 0;

        // ����SUB1Gͨ��״̬
        if (sub1g_status.rx_buffer[6] != 0x41 && sub1g_status.rx_buffer[6] != 0x42 && sub1g_status.rx_buffer[6] != 0x43)
        {
            sys_param.sub1g.state = 4; // 4 = ����ԣ�ͨ��������
            sys_param.sub1g.timeout_count = 0;
        }

        sys_param.sub1g.reboot_count = 0;
    }
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_process_received_frame(void)
 Input       : ??
 Output      : ??
 Description : �������յ�������֡������������ַ�����
               �����豸����״̬�����������ϱ��Ⱥ�������
---------------------------------------------------------------------------*/
void sub1g_process_received_frame(void)
{
    uint8_t *data = sub1g_status.rx_buffer;
    uint8_t data_content_length;
    uint8_t inv_index;
    int8_t unpaired_index;

    // ����֡ͷ���ֶ�
    sub1g_rx_frame.frame_header = (data[0] << 8) | data[1];
    sub1g_rx_frame.sub1g_addr = ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | data[4];
    sub1g_rx_frame.data_length = data[5];
    sub1g_rx_frame.command_code = data[6];

    // if (sub1g_rx_frame.sub1g_addr != 0x111111)
    // {
    //     DEBUG_PRINTF("[SUB1G] Received frame: sub1g_addr: %06X: command_code: 0x%02X\r\n", sub1g_rx_frame.sub1g_addr, sub1g_rx_frame.command_code);
    // }

    // У��֡ͷ
    if (sub1g_rx_frame.frame_header != SUB1G_FRAME_HEADER)
    {
        return;
    }

    // �������ݳ��� = ���ݳ����ֶ� - ������(1B)
    data_content_length = sub1g_rx_frame.data_length - 1;

    // ���������ݸ��Ƶ�֡�ṹ��
    if (data_content_length > 0 && data_content_length <= SUB1G_MAX_DATA_SIZE)
    {
        memcpy(sub1g_rx_frame.data_content, &data[7], data_content_length);
    }

    // ����������ַ�����
    switch (sub1g_rx_frame.command_code)
    {
    case CMD_INV_PAIR_BROADCAST: // 0x01 - ΢����Թ㲥
    {
        // ����data_content����Ʒ�ͺ�(1B) + �豸SN(15B)���ͺ�1:800W΢��
        uint8_t product_model = sub1g_rx_frame.data_content[0];

        char device_sn[SN_LENGTH + 1] = {0}; // ĩβ��ʼ��Ϊ0
        memcpy(device_sn, &sub1g_rx_frame.data_content[1], SN_LENGTH);

        // ����SN�з���ĸ�����ַ����ضϵ���Ч�ַ�
        for (uint8_t i = 0; i < SN_LENGTH; i++)
        {
            char c = device_sn[i];
            if (!((c >= '0' && c <= '9') ||
                  (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z')))
            {
                device_sn[i] = '\0'; // �ضϣ������ֽ���Ч
                break;
            }
        }
        device_sn[SN_LENGTH] = '\0'; // ȷ����β

        // printf("Received pairing broadcast from device: %s, model: %d\r\n", device_sn, product_model);

        // ����Ƿ�ΪԤ�����豸��EEPROM���д�SN��sub1g_addrΪ0�ļ�¼��
        int8_t pre_alloc_index = eeprom_find_inv_index_by_sn(device_sn);
        if (pre_alloc_index >= 0 && sys_param.paired_inv_info[pre_alloc_index].sub1g_addr == 0)
        {
            // ===== ���0: Ԥ�����豸��ȫsub1g_addr =====
            // printf("Found pre-allocated device: SN=%s, updating sub1g_addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr);

            // ����EEPROM��¼�е�sub1g_addr���ͺ�
            if (eeprom_update_device_sub1g_addr(device_sn, sub1g_rx_frame.sub1g_addr, product_model) == 0)
            {
                // ����INV��������б�������2��ȷ�ϴ���
                inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);
                if (unpaired_index >= 0)
                {
                    sys_param.inv_request_pair_list[unpaired_index].paired_unvalid_ms = 2000;
                }

                // �������Ͱ�����
                sub1g_send_bind(sub1g_rx_frame.sub1g_addr);

                // printf("Pre-allocated device bound: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr);
            }
            break;
        }

        // ����Ƿ��Ѱ󶨣�����paired_inv_info�У�
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            // printf("Device already paired: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr);
            // ===== ���1: �Ѱ��豸�������� =====
            unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);

            // ����Ƿ���2��ȷ�ϴ�����
            bool in_confirm_window = (unpaired_index >= 0 && sys_param.inv_request_pair_list[unpaired_index].paired_unvalid_ms > 0);

            if (in_confirm_window)
            {
                // ��ȷ�ϴ����ڣ��ط�������
                sub1g_send_bind(sub1g_rx_frame.sub1g_addr);
            }
            else
            {
                // ���ڴ��ڣ�����Ƿ����û�����б�
                int8_t user_pair_index = user_pair_list_find_by_sn(device_sn);

                if (user_pair_index >= 0)
                {
                    // ���û��б��У�������������б��������ذ�
                    inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                    // ����2��ȷ�ϴ���
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
                    // �����û��б������»�����INV��������б�
                    if (unpaired_index >= 0)
                    {
                        // �����б��У�ˢ���ϱ�ʱ��
                        sys_param.inv_request_pair_list[unpaired_index].unpaired_updata_ms = 0;
                    }
                    else
                    {
                        // �����б�����������������б�
                        inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                    }
                }
            }
        }
        else
        {
            // ===== ���2: δ���豸������� =====
            // ����Ƿ����û�����б�
            int8_t user_pair_index = user_pair_list_find_by_sn(device_sn);

            if (user_pair_index >= 0)
            {
                // ���û��б��У�������������б����Զ���
                inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                if (user_bind_device(sub1g_rx_frame.sub1g_addr))
                {
                    // // ��ӡ��־
                    // printf("Device auto-paired: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                }
            }
            else
            {
                // �����û��б������»�����INV��������б�
                unpaired_index = inv_request_pair_list_find_by_addr(sub1g_rx_frame.sub1g_addr);

                if (unpaired_index >= 0)
                {
                    // printf("Device in unpaired list: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                    // �����б��У�ˢ���ϱ�ʱ��
                    sys_param.inv_request_pair_list[unpaired_index].unpaired_updata_ms = 0;
                }
                else
                {
                    // printf("Add New device in unpaired list: SN=%s, Addr=0x%06X\r\n", device_sn, sub1g_rx_frame.sub1g_addr & 0xFFFFFF);
                    // �������豸��INV��������б�
                    inv_request_pair_list_add(sub1g_rx_frame.sub1g_addr, device_sn, product_model);
                }
            }
        }
        break;
    }

    case CMD_INV_REPORT_NORMAL: // 0x50 - ΢���ճ��ϱ�(3s����)
    {
        // ����sub1g_addr�����Ѱ��豸�������������Ѱ󶨵�SUB1G��ַ��
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);
        // printf("Receive device:%06X id=%d updata \r\n", sub1g_rx_frame.sub1g_addr, inv_index);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            // ����data_content���ݲ�����sys_param.paired_inv_info[]
            uint8_t *data = sub1g_rx_frame.data_content;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // ��¼�ϴι��Ϻͷ�����״̬���ڱ�����
            uint32_t fault_param_last = inv->fault_param;
            uint8_t antiflow_enable_last = inv->antiflow_enable;

            // �״��յ�����ʱ��ʼ��ͳ��
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
            // ���������ֶ�

            // Byte 0-1: ��������(W)
            inv->grid_power = ((int16_t)((data[0] << 8) | data[1]));

            // Byte 2: ����״̬
            inv->work_state = data[2];

            // Byte 3-6: ���ϲ���
            inv->fault_param = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6];

            // Byte 7: ����������״̬
            inv->antiflow_enable = data[7];

            // Byte 8: PV·��
            uint8_t pv_num = data[8];
            inv->pv_num = pv_num;

            // ��ȡPV��·���� (2B*n)
            uint16_t offset = 9;
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].power = ((int16_t)((data[offset] << 8) | data[offset + 1])) / 10.0f;
                offset += 2;
            }

            // DEBUG_PRINTF("0X50: inv_index = %d, grid_power = %.2f, work_state = %d, fault_param = %d, antiflow_enable = %d, pv_num = %d, pv0_power = %.2f, pv1_power = %.2f.\n", inv_index, inv->grid_power, inv->work_state, inv->fault_param, inv->antiflow_enable, inv->pv_num, inv->pv[0].power, inv->pv[1].power);

            // ������Ա��
            if (fault_param_last != inv->fault_param || antiflow_enable_last != inv->antiflow_enable)
            {
                inv->prop_changed = true;
            }

            // ����sub1g��ַ
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            // ����״̬�仯ʱ������Ա��
            if (inv->online_state != 2)
            {
                DEBUG_PRINTF("inv_index = %d, online_state = %d, prop_changed = %d.\n", inv_index, inv->online_state, inv->prop_changed);
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;

            // ���ݹ���ģʽͬ������������
            switch (sys_param.power_work_mode)
            {
            case 1:                                 // ������ģʽ��Ĭ�ϣ�
                sys_param.anti_backflow_switch = 1; // ����������
                break;

            case 2:                                 // ǿ�Ʒ���ģʽ
                sys_param.anti_backflow_switch = 1; // ����������
                break;

            case 3:                                 // ���ɷ���ģʽ
                sys_param.anti_backflow_switch = 0; // �رշ�����
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

    case CMD_INV_REPORT_NORMAL_2: // 0x52 - ΢���ճ��ϱ�2(10s����)
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // ��ʼ��ͳ��
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

            // 0x52���ռ���
            inv->rx_0x52_count++;
            //  ���������ֶ�

            // Byte 0: ���շ���ʱ��(0.1h��λ) - ����10��Сʱ
            inv->today_power_time = data[0] / 10.0f;

            // Byte 1-4: ���շ�����(float, kWh)
            sub1g_bytes_to_float(&data[1], &inv->today_energy);

            // Byte 5-8: �ۼƷ�����(float, kWh)
            sub1g_bytes_to_float(&data[5], &inv->lifetime_energy);

            // Byte 9-10: �����¶�(��10��������תfloat)
            inv->ambient_temperature = ((int16_t)((data[9] << 8) | data[10])) / 10.0f;

            // Byte 11-12: ����Ƶ��(��10��������תfloat)
            inv->grid_frequency = ((int16_t)((data[11] << 8) | data[12])) / 10.0f;

            // Byte 13-14: ������ѹ(��10��������תfloat)
            inv->grid_voltage = ((int16_t)((data[13] << 8) | data[14])) / 10.0f;

            // printf("0x52: inv_index = %d, today_power_time = %f, today_energy = %f, lifetime_energy = %f, temperature = %f, fre = %f, vol = %f.\n",
            //        inv_index,
            //        sys_param.paired_inv_info[inv_index].today_power_time,
            //        sys_param.paired_inv_info[inv_index].today_energy,
            //        sys_param.paired_inv_info[inv_index].lifetime_energy,
            //        sys_param.paired_inv_info[inv_index].ambient_temperature,
            //        sys_param.paired_inv_info[inv_index].grid_frequency,
            //        sys_param.paired_inv_info[inv_index].grid_voltage);

            // ����sub1g��ַ������״̬
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

    case CMD_INV_REPORT_ALL: // 0x51 - ΢���ϱ�ȫ������
    {
        if (sys_param.inv_0x51_report.valid != 1)
        {
            // ����sub1g_addr�����Ѱ��豸����
            inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

            if (inv_index < INV_DEVICE_MAX_NUM)
            {
                // ��¼0x51�ϱ�����
                sys_param.inv_0x51_report.inv_index = inv_index;
                sys_param.inv_0x51_report.data_len = sub1g_rx_frame.data_length - 1; // �ܳ���-�����볤��

                // ���ȳ���������ʱ�ض�
                if (sys_param.inv_0x51_report.data_len > INV_REPORT_51_SIZE)
                {
                    sys_param.inv_0x51_report.data_len = INV_REPORT_51_SIZE;
                }

                // �������ݲ����ӽ�β��
                memcpy(sys_param.inv_0x51_report.data, sub1g_rx_frame.data_content, sys_param.inv_0x51_report.data_len);
                sys_param.inv_0x51_report.data[sys_param.inv_0x51_report.data_len] = '\0';
                sys_param.inv_0x51_report.valid = 1;

                // ��������״̬
                sys_param.paired_inv_info[inv_index].sub1g_addr = sub1g_rx_frame.sub1g_addr;
                sys_param.paired_inv_info[inv_index].online_state = 2;
                sys_param.paired_inv_info[inv_index].offline_updata_ms = 0;

                DEBUG_PRINTF("Received 0x51 from inv[%d], addr=0x%06X, len=%d\r\n", inv_index, sub1g_rx_frame.sub1g_addr, sys_param.inv_0x51_report.data_len);
            }
            else
            {
                // δ�ҵ����豸�����ͽ��
                sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
            }
        }
        break;
    }

    case CMD_INV_REPORT_NORMAL_3: // 0x54 - ΢���ճ��ϱ�3(7s����)
    {
        // ���¼�¼�ŵ�
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // �����ֶ�
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

            // ��¼�ϴη��翪��״̬���ڱ�����
            uint8_t power_enable_last = inv->power_enable;

            // Byte 0: ���翪��״̬
            inv->power_enable = data[0];

            // Byte 1: ΢��RSSI
            inv->inv_rssi = (int8_t)data[1];

            // Byte 2: PV·��
            uint8_t pv_num = data[2];
            inv->pv_num = pv_num;

            // ��ȡPV��·״̬ (1B*n)
            uint16_t offset = 3;
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].state = data[offset + i];
            }
            offset += pv_num;

            // ��ȡPV��·��ѹ (2B*n)
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].voltage = ((int16_t)((data[offset] << 8) | data[offset + 1])) / 10.0f;
                offset += 2;
            }

            // ��ȡPV��·���� (2B*n)
            for (uint8_t i = 0; i < inv->pv_num; i++)
            {
                inv->pv[i].current = ((int16_t)((data[offset] << 8) | data[offset + 1])) / 10.0f;
                offset += 2;
            }

            // �ŵ�����������
            if (power_enable_last != inv->power_enable)
            {
                inv->prop_changed = true;
            }

            // ����sub1g��ַ������״̬
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

    case CMD_INV_REPORT_SET_1: // 0x55 - ΢�������ϱ�1: ����� + �������� + sub1g�汾
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            uint8_t str_packet_len = 0;
            uint8_t safe_copy_len = 0;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // ��ʼ��ͳ��
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

            // ��¼��ֵ���ڱ�����
            uint8_t old_connection_point = inv->connection_point;
            uint16_t old_power_limit = inv->power_limit;
            char old_sub1g_version[VERSION_STRING_MAX_LEN + 1];
            strncpy(old_sub1g_version, inv->sub1g_version, VERSION_STRING_MAX_LEN);
            old_sub1g_version[VERSION_STRING_MAX_LEN] = '\0';

            // Byte 0: �����
            inv->connection_point = data[index++];

            // Byte 1-2: ��������(W)
            inv->power_limit = ((uint16_t)data[index] << 8) | data[index + 1];
            index += 2;

            // sub1g�汾 (����ǰ׺ + �ַ���)
            str_packet_len = data[index++];
            safe_copy_len = (str_packet_len > VERSION_STRING_MAX_LEN) ? VERSION_STRING_MAX_LEN : str_packet_len;
            memcpy(inv->sub1g_version, &data[index], safe_copy_len);
            inv->sub1g_version[safe_copy_len] = '\0';
            index += str_packet_len;

            // DEBUG_PRINTF("0X55: inv_index = %d, connection_point = %d, power_limit = %d, sub1g_version = %s.\n", inv_index, inv->connection_point, inv->power_limit, inv->sub1g_version);

            // �����
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

            // ����sub1g��ַ������״̬������״̬�仯ʱ������Ա��
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

    case CMD_INV_REPORT_SET_2: // 0x56 - ΢�������ϱ�2: MCU�汾 + ����
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            uint8_t str_packet_len = 0;
            uint8_t safe_copy_len = 0;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // ������
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

            // ������Ա��
            char old_sw_version[VERSION_STRING_MAX_LEN + 1];
            strncpy(old_sw_version, inv->sw_version, VERSION_STRING_MAX_LEN);
            old_sw_version[VERSION_STRING_MAX_LEN] = '\0';

            // MCU�汾 (����ǰ׺ + �ַ���)
            str_packet_len = data[index++];
            safe_copy_len = (str_packet_len > VERSION_STRING_MAX_LEN) ? VERSION_STRING_MAX_LEN : str_packet_len;
            memcpy(inv->sw_version, &data[index], safe_copy_len);
            inv->sw_version[safe_copy_len] = '\0';
            index += str_packet_len;

            // ֡ͷ
            uint8_t inv_phase = data[index++];
            uint8_t local_phase = inv->phase;

            // DEBUG_PRINTF("0X56: inv_index = %d, sw_version = %s, phase = %d.\n", inv_index, inv->sw_version, inv_phase);

            // #ifdef FFT_DEBUG_PRINT
            //             printf("Device: 0x%06X, Report phase=%d, local phase=%d\r\n",
            //                    sub1g_rx_frame.sub1g_addr, inv_phase, local_phase);
            // #endif

            // ����������Чʱ����У��
            if (local_phase > 0 && local_phase <= 3)
            {
                if (inv_phase != local_phase)
                {
                    // ����һ�£��·���ȷ����
                    sub1g_send_set_inv_phase(sub1g_rx_frame.sub1g_addr, local_phase);

                    DEBUG_PRINTF("Inv[%d] phase mismatch! Correct: %d -> %d\r\n", inv_index, inv_phase, local_phase);
                }
            }
            else if ((inv_phase == 0 || local_phase != inv_phase) && !sys_param.fft_identify.is_ffting)
            {
                // ����FFTʶ��
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

            // �汾������
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

            // �״��յ�����ʱ��ʼ��ͳ��
            inv->sub1g_addr = sub1g_rx_frame.sub1g_addr;
            // ����״̬�仯ʱ������Ա��
            if (inv->online_state != 2)
            {
                inv->prop_changed = true;
            }
            inv->online_state = 2;
            inv->offline_updata_ms = 0;
        }
        else
        {
            // δ�ҵ����豸�����ͽ��
            sub1g_send_unbind(sub1g_rx_frame.sub1g_addr);
        }
        break;
    }

    case CMD_INV_REPORT_SET_3: // 0x57 - ΢�������ϱ�3: �ͺ� + �ŵ�����
    {
        inv_index = find_inv_index_by_sub1g_addr(sub1g_rx_frame.sub1g_addr);

        if (inv_index < INV_DEVICE_MAX_NUM)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            uint8_t str_packet_len = 0;
            uint8_t safe_copy_len = 0;
            inv_device_t *inv = &sys_param.paired_inv_info[inv_index];

            // ������
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

            // ����ڴ��λ�������
            char old_product_model[PRODUCT_MODEL_MAX_LEN + 1];
            strncpy(old_product_model, inv->product_model, PRODUCT_MODEL_MAX_LEN);
            old_product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

            // �ͺ��ַ��� (����ǰ׺ + �ַ���)
            str_packet_len = data[index++];
            safe_copy_len = (str_packet_len > PRODUCT_MODEL_MAX_LEN) ? PRODUCT_MODEL_MAX_LEN : str_packet_len;
            memcpy(inv->product_model, &data[index], safe_copy_len);
            inv->product_model[safe_copy_len] = '\0';
            index += str_packet_len;

            // �ŵ����������Ⲣ���������ϱ�
            if (inv->channel_index != data[index])
            {
                inv->settings_changed = true;
            }

            // ���¼�¼�ŵ�
            inv->channel_index = data[index++];
            // DEBUG_PRINTF("0X57: inv_index = %d, product_model = %s, channel_index = %d.\n", inv_index, inv->product_model, inv->channel_index);

            // �ͺű�����
            if (strncmp(old_product_model, inv->product_model, PRODUCT_MODEL_MAX_LEN) != 0)
            {
                inv->settings_changed = true;
                DEBUG_PRINTF("[INV] [0x%06X] updata: Sub1g=%s, SW=%s, Model=%s\r\n",
                             inv->sub1g_addr,
                             inv->sub1g_version,
                             inv->sw_version,
                             inv->product_model);
            }

            // ����sub1g��ַ������״̬
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

    case CMD_CT_SUB1G_VERSION: // �յ�CTģ���sub1g�汾�ظ�
    {
        uint8_t *data = sub1g_rx_frame.data_content;

        // ����3�ֽڵ�sub1g��ַ
        sys_param.sub1g.ct_sub1g_addr = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

        // ����ɰ汾���ڱȽ�
        char old_version[VERSION_STRING_MAX_LEN + 1];
        strncpy(old_version, sys_param.sub1g.sw_version, VERSION_STRING_MAX_LEN);
        old_version[VERSION_STRING_MAX_LEN] = '\0';

        // �����汾�ַ���
        uint8_t str_version_len = data_content_length - 3; // ȥ��ǰ3�ֽڵ�ַ
        if (str_version_len > VERSION_STRING_MAX_LEN)
        {
            str_version_len = VERSION_STRING_MAX_LEN;
        }
        memcpy(sys_param.sub1g.sw_version, &data[3], str_version_len);
        sys_param.sub1g.sw_version[str_version_len] = '\0';

        DEBUG_PRINTF("[SUB1G] CT Sub1G: Src_addr=0x%06lX, Version=%s\r\n", sys_param.sub1g.ct_sub1g_addr, sys_param.sub1g.sw_version);

        // �汾�����仯ʱ�����汾�����ϱ�
        if (strcmp(old_version, sys_param.sub1g.sw_version) != 0 && old_version[0] != '\0')
        {
            DEBUG_PRINTF("[SUB1G] CT Sub1G changed: %s -> %s\r\n", old_version, sys_param.sub1g.sw_version);
            sys_param.slave_version.slave_version_reported = false;
     
        }

        // ֪ͨOTAģ��CT Sub1G�汾�ϱ�
        ota_handle_ct_sub1g_version_report(sys_param.sub1g.sw_version);

        break;
    }

    case CMD_CT_SUB1G_RSSI: // ???CT??sub1g RSSI???
    {
        uint8_t *data = sub1g_rx_frame.data_content;

        // RSSI
        sys_param.sub1g.rssi = (int8_t)data[0];

        // CT��ǰ�ŵ�����
        if (data_content_length >= 2)
        {
            sys_param.sub1g.channel_index = data[1];
        }
        // DEBUG_PRINTF("[SubG]CT report RSSI=%d, Channel=%d\r\n", sys_param.sub1g.rssi, sys_param.sub1g.channel_index);
        break;
    }

    case CMD_CT_SUB1G_RESTART: // SUB1G????(OTA???)
    {
        // ����汾�ַ���
        sys_param.sub1g.sw_version[0] = '\0';
        sys_param.sub1g.version_timer_ms = 2000;

        // ������Ϣ
        sub1g_send_record_channel();

        // �汾�ѱ仯�������汾�����ϱ�
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

    case SUB1G_OTA_CMD_VERSION_REPORT: // 0x77 - ???????��?
    {
        if (sub1g_rx_frame.data_length >= 2)
        {
            uint8_t *data = sub1g_rx_frame.data_content;
            uint8_t index = 0;
            char sub1g_version[VERSION_STRING_MAX_LEN + 1] = {0};
            char mcu_version[VERSION_STRING_MAX_LEN + 1] = {0};

            // ����Sub1G�汾 (����ǰ׺ + �ַ���)
            uint8_t sub1g_ver_len = data[index++];
            if (sub1g_ver_len > 0 && sub1g_ver_len <= VERSION_STRING_MAX_LEN && (index + sub1g_ver_len) <= sub1g_rx_frame.data_length)
            {
                memcpy(sub1g_version, &data[index], sub1g_ver_len);
                sub1g_version[sub1g_ver_len] = '\0';
                index += sub1g_ver_len;
            }

            // ����MCU�汾 (����ǰ׺ + �ַ���)
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

            // ֪ͨOTA�����������汾�ϱ�
            ota_handle_sub1g_version_report(sub1g_rx_frame.sub1g_addr, sub1g_version, mcu_version);
        }
        break;
    }

    default:
        // δ֪���������
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : uint8_t inv_request_pair_list_add(...)
 Input       :
 Output      : INV��������б�������ʧ�ܷ���UNPAIRED_DEVICE_MAX_NUM
 Description : ���豸���ӵ�INV��������б�
---------------------------------------------------------------------------*/
uint8_t inv_request_pair_list_add(uint32_t sub1g_addr, const char *device_sn, uint8_t product_model)
{
    int8_t existing_index;
    uint8_t free_slot = UNPAIRED_DEVICE_MAX_NUM;

    // ����Ƿ��Ѵ���
    existing_index = inv_request_pair_list_find_by_addr(sub1g_addr);
    if (existing_index >= 0)
    {
        // �Ѵ�����ˢ��10�볬ʱ��ʱ��������
        sys_param.inv_request_pair_list[existing_index].unpaired_updata_ms = 0;
        return existing_index;
    }

    // ���ҿ��в�λ
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
        return UNPAIRED_DEVICE_MAX_NUM; // �б�����
    }

    // �����豸��Ϣ
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
 Input       : sub1g_addr - SUB1G��ַ
 Output      : ??
 Description : ��INV��������б��Ƴ��豸
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
    Output      : �ҵ�����������δ�ҵ�����-1
    Description : ����ַ����INV��������豸
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

// ================== �û�����б��Ͱ�/���������� ==================

/*---------------------------------------------------------------------------
 Name        : bool user_pair_list_add(const char *device_sn)
 Input       : device_sn - �豸SN
 Output      : true=�ɹ�, false=ʧ��
 Description : ���豸SN�����û�����б���ͬʱԤ����SIID��paired_inv_info
               �Ա��豸����ʱ������ɨ��ֱ����ɰ󶨣�sub1g_addr������ȫ��
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

    // ���ҿ��в�λ
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
        return false; // �б�����
    }

    // д���豸SN�������Ч
    strncpy(sys_param.user_pair_list[free_slot].device_sn, device_sn, SN_LENGTH);
    sys_param.user_pair_list[free_slot].device_sn[SN_LENGTH] = '\0';
    sys_param.user_pair_list[free_slot].is_valid = true;

    // printf("WIFI ADD user_pair: %s, id=%d, is_valid =%d\r\n", device_sn, free_slot, sys_param.user_pair_list[free_slot].is_valid);

    // ͬ��д��EEPROM
    if (eeprom_save_user_pair_device_by_sn(device_sn) != 0)
    {
        // // EEPROMд��ʧ����ع��ڴ�
        // printf("Warning: Failed to save user pair to EEPROM: %s\r\n", device_sn);
        sys_param.user_pair_list[free_slot].is_valid = false;
        sys_param.user_pair_list[free_slot].device_sn[0] = '\0';
        return false;
    }

    // Ԥ����SIID��paired_inv_info��ʹ�豸�ϵ�����Զ��ָ���
    uint8_t siid = eeprom_add_device_by_sn_only(device_sn);
    if (siid == 0)
    {
        // SIID����ʧ�ܲ���ֹ���ӣ���Ԥ���䣬�Ǳ��룩
        // �豸���ߺ�������߰�����
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
 Input       : device_sn - �豸SN
 Output      : �ҵ�����������δ�ҵ�����-1
 Description : ��SN�����û�����б�
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
            // ��ʵ��SN���ȱȽϣ�����ض�ƥ�����
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
 Description : ��SN���û�����б�ɾ���������EEPROM��¼
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

    // ���EEPROM��¼
    if (eeprom_clear_user_pair_list_device_by_sn(device_sn) != 0)
    {
        // printf("Warning: Failed to clear user pair from EEPROM: %s\r\n", device_sn);
    }

    // ��ջ�����
    sys_param.user_pair_list[index].is_valid = false;
    sys_param.user_pair_list[index].device_sn[0] = '\0';

    // printf("User pair removed: SN=%s\r\n", device_sn);
}

/*---------------------------------------------------------------------------
Name        : bool user_bind_device(uint32_t sub1g_addr)
Input       : sub1g_addr - ????SUB1G???
Output      : true=??????false=?????
Description : ??????APP?????????????
            1. ��δ����б������豸��Ϣ
            2. ����eeprom_add_device()д��EEPROM (�Զ�����SIID������sys_param)
            3. �������ȷ�ϴ����� (�ڼ��ظ��յ��㲥�Ի�Ӧ��)
            4. ��δ����б��Ƴ��豸
            5. ���Ͱ�����豸
---------------------------------------------------------------------------*/
bool user_bind_device(uint32_t sub1g_addr)
{
    int8_t unpaired_index;
    uint8_t siid;
    uint8_t inv_index;

    // ����Ƿ��Ѱ�
    if (find_inv_index_by_sub1g_addr(sub1g_addr) < INV_DEVICE_MAX_NUM)
    {
        return false; // �Ѱ�
    }

    // ��δ����б������豸��Ϣ
    unpaired_index = inv_request_pair_list_find_by_addr(sub1g_addr);
    if (unpaired_index < 0)
    {
        return false; // δ�ҵ��豸
    }

    unpaired_device_t *device = &sys_param.inv_request_pair_list[unpaired_index];
    device->paired_unvalid_ms = 2000; // 2�����ظ��յ��㲥�Ի�Ӧ������

    // ����eeprom_add_deviceд���豸
    siid = eeprom_add_device(device->device_sn, sub1g_addr, device->product_model);
    if (siid == 0)
    {
        return false; // д��ʧ�� (EEPROM��������������)
    }

    inv_index = find_inv_index_by_sub1g_addr(sub1g_addr);

    // ��������״̬
    sys_param.paired_inv_info[inv_index].online_state = 2; // �豸����

    // ���Ͱ�����豸
    sub1g_send_bind(sub1g_addr);

    // �����û�����б��е��豸SN����ɾ����
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

    // ��ȡ�豸SN���ں��������û�����б�
    char device_sn[SN_LENGTH + 1];
    strncpy(device_sn, sys_param.paired_inv_info[inv_index].device_sn, SN_LENGTH);
    device_sn[SN_LENGTH] = '\0';

    // ��EEPROMɾ���豸��¼
    if (eeprom_remove_device_by_sub1g_addr(sub1g_addr) != 0)
    {
        return false;
    }

    // ����ڴ��е��豸��¼
    memset(&sys_param.paired_inv_info[inv_index], 0, sizeof(inv_device_t));
    sys_param.paired_inv_info[inv_index].is_valid = false;

    // ͬʱ����û�����б��е�SN
    eeprom_clear_user_pair_list_device_by_sn(device_sn);

    // ���ͽ������豸
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

    // ��paired_inv_info�в���ƥ���豸
    for (uint8_t i = 0; i < 8; i++)
    {
        if (strcmp(sys_param.paired_inv_info[i].device_sn, device_sn) == 0 && sys_param.paired_inv_info[i].is_valid)
        {
            // Ԥ����״̬��EEPROM�м�¼��sub1g_addrΪ0��
            if (sys_param.paired_inv_info[i].sub1g_addr == 0)
            {
                // ͨ��SIID��EEPROMɾ����¼
                uint8_t siid = sys_param.paired_inv_info[i].siid;

                // ���EEPROM�е��豸��¼
                eeprom_remove_device_by_siid(siid);

                // ����ڴ�
                memset(&sys_param.paired_inv_info[i], 0, sizeof(inv_device_t));
                sys_param.paired_inv_info[i].is_valid = false;

                // ����û�����б��е�SN
                eeprom_clear_user_pair_list_device_by_sn(device_sn);
                user_pair_list_remove_by_sn(device_sn);

                return true;
            }
            else
            {
                // �����Ѱ��豸��addr���
                return user_unbind_device_by_sub1g_addr(sys_param.paired_inv_info[i].sub1g_addr);
            }
        }
    }

    // �豸����paired_inv_info�У���������user_pair_list
    for (uint8_t i = 0; i < 8; i++)
    {
        if (strcmp(sys_param.user_pair_list[i].device_sn, device_sn) == 0)
        {
            // ����û�����б��е�SN
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

    // ���ֽڷ���
    memset(paired_device_list_buffer, 0, DEVICE_LIST_BUFFER_SIZE);

    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid &&
            sys_param.paired_inv_info[i].siid >= SIID_MIN &&
            sys_param.paired_inv_info[i].siid <= SIID_MAX)
        {
            // ��黺�����Ƿ����㹻�ռ�
            // ÿ����ʽ: siid(1) + "," + SN(15) + ";" = ���18�ֽ�
            if (offset + 18 >= DEVICE_LIST_BUFFER_SIZE)
            {
                break; // �������ռ䲻�㣬ֹͣ����
            }

            // �ǵ�һ����¼ǰ�ӷָ���
            if (!first_device)
            {
                paired_device_list_buffer[offset++] = ';';
            }
            first_device = false;

            // ��ʽ: "SIID,SN;SIID,SN;SIID,SN"
            offset += sprintf(paired_device_list_buffer + offset, "%d,%s",
                              sys_param.paired_inv_info[i].siid,
                              sys_param.paired_inv_info[i].device_sn);
            device_count++;
        }
    }
    // ׷�ӽ�β��ȷ���ַ����Ϸ�
    if (device_count == 0)
    {
        paired_device_list_buffer[0] = '\0';
    }

    return paired_device_list_buffer;
}

/*---------------------------------------------------------------------------
 Name        : const char *get_inv_request_pair_list_string(void)
 Output      : INV???????????��???????
 Description : ???INV???????????��??????????????ADDR,MODEL;ADDR,MODEL;...??
---------------------------------------------------------------------------*/
const char *get_inv_request_pair_list_string(void)
{
    uint16_t offset = 0;
    bool first_device = true;
    uint8_t device_count = 0;

    // ��ջ�����
    memset(unpaired_device_list_buffer, 0, DEVICE_LIST_BUFFER_SIZE);

    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.inv_request_pair_list[i].is_valid)
        {
            // ��黺�����Ƿ����㹻�ռ�
            if (offset + 19 >= DEVICE_LIST_BUFFER_SIZE)
            {
                break; // �������ռ䲻�㣬ֹͣ����
            }

            // �ǵ�һ����¼ǰ�ӷָ���
            if (!first_device)
            {
                unpaired_device_list_buffer[offset++] = ';';
            }
            first_device = false;

            // ��ȡ��Ʒ�ͺ��ַ���
            const char *model_str = product_model_code_to_string(sys_param.inv_request_pair_list[i].product_model);

            // ��ʽ: "XXXXXX,�ͺ�"
            offset += sprintf(unpaired_device_list_buffer + offset, "%s,%s", sys_param.inv_request_pair_list[i].device_sn, model_str);
            device_count++;
        }
    }

    // ���豸ʱȷ���ַ���Ϊ��
    if (device_count == 0)
    {
        unpaired_device_list_buffer[0] = '\0';
    }

    return unpaired_device_list_buffer;
}

/*---------------------------------------------------------------------------
 Name        : const char *get_user_pair_list_string(void)
 Output      : �û�����б��ַ���
 Description : ���û�����б����л�Ϊ�ַ�������ʽ"SN1,SN2,SN3"
---------------------------------------------------------------------------*/
const char *get_user_pair_list_string(void)
{
    uint16_t offset = 0;
    bool first_device = true;
    uint8_t device_count = 0;

    // ��ջ�����
    memset(user_pair_list_buffer, 0, DEVICE_LIST_BUFFER_SIZE);

    for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
    {
        if (sys_param.user_pair_list[i].is_valid)
        {
            // ��黺�����Ƿ����㹻�ռ�
            if (offset + SN_LENGTH + 2 >= DEVICE_LIST_BUFFER_SIZE)
            {
                break;
            }

            // �ǵ�һ����¼ǰ�Ӷ��ŷָ�
            if (!first_device)
            {
                user_pair_list_buffer[offset++] = ',';
            }
            first_device = false;

            // д��SN
            offset += sprintf(user_pair_list_buffer + offset, "%s", sys_param.user_pair_list[i].device_sn);
            device_count++;
        }
    }

    // ���豸ʱȷ���ַ���Ϊ��
    if (device_count == 0)
    {
        user_pair_list_buffer[0] = '\0';
    }

    return user_pair_list_buffer;
}

// ================= ����Ϊ CT ����������� sub1g_ct �ĺ��� =================

/*---------------------------------------------------------------------------
 Name        : bool uart1_tx_queue_push(const uint8_t *data, uint16_t len)
 Input       : data - �����͵�����ָ��
               len  - ���ݳ���
 Output      : true=��ӳɹ�, false=�����������������
 Description : ����Ϣ���뷢�Ͷ���
---------------------------------------------------------------------------*/
bool uart1_tx_queue_push(const uint8_t *data, uint16_t len)
{
    // ����У��
    if (data == NULL || len == 0 || len > UART1_TX_MSG_MAX_LEN)
    {
        DEBUG_PRINTF("uart1_tx_queue_push: error\n");
        return false;
    }

    // �������Ƿ�����
    if (g_uart1_tx_queue.count >= UART1_TX_QUEUE_SIZE)
    {
        // ���������������˴η���
        // sys_param.sub1g.state = 5;  // 5 = �������������ã�
        DEBUG_PRINTF("uart1_tx_queue_push: queue full\n");
        return false;
    }

    // ������д�뵱ǰ���в�λ
    uart1_tx_msg_t *slot = &g_uart1_tx_queue.buffer[g_uart1_tx_queue.write_index];
    memcpy(slot->data, data, len);
    slot->length = len;

    // дָ��ǰ�ƣ����Σ�
    g_uart1_tx_queue.write_index = (g_uart1_tx_queue.write_index + 1) % UART1_TX_QUEUE_SIZE;

    // ��Ϣ������һ
    g_uart1_tx_queue.count++;

    return true;
}

/*---------------------------------------------------------------------------
 Name        : static bool uart1_tx_queue_pop(uart1_tx_msg_t *msg)
 Input       : msg - ������Ϣ��Ŀ�껺����
 Output      : true=���ӳɹ�, false=���п�
 Description : �Ӷ���ȡ��һ����Ϣ��ʹ��memcpy���Ƶ������߻�����
               ����ֱ�Ӳ���bufferָ�룬����ָ����ǰ�ƽ�����CPU�������
---------------------------------------------------------------------------*/
static bool uart1_tx_queue_pop(uart1_tx_msg_t *msg)
{
    // ����Ϊ��ʱֱ�ӷ���
    if (g_uart1_tx_queue.count == 0)
    {
        return false;
    }

    // ��ӷ���
    memcpy(msg, &g_uart1_tx_queue.buffer[g_uart1_tx_queue.read_index], sizeof(uart1_tx_msg_t));

    // ��ָ��ǰ�ƣ����Σ�
    g_uart1_tx_queue.read_index = (g_uart1_tx_queue.read_index + 1) % UART1_TX_QUEUE_SIZE;

    // ��Ϣ������һ
    g_uart1_tx_queue.count--;

    return true;
}

/*---------------------------------------------------------------------------
 Name        : void uart1_tx_queue_process(void)
 Input       : ??
 Output      : ??
 Description : ��ѭ�����ã��Ӷ���ȡ����Ϣ��ͨ��UART1���ֽڷ���
---------------------------------------------------------------------------*/
void uart1_tx_queue_process(void)
{
    uart1_tx_msg_t msg;

    // ���ڷ��� �� ����Ϊ��ʱֱ�ӷ���
    if (g_uart1_tx_queue.is_sending || g_uart1_tx_queue.count == 0)
    {
        return;
    }

    // �Ӷ���ȡ��һ����Ϣ��ʧ���򷵻�
    if (!uart1_tx_queue_pop(&msg))
    {
        return;
    }

    // �ӷ�����
    g_uart1_tx_queue.is_sending = true;

    // printf("uart1_tx: %d bytes: ", msg.length);
    // for (uint16_t i = 0; i < msg.length; i++)
    // {
    //     printf("%02X ", msg.data[i]);
    // }
    // printf("\r\n");

    // ���ֽڷ���
    for (uint16_t i = 0; i < msg.length; i++)
    {
        USART_WriteData(CM_USART1, msg.data[i]);

        // �ȴ����ͼĴ�����
        while (RESET == USART_GetStatus(CM_USART1, USART_FLAG_TX_EMPTY))
        {
            // ��ѯ�ȴ��������
        }
    }

    // ������ɣ��ͷŷ�����
    g_uart1_tx_queue.is_sending = false;
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_bind(uint32_t target_addr)
 Input       : target_addr - Ŀ���豸SUB1G��ַ
 Output      : ??
 Description : ���Ͱ�����0x02��֡��ʽ��֡ͷ + Ŀ���ַ + ����(1) + ������
---------------------------------------------------------------------------*/
void sub1g_send_bind(uint32_t target_addr)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    // ֡ͷ
    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;

    // Ŀ��SUB1G��ַ
    temp_buffer[2] = (target_addr >> 16) & 0xFF;
    temp_buffer[3] = (target_addr >> 8) & 0xFF;
    temp_buffer[4] = target_addr & 0xFF;

    // �����ֶ�=1����������1B��
    temp_buffer[5] = 1;

    // ������
    temp_buffer[6] = CMD_CT_BIND;

    // ��ӷ���
    if (!uart1_tx_queue_push(temp_buffer, 7))
    {
        // ������ʱ����
        // sys_param.sub1g.state = 5;  // 5 = �������������ã�
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
    temp_buffer[2] = 0x00; // �㲥��ַ000000
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;
    temp_buffer[5] = sn_len + 1; // ���� = SN�ֽ��� + ������ռ1�ֽ�

    temp_buffer[6] = CMD_CT_BROADCAST_UNBIND_SN; // 0x14

    // ����SN
    memcpy(&temp_buffer[7], device_sn, sn_len);

    uart1_tx_queue_push(temp_buffer, 7 + sn_len);
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_send_broadcast_single_phase_power(float power, uint8_t inv_count, uint32_t report_addr)
 Input       : power - ���ಢ������(W)
               inv_count - ��ǰ��������΢������
               report_addr - ָ��΢����ӵ�ַ���㲥��˭��
 Output      : ??
 Description : ֡��ʽ��֡ͷ + �㲥��ַ + ����(7) + ������ + ����(2B) + �豸��(1B) + �ӵ�ַ(3B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_single_phase_power(float power, uint8_t inv_count, uint32_t report_addr)
{
    if (g_ota_manager.disable_broadcast) // OTA�ڼ��ֹ�㲥
    {
        return;
    }

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];
    int16_t power_int = (int16_t)power;

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x00; // �㲥��ַ
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
 Input       : power_ct1/ct2/ct3 - ���๦��(W)������������
               report_addr - Ŀ���ӵ�ַ
 Output      : ??
 Description : ֡��ʽ��֡ͷ + �㲥��ַ + ����(10) + ������ + A�๦��(2B) + B�๦��(2B) + C�๦��(2B) + �ӵ�ַ(3B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_three_phase_power(int16_t power_ct1, int16_t power_ct2, int16_t power_ct3, uint32_t report_addr)
{
    if (g_ota_manager.disable_broadcast) // OTA�ڼ��ֹ�㲥
    {
        return;
    }
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x00; // �㲥��ַ
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
 Input       : date - �����ַ��� "YYYY-MM-DD" (10�ֽ�)
 Output      : ??
 Description : ֡��ʽ��֡ͷ + �㲥��ַ + ����(5) + ������(1B) + ��(2B) + ��(1B) + ��(1B)
---------------------------------------------------------------------------*/
void sub1g_send_broadcast_date(const char *date)
{
    if (g_ota_manager.disable_broadcast) // OTA�ڼ��ֹ�㲥
    {
        return;
    }

    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    // ����У�飬date����Ϊ��������10�ֽ�
    if (date == NULL || strlen(date) < 10)
    {
        return;
    }

    // ����������
    char year_str[5], month_str[3], day_str[3];
    uint16_t year;
    uint8_t month, day;

    // ��"YYYY-MM-DD"��ʽ�ֱ��ȡ�������ַ���
    strncpy(year_str, date, 4);
    year_str[4] = '\0';
    strncpy(month_str, date + 5, 2);
    month_str[2] = '\0';
    strncpy(day_str, date + 8, 2);
    day_str[2] = '\0';

    // �ַ���ת����
    year = (uint16_t)atoi(year_str);
    month = (uint8_t)atoi(month_str);
    day = (uint8_t)atoi(day_str);
    // ֡ͷ
    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    // 0x0000�㲥��ַ
    temp_buffer[2] = 0x00;
    temp_buffer[3] = 0x00;
    temp_buffer[4] = 0x00;

    // ���ݳ���=������1B + ��2B + ��1B + ��1B = 5B
    temp_buffer[5] = 5;

    // ������
    temp_buffer[6] = CMD_CT_BROADCAST_DATE;

    // ���2�ֽڴ��
    temp_buffer[7] = (year >> 8) & 0xFF;
    temp_buffer[8] = year & 0xFF;

    // �·�1�ֽ�
    temp_buffer[9] = month;

    // ����1�ֽ�
    temp_buffer[10] = day;

    uart1_tx_queue_push(temp_buffer, 11); // ֡ͷ2 + ��ַ3 + ����1 + ������1 + ����4 = 11
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

    temp_buffer[2] = 0x00; // �㲥��ַ
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
 Input       : target_addr - Ŀ���豸SUB1G��ַ
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
    if (g_ota_manager.disable_broadcast) // OTA�ڼ��ֹ�㲥
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
               time - ����ʶ�����ʱ�� (��λ: s)
               power - ʶ������ֵ(W)
               power_interval - ʶ���ʼ�� (��λ: 10ms��n�����ڿ���n�����ڹ�)
 Output      : ??
 Description : ֡��ʽ��֡ͷ + Ŀ���ַ + ����(5) + ������(0x23) + ʶ��ʱ��(1B) + ����(2B) + �����10ms(1B)
---------------------------------------------------------------------------*/
void sub1g_send_enable_phase_identify(uint32_t target_addr, uint8_t time, uint16_t power, uint8_t power_interval)
{
    // ����ʶ����������У��ܾ�������
    if (sys_param.fft_identify.is_ffting == 1)
    {
#ifdef FFT_DEBUG_PRINT
        printf("Busy: Device 0x%06X is identifying\r\n", sys_param.fft_identify.sub1g_addr);
#endif
        return;
    }

    // // �����豸����
    // uint8_t inv_index = INV_DEVICE_MAX_NUM;
    // inv_index = find_inv_index_by_sub1g_addr(target_addr);
    // if (inv_index >= INV_DEVICE_MAX_NUM)
    // {
    //     // printf("Device not found: 0x%06X\r\n", target_addr);
    //     return;
    // }

    // // ����豸����״̬
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

    // ���� fft_identify ״̬
    sys_param.fft_identify.is_ffting = 1; // ���ʶ����
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
 Description : ??????CT??SUB1G?��????????0x41?? ??????? + ?????(0x111111) + ????(1) + ??????
---------------------------------------------------------------------------*/
void sub1g_send_get_version(void)
{
    uint8_t temp_buffer[UART1_TX_MSG_MAX_LEN];

    temp_buffer[0] = 0xFA;
    temp_buffer[1] = 0xCE;
    temp_buffer[2] = 0x11; // �㲥��ַ0x111111(CTר�õ�SUB1G��ַ)
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
    temp_buffer[2] = 0x11; // �㲥��ַ0x111111(CTר�õ�SUB1G��ַ)
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
    temp_buffer[2] = 0x11; // �㲥��ַ0x111111(CTר�õ�SUB1G��ַ)
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
    temp_buffer[2] = 0x11; // �㲥��ַ0x111111(CTר�õ�SUB1G��ַ)
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
 Description : ???????��????????????0x25?? ??????? + ????? + ????(3) + ?????? + ????????(2B?????)
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

//  ================= ����Ϊ���ߺ���  =================

/*---------------------------------------------------------------------------
 Name        : uint8_t find_inv_index_by_sub1g_addr(uint32_t sub1g_addr)
 Input       :
 Output      : �ҵ���������0-7��δ�ҵ�����INV_DEVICE_MAX_NUM
 Description : ��SUB1G��ַ���Ѱ��豸�б��в�������
---------------------------------------------------------------------------*/
uint8_t find_inv_index_by_sub1g_addr(uint32_t sub1g_addr)
{
    // �����Ѱ��豸�б�
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // �ҵ�ƥ������Ч�ļ�¼
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr)
        {
            return i;
        }
    }

    // δ�ҵ���������Ч����
    return INV_DEVICE_MAX_NUM;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t find_free_inv_slot(void)
 Input       : ??
 Output      : ���в�λ����0-7���޿��в�λ����INV_DEVICE_MAX_NUM
 Description : ����δʹ�õ��豸��λ����
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
 Input       : bytes - 4�ֽڴ�˸�������
 Output      : value - ������ĸ���ֵ
 Description : ��4�ֽڴ������ת��Ϊfloat��IEEE754��
---------------------------------------------------------------------------*/
void sub1g_bytes_to_float(const uint8_t *bytes, float *value)
{
    union
    {
        float f;
        uint8_t b[4];
    } converter;

    // ���תС���ֽ���ӳ��
    converter.b[3] = bytes[0];
    converter.b[2] = bytes[1];
    converter.b[1] = bytes[2];
    converter.b[0] = bytes[3];

    *value = converter.f;
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_float_to_bytes(float value, uint8_t *bytes)
 Input       : value - ����ֵ
 Output      : bytes - 4�ֽڴ�����������
 Description : ��float�Դ���ֽ���д��4�ֽڻ�����
---------------------------------------------------------------------------*/
void sub1g_float_to_bytes(float value, uint8_t *bytes)
{
    union
    {
        float f;
        uint8_t b[4];
    } converter;

    converter.f = value;

    // С��ת����ֽ���ӳ��
    bytes[0] = converter.b[3];
    bytes[1] = converter.b[2];
    bytes[2] = converter.b[1];
    bytes[3] = converter.b[0];
}
