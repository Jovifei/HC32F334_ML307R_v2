//
// Included Files
//

#include "arm_math.h"
#include "board.h"
#include "main.h"
#include "mmi.h"
#include "grid.h"
#include "wifi_info.h"
#include "sub1g.h"
#include "debug.h"
#include "eeprom.h"
#include "ota_max.h"
#include "fft.h"
#include "uart_at.h"
#include <stdint.h>
#include "ml307r.h"

//
// Globals
//
sys_param_t sys_param;
wifi_info_t wifi_info;

float ua_voltage_buffer[TOTAL_SAMPLES];
float last_ua_voltage_buffer[TOTAL_SAMPLES];

// ����������
float current1_buffer[TOTAL_SAMPLES];
float current2_buffer[TOTAL_SAMPLES];
float current3_buffer[TOTAL_SAMPLES];

volatile uint8_t phase_identify_timer_100ms = 0; // ����ʶ��100ms��ʱ��־
uint8_t buffer_filled = 0;

// �ڲ���������
static uint16_t buffer_index = 0;

/*---------------------------------------------------------------------------
 Name        : uint16_t get_voltage_buffer_index(void)
 Input       : 无
 Output      : 当前缓冲区写指针
 Description : 由 grid.c 的 phase_matching_calculation 获取当前缓冲写指针，
               用于降低噪声计算时，与 last_ua 保持时间窗口一致。
---------------------------------------------------------------------------*/
uint16_t get_voltage_buffer_index(void)
{
    return buffer_index;
}

// ���������գ�ct_task ��ڴ������������ж��ƽ�? buffer_index ���� RMS/���ʴ��ڲ�һ�£�
static uint16_t s_calc_buf_snap = 0;

/*---------------------------------------------------------------------------
 Name        : uint16_t get_calc_buf_snap(void)
 Input       : 无
 Output      : ct_task 当前可获取的缓冲快照索引
 Description : 由 grid.c 的相位识别/匹配计算获取具有时间戳一致性的缓冲快照。
               确保压频计算稳定性，便于跨文件同步对齐。
---------------------------------------------------------------------------*/
static float ct_power_accum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t three_phase_broadcast_count = 0;

// �����������ڲ�������
static void fault_detection_task(void);
static void ct_power_calculate_task(void);
static void copy_ua_ring_to_last_ua_linear(uint16_t spc, uint16_t snap_idx);

static void boardcast_power_task(void);
static void broadcast_other_task(void);
static void param_update_1s_task(void);
static void sub1g_timer_task(void);
// static void state_machine_partial_reset(void);

/*---------------------------------------------------------------------------
 Name        : int main(void)
 Input       : 无
 Output      : 无
 Description : 主函数入口。初始化板卡、GPIO、中断等设备。
---------------------------------------------------------------------------*/
int main(void)
{
    //
    // SysConfig settings
    //
    board_init();

/*---------------------------------------------------------------------------
 Name        : int main(void)
 Input       : 无
 Output      : 无
 Description : 主函数入口。执行设备初始化，GPIO配置，中断优先级等初始化操作。
---------------------------------------------------------------------------*/
    system_param_init();

    // ��ʼ����ͨ��FFTģ��
    fft_3ch_init();

    // run_eeprom_tests();
    int ret = eeprom_init_and_load_devices();
    if (ret == 0)
    {
        print_device_list(); // ��ʾ�����豸

        // �ϵ�ʱ�����΢����ʶ����λ������?0x22��֪΢��������
        for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
        {
            if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].phase > 0)
            {
                sub1g_send_set_inv_phase(sys_param.paired_inv_info[i].sub1g_addr, sys_param.paired_inv_info[i].phase);
            }
        }
    }
    else
    {
        printf(" EEPROM init failed!\n");
    }

    printf("\r\n CT SW_Version %s", SW_VERSION);
    printf("\r\n CT HW_Version %s\r\n", HW_VERSION);

    printf("================ Start Normal Run ================ \r\n");

    while (1)
    {

        // 处睆UART AT收到的数杮
        uart_at_process();

        // ��鲢����״̬������? - 50us��ADC�ж��и���־
        if (sys_param.flags.task.state_machine_ready)
        {
            system_state_machine(&sys_param.grid, &sys_param.ct1, &sys_param.ct2, &sys_param.ct3);
            sys_param.flags.task.state_machine_ready = 0;
        }

        // ����AC�����ĵ�ѹ��Чֵ��Ƶ��
        grid_task();

        // ��鲢����LED��������
        // mmi_task();

        // ��鲢�������Է�������?
        debug_task();

        // ��鲢����SN����
        debug_sn_task();

        // ִ��FFT����
        fft_check_and_analyze();

        // CT������Чֵ�Լ�����Ƿ�ǯ�������ϣ�������?3��CT�ϵĹ���
        ct_task();

        // ʶ��΢�����ĸ�CT��
        inv_phase_detect_fix_direction_task();

        // �㲥����/���๦��
        boardcast_power_task();

        // ����Ƿ���ҪFFT������������ÿ10s�㲥һ�ν�������
        broadcast_other_task();

        // UART1���Ͷ��д���
        uart1_tx_queue_process();

        // ����1s����
        param_update_1s_task();

        // ML307R 4G模组任务（非阻塞状态机，每次主循环执行）
        ml307r_task();

        // sub1g���ݽ��մ���
        sub1g_rx_task();

        // sub1g��ʱ������(�ϵ�3���ȡ��?,ÿ2���ȡRSSI)
        sub1g_timer_task();

        // ����OTA����1ms���ڣ�
        ota_manager_task();

        // UART1���Ͷ��д���
        uart1_tx_queue_process();
    }
}

/*---------------------------------------------------------------------------
 Name        : void voltage_and_current_buffer_record(void)
 Input       : 无
 Output      : 无
 Description : 电压电流缓冲记录。在ADC中断中被调用，周期50us。
               将循环缓冲数据复制到线性缓冲区，执行后续计算。
---------------------------------------------------------------------------*/
void voltage_and_current_buffer_record(void)
{
    // д�뻷�λ��壨buffer_index �ѱ�֤�� [0, TOTAL_SAMPLES-1]������Խ�籣����
    ua_voltage_buffer[buffer_index] = sys_param.signal.ac_voltage_LPF;
    current1_buffer[buffer_index] = sys_param.signal.ct1_current_LPF;
    current2_buffer[buffer_index] = sys_param.signal.ct2_current_LPF;
/*---------------------------------------------------------------------------
 Name        : void voltage_and_current_buffer_record(void)
 Input       : 无
 Output      : 无
 Description : 电压电流缓冲区记录。在ADC中断中被调用，50us周期内被调用。
               在中断中将循环缓冲区数据复制到线性缓冲区，然后执行各种计算。
---------------------------------------------------------------------------*/
    buffer_index++;

    // �����ƻ�
    if (buffer_index >= TOTAL_SAMPLES)
    {
        buffer_index = 0;
        buffer_filled = 1;

        // ����һȦ��δ����������㣺Ƶ�ʹ��ͣ�?< 45Hz������Ƶ�ʹ���
        if (!s_zero_crossed_since_wrap)
        {
            sys_param.fault.bit.grid_frequency = 1;
        }
        s_zero_crossed_since_wrap = 0; // Ϊ��һȦ��λ
    }
}

/*---------------------------------------------------------------------------
 Name        : void system_state_machine(...)
/*---------------------------------------------------------------------------
 Name        : void grid_manager_handle_state_machine(grid_manager_t *grid_mgr)
 Input       : grid_mgr - 电网管理器
 Output      : 无
 Description : 电网状态机处理。在主循环中的while(1)中被调用。
---------------------------------------------------------------------------*/
    if (sys_param.restore_sys)
    {
        __NVIC_SystemReset();
    }

    // // ϵͳ��/�������ͱ仯���?
    // if (grid_mgr->system_type_changed)
    // {
    //     // ϵͳ���ͱ仯,���³�ʼ��
    //     DEBUG_PRINTF("[State Machine] System type changed, re-initializing...\r\n");

    //     sys_param.state = SYS_INIT;

    //     // ��������ʶ�����?
    //     phase_identify_init(&sys_param.grid.phase_id);

    //     // ���ù��ʷ�����
    //     ct_power_direction_detect_init(&sys_param.ct1);
    //     ct_power_direction_detect_init(&sys_param.ct2);
    //     ct_power_direction_detect_init(&sys_param.ct3);

    //     grid_mgr->system_type_changed = false;
    //     return;
    // }

    // ״̬���߼� - ֻ���ݱ�־λ��״̬�л�
    switch (sys_param.state)
    {
    case SYS_INIT: // Case 0: ���AC��ѹ����͵���Ƶ��?

        // �ȼ��Ƶ���Ƿ��й���?
        if (sys_param.fault.bit.grid_frequency)
        {
            sys_param.state = SYS_FREQ_FAULT;
            break;
        }

        if (grid_mgr->zero_cross.zero_cross_count >= ZERO_CROSS_COUNT_TARGET) // ��⵽�㹻�Ĺ���������޹��ϣ�״̬ת��
        {
            sys_param.state = SYS_WAIT_CT;
        }
        break;

    case SYS_WAIT_CT: // Case 1: �ȴ�CT���?

        if (sys_param.grid.phase_id.sequence_k == 0)
        {
            sys_param.state = SYS_PHASE_IDENTIFY;
            DEBUG_PRINTF("[State Machine] sequence_k=0, entering SYS_PHASE_IDENTIFY.\r\n");
        }
        else
        {
            sys_param.state = SYS_NORMAL_RUN;
            DEBUG_PRINTF("[State Machine] skip phase identify, entering SYS_NORMAL_RUN.\r\n");
        }
        break;

    case SYS_PHASE_IDENTIFY: // Case 2: ����ʶ��

        if (grid_mgr->phase_id.identification_valid) // ����ʶ����ɣ�ֱ�ӽ�����������?
        {
            // �̶����ʷ���Ϊ�������跽����
            sys_param.ct1.power.direction_detect_complete = 1;
            sys_param.ct2.power.direction_detect_complete = 1;
            sys_param.ct3.power.direction_detect_complete = 1;

            // // �Զ�ʶ�������浽EEPROM������������Ч����������tag��ƥ����ع�Ĭ��?1��
            // eeprom_save_set_param();

            printf("[State Machine] Auto phase identify done. CT Mapping: CT1->Phase %c, CT2->Phase %c, CT3->Phase %c\r\n",
                   'A' + grid_mgr->phase_id.ct_to_phase[0],
                   'A' + grid_mgr->phase_id.ct_to_phase[1],
                   'A' + grid_mgr->phase_id.ct_to_phase[2]);

            sys_param.state = SYS_NORMAL_RUN;
        }
        // else if (!grid_mgr->ct_connected) // CT�Ͽ������س�ʼ״̬
        // {
        //     DEBUG_PRINTF("[State Machine] Ct Not Connected.\r\n");
        // }
        break;

    case SYS_POWER_DIR_DETECT: // Case 3: ���ʷ�����

        // �������CT�Ĺ��ʷ����Ƿ��Ѽ�����
        if (sys_param.ct1.power.direction_detect_complete &&
            sys_param.ct2.power.direction_detect_complete &&
            sys_param.ct3.power.direction_detect_complete)
        {
            if (!grid_mgr->phase_id.relay_opening_pending)
            {
                // ��ʼ2��̵����򿪹���?
                grid_mgr->phase_id.relay_opening_pending = 1;
                grid_mgr->phase_id.relay_open_timer_ms = 0;
            }
            else if (grid_mgr->phase_id.relay_open_timer_ms >= 2000)
            {
                // 2��̵����򿪹�����ɣ�������������״̬
                grid_mgr->phase_id.relay_opening_pending = 0;
                grid_mgr->phase_id.relay_open_timer_ms = 0;
                sys_param.state = SYS_NORMAL_RUN;
                printf("[State Machine] Power direction detection complete, entering SYS_NORMAL_RUN.\r\n");
            }
        }
        // else if (!grid_mgr->ct_connected) // CT�Ͽ������س�ʼ״̬
        // {
        //     sys_param.state = SYS_INIT;
        //     state_machine_partial_reset(); // �������ò���
        //     grid_mgr->phase_id.identification_valid = 0;
        // }
        break;

    case SYS_NORMAL_RUN: // Case 4: ��������

        // if (!grid_mgr->ct_connected)
        // {
        //     // ��⵽CTδ���룬���صȴ�״̬
        //     sys_param.state = SYS_INIT;
        //     state_machine_partial_reset(); // �������ò���
        //     grid_mgr->phase_id.identification_valid = 0;
        // }

        break;

    case SYS_FREQ_FAULT: // Case 5: ����Ƶ�ʹ��ϣ�����45Hz-65Hz��Χ��
        // Ƶ�ʹ������ж��� zero_cross_detect() ��⣺�Ϸ�ʱ��� fault.bit.grid_frequency
        // �˴�����������λ���ָ���ص�? SYS_INIT ���³�ʼ��
        if (sys_param.fault.bit.grid_frequency == 0)
        {
            DEBUG_PRINTF("[State Machine] Grid frequency recovered, back to SYS_INIT.\r\n");
            sys_param.state = SYS_INIT;
            sys_param.grid.zero_cross.zero_cross_count = 0;
        }
        break;

    default:
        sys_param.state = SYS_INIT;
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : void inv_phase_detect_fix_direction_task(void)
 Input       : 无
 Output      : 无
 Description : 逆变器相位检测与方向修正任务。
---------------------------------------------------------------------------*/
{
    // Ԥ�ȼ���FFT�ɼ�������������ÿ��CT�������ظ��ж�
    bool fft_collect_enabled = (sys_param.state == SYS_NORMAL_RUN) && (sys_param.grid.phase_id.sequence_k > 0) && (sys_param.fft_identify.enable_collect == 1);

    // CT1���ʴ���
    if (sys_param.ct1.power.power_ready)
    {
        // �ڹ��ʷ�����״̬�½��й��ʷ�����
        if (sys_param.state == SYS_POWER_DIR_DETECT)
        {
            ct_power_direction_detect_process(&sys_param.ct1);
        }

        sys_param.ct1.power.power_ready = 0;

        // FFT���ݲɼ�����������ǰ������ʱ��
        if (fft_collect_enabled)
        {
            fft_collect_power_data_3ch(CT_CHANNEL_1, sys_param.ct1.power.fix_dir_power);
        }
    }

    // CT2���ʴ���
    if (sys_param.ct2.power.power_ready)
    {
        // �ڹ��ʷ�����״̬�½��й��ʷ�����
        if (sys_param.state == SYS_POWER_DIR_DETECT)
        {
            ct_power_direction_detect_process(&sys_param.ct2);
        }

        sys_param.ct2.power.power_ready = 0;

        // FFT���ݲɼ�����������ǰ������ʱ��
        if (fft_collect_enabled)
        {
            fft_collect_power_data_3ch(CT_CHANNEL_2, sys_param.ct2.power.fix_dir_power);
        }
    }

    //  CT3���ʴ���
    if (sys_param.ct3.power.power_ready)
    {
        // �ڹ��ʷ�����״̬�½��й��ʷ�����
        if (sys_param.state == SYS_POWER_DIR_DETECT)
        {
            ct_power_direction_detect_process(&sys_param.ct3);
        }

        sys_param.ct3.power.power_ready = 0;

        // FFT���ݲɼ�����������ǰ������ʱ��
        if (fft_collect_enabled)
        {
            fft_collect_power_data_3ch(CT_CHANNEL_3, sys_param.ct3.power.fix_dir_power);
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void ct_task(void)
/*---------------------------------------------------------------------------
 Name        : void ct_task(void)
 Input       : 无
 Output      : 无
 Description : CT电流有效值计算。
---------------------------------------------------------------------------*/
void ct_task(void)
{
    // ���������� RMS ���ʼ����������������? buffer_index ����
    if (sys_param.flags.rms_calc_ready || sys_param.flags.task.power_calc_ready)
    {
        s_calc_buf_snap = buffer_index; // ���գ�������������ڴ����
        uint16_t spc = sys_param.grid.samples_per_cycle;
        if (spc > 0 && spc <= TOTAL_SAMPLES && sys_param.grid.zero_cross.frequency_valid)
        {
            // �� ua ���λ������? spc ������չ���� last_ua[0..spc-1]
            copy_ua_ring_to_last_ua_linear(spc, s_calc_buf_snap);
        }
    }

    // 1. �ȼ��� RMS���빦�ʴ���ͬ���ڣ����� RMS ��֤ PF ʹ������ֵ��
    if (sys_param.flags.rms_calc_ready)
    {
        ct_rms_calculate();
        sys_param.flags.rms_calc_ready = 0;

        // �ж��Ƿ�ǯ�ڵ�����
        ct_online_detect_process(&sys_param.ct1, sys_param.ct1.rms_value);
        ct_online_detect_process(&sys_param.ct2, sys_param.ct2.rms_value);
        ct_online_detect_process(&sys_param.ct3, sys_param.ct3.rms_value);

        // ͳ������CT����
        sys_param.grid.online_ct_count = 0;

        if (sys_param.ct1.status.connect_status == CT_STATUS_ONLINE)
            sys_param.grid.online_ct_count++;
        if (sys_param.ct2.status.connect_status == CT_STATUS_ONLINE)
            sys_param.grid.online_ct_count++;
        if (sys_param.ct3.status.connect_status == CT_STATUS_ONLINE)
            sys_param.grid.online_ct_count++;

        // ����CT���ӱ�־
        sys_param.grid.ct_connected = (sys_param.grid.online_ct_count > 0);

        // �ж�ϵͳ���ͣ�����/���ࣩ
        static bool last_is_three_phase = false;
        bool current_is_three_phase;

        // ������Ա�����࣬����ʶ�𲻿ɱ�Ϊ����
        if (sys_param.grid.online_ct_count >= 2)
        {
            current_is_three_phase = true;
        }
        else if (sys_param.grid.online_ct_count == 1 && last_is_three_phase == false)
        {
            current_is_three_phase = false;
        }
        else
        {
            current_is_three_phase = sys_param.is_three_phase; // ����ԭ��״̬
        }

        // ����ϵͳ����
        if (sys_param.grid.online_ct_count > 0)
        {
            sys_param.is_three_phase = current_is_three_phase;
            last_is_three_phase = current_is_three_phase;
        }
    }

    // 2. �ټ��㹦�ʺ� PF����������ո��µ�? rms_value��
    if (sys_param.flags.task.power_calc_ready)
    {
        // ���๦�ʼ�����������������
        ct_power_calculate_task();

        // ����ʶ���ڴ˴����ã�last_ua���ա�s_calc_buf_snap��RMS�����ڱ���׼������
        phase_identify_process(&sys_param.grid.phase_id);

        sys_param.flags.task.power_calc_ready = 0;

        // ÿ���������ڹ��ʼ�����ɺ���λ�㲥�������?
        sys_param.flags.task.power_cycle_ready = 1;
    }
}

/*---------------------------------------------------------------------------
 Name        : void adc_sample_and_process(void)
/*---------------------------------------------------------------------------
 Name        : void adc_sample_and_process(void)
 Input       : 无
 Output      : 无
 Description : ADC采样数据处理。
---------------------------------------------------------------------------*/
    sys_param.signal.adc1_raw[0] = ADC_GetValue(CM_ADC1, ADC_CH0); // I_CT1
    sys_param.signal.adc1_raw[1] = ADC_GetValue(CM_ADC1, ADC_CH1); // I_CT2
    sys_param.signal.adc1_raw[2] = ADC_GetValue(CM_ADC1, ADC_CH2); // I_CT3
    sys_param.signal.adc1_raw[3] = ADC_GetValue(CM_ADC1, ADC_CH3); // V_AC
    sys_param.signal.adc1_raw[4] = ADC_GetValue(CM_ADC1, ADC_CH4); // V_1.65V

    // ==========================��ͨ�˲�==========================
    sys_param.signal.adc1_raw_LPF[0] = KLPF_Function_Float(sys_param.signal.adc1_raw[0], 0.3f, 0); // I_CT1�˲�ֵ
    sys_param.signal.adc1_raw_LPF[1] = KLPF_Function_Float(sys_param.signal.adc1_raw[1], 0.3f, 1); // I_CT2�˲�ֵ
    sys_param.signal.adc1_raw_LPF[2] = KLPF_Function_Float(sys_param.signal.adc1_raw[2], 0.3f, 2); // I_CT3�˲�ֵ
    sys_param.signal.adc1_raw_LPF[3] = KLPF_Function_Float(sys_param.signal.adc1_raw[3], 0.3f, 3); // V_AC�˲�ֵ
    sys_param.signal.adc1_raw_LPF[4] = KLPF_Function_Float(sys_param.signal.adc1_raw[4], 0.3f, 4); // V_1.65V�˲�ֵ

    // ==========================���ݴ���==========================
    // ������ѹת����ת��ϵ�� ADC/4096*3300mV*0.2667(V/mV)
    sys_param.signal.ac_voltage = (float)((int)sys_param.signal.adc1_raw[3] - (int)sys_param.signal.adc1_raw[4]) * 0.2149f;

    // ��·����������ת����ת��ϵ�� ADC/4096*3300mV*0.025(A/mV)
    sys_param.signal.ct1_current = (float)((int)sys_param.signal.adc1_raw[0] - (int)sys_param.signal.adc1_raw[4]) * 0.0201416f;
    sys_param.signal.ct2_current = (float)((int)sys_param.signal.adc1_raw[1] - (int)sys_param.signal.adc1_raw[4]) * 0.0201416f;
    sys_param.signal.ct3_current = (float)((int)sys_param.signal.adc1_raw[2] - (int)sys_param.signal.adc1_raw[4]) * 0.0201416f;

    // 1.65V�ο���ѹת����ADC/4096*3.3V
    sys_param.signal.v1p65_voltage = (float)sys_param.signal.adc1_raw[4] * 0.000806f;

    // ==========================�˲�������ݴ���?==========================
    sys_param.signal.ac_voltage_LPF = (float)((int)sys_param.signal.adc1_raw_LPF[3] - (int)sys_param.signal.adc1_raw_LPF[4]) * 0.2149f;
    sys_param.signal.ct1_current_LPF = (float)((int)sys_param.signal.adc1_raw_LPF[0] - (int)sys_param.signal.adc1_raw_LPF[4]) * 0.0201416f;
    sys_param.signal.ct2_current_LPF = (float)((int)sys_param.signal.adc1_raw_LPF[1] - (int)sys_param.signal.adc1_raw_LPF[4]) * 0.0201416f;
    sys_param.signal.ct3_current_LPF = (float)((int)sys_param.signal.adc1_raw_LPF[2] - (int)sys_param.signal.adc1_raw_LPF[4]) * 0.0201416f;
    sys_param.signal.v1p65_voltage_LPF = (float)sys_param.signal.adc1_raw_LPF[4] * 0.000806f;
}

/*---------------------------------------------------------------------------
 Name        : void ct_rms_calculate(void)
 Input       : 无
 Output      : 无
 Description : CT有效值计算。使用缓冲快照索引作为时间基准。
               避免处理过程中产生异步问题。
---------------------------------------------------------------------------*/
 Output      : ��
 Description : ���ж����������־λ������ѭ����������ҵ���߼�?
---------------------------------------------------------------------------*/
void set_task_flags_from_interrupt(void)
{
    // ÿ��ADC�ж϶���Ҫ������
    sys_param.flags.task.fault_check_ready = 1;

    // ÿ��ADC�ж϶���Ҫ����״̬��
    sys_param.flags.task.state_machine_ready = 1;

    // ע�⣺ct_phase_identify_ready �ѷ���������ʶ���� ct_task �� power_calc_ready ����
}

/*---------------------------------------------------------------------------
 Name        : static void copy_ua_ring_to_last_ua_linear(uint16_t spc, uint16_t snap_idx)
/*---------------------------------------------------------------------------
 Name        : void three_phase_broadcast_task(void)
 Input       : 无
 Output      : 无
 Description : 处理三相功率及其他广播任务。在状态机完成后执行。
---------------------------------------------------------------------------*/
 Output      : ��
 Description : ���๦�ʼ���������������������ѭ���е��ã���
               �������������ʷ��������������״�? + ����ʶ����Ч + �����������?
                        + frequency_valid + !frequency_fault��
               ʹ�� s_calc_buf_snap ȷ�����ε����������?
               A/B/C �����ѹ����? last_ua_voltage_buffer[0..spc-1]�����Կ��գ�ȡֵ��
               ��֤�������ʱ�䴰��ȫ���룬����? buffer_index �ƽ��������λƯ�ơ�?
               PF = avg_power / (ua_vol_rms * ct_rms_value)���������޷���[-1,1]��
---------------------------------------------------------------------------*/
static void ct_power_calculate_task(void)
{
    if ((sys_param.state != SYS_POWER_DIR_DETECT && sys_param.state != SYS_NORMAL_RUN) ||
        !sys_param.grid.phase_id.identification_valid ||
        !buffer_filled ||
        !sys_param.grid.zero_cross.frequency_valid ||
        sys_param.grid.frequency_fault)
    {
        return;
    }

    uint16_t spc = sys_param.grid.samples_per_cycle;
    if (spc == 0 || spc > TOTAL_SAMPLES)
    {
        return;
    }

    uint16_t pb = sys_param.grid.phase_b_delay_samples;
    uint16_t pc = sys_param.grid.phase_c_delay_samples;

    uint8_t ct1_phase = sys_param.grid.phase_id.ct_to_phase[0];
    uint8_t ct2_phase = sys_param.grid.phase_id.ct_to_phase[1];
    uint8_t ct3_phase = sys_param.grid.phase_id.ct_to_phase[2];

    // ���ε���������㣨��? RMS ʹ��ͬһ���գ�
    uint16_t curr_start = (uint16_t)((s_calc_buf_snap + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES);

    float sum1 = 0.0f, sum2 = 0.0f, sum3 = 0.0f;

    for (uint16_t i = 0; i < spc; i++)
    {
        // �����ѹ����? last_ua ���Կ���ȡֵ����֤�����ʱ�䴰����?
        float va = last_ua_voltage_buffer[i];
        float vb = last_ua_voltage_buffer[(i + spc - pb) % spc];
        float vc = last_ua_voltage_buffer[(i + spc - pc) % spc];

        float phase_voltage[3];
        phase_voltage[0] = va;
        phase_voltage[1] = vb;
        phase_voltage[2] = vc;

        // �����ӻ��λ��尴�������ȡ�?
        uint16_t ci = (curr_start + i) % TOTAL_SAMPLES;
        sum1 += phase_voltage[ct1_phase] * current1_buffer[ci];
        sum2 += phase_voltage[ct2_phase] * current2_buffer[ci];
        sum3 += phase_voltage[ct3_phase] * current3_buffer[ci];
    }

    // ---- ������·�й����� ----
    float inv_spc = 1.0f / (float)spc;

    sys_param.ct1.power.avg_power = sum1 * inv_spc;
    sys_param.ct1.power.fix_dir_power = sys_param.ct1.power.avg_power * sys_param.ct1.power.power_direction;
    sys_param.ct1.power.power_ready = 1;
    sys_param.ct1.power.sum_power = 0.0f;
    sys_param.ct1.power.power_sample_count = 0;

    sys_param.ct2.power.avg_power = sum2 * inv_spc;
    sys_param.ct2.power.fix_dir_power = sys_param.ct2.power.avg_power * sys_param.ct2.power.power_direction;
    sys_param.ct2.power.power_ready = 1;
    sys_param.ct2.power.sum_power = 0.0f;
    sys_param.ct2.power.power_sample_count = 0;

    sys_param.ct3.power.avg_power = sum3 * inv_spc;
    sys_param.ct3.power.fix_dir_power = sys_param.ct3.power.avg_power * sys_param.ct3.power.power_direction;
    sys_param.ct3.power.power_ready = 1;
    sys_param.ct3.power.sum_power = 0.0f;
    sys_param.ct3.power.power_sample_count = 0;

    // ---- ���㹦������ PF = P / (V_rms * I_rms) ----
    float v_rms = sys_param.grid.ua_vol_rms;
    if (v_rms > 0.1f)
    {
        if (sys_param.ct1.rms_value > CT_OFFLINE_THRESHOLD)
        {
            float pf = sys_param.ct1.power.avg_power / (v_rms * sys_param.ct1.rms_value);
            if (pf > 1.0f)
                pf = 1.0f;
            if (pf < -1.0f)
                pf = -1.0f;
            sys_param.ct1.power.power_factor = pf;
        }
        if (sys_param.ct2.rms_value > CT_OFFLINE_THRESHOLD)
        {
            float pf = sys_param.ct2.power.avg_power / (v_rms * sys_param.ct2.rms_value);
            if (pf > 1.0f)
                pf = 1.0f;
            if (pf < -1.0f)
                pf = -1.0f;
            sys_param.ct2.power.power_factor = pf;
        }
        if (sys_param.ct3.rms_value > CT_OFFLINE_THRESHOLD)
        {
            float pf = sys_param.ct3.power.avg_power / (v_rms * sys_param.ct3.rms_value);
            if (pf > 1.0f)
                pf = 1.0f;
            if (pf < -1.0f)
                pf = -1.0f;
            sys_param.ct3.power.power_factor = pf;
        }
    }

    // ---- ����ϵͳ�У������ߵ�CT����/PF���� ----
    if (!sys_param.is_three_phase)
    {
        if (sys_param.ct1.status.connect_status != CT_STATUS_ONLINE)
        {
            sys_param.ct1.power.fix_dir_power = 0.0f;
            sys_param.ct1.power.ct_sub1g_boardcast_power_avg = 0.0f;
            sys_param.ct1.power.power_factor = 0.0f;
        }
        if (sys_param.ct2.status.connect_status != CT_STATUS_ONLINE)
        {
            sys_param.ct2.power.fix_dir_power = 0.0f;
            sys_param.ct2.power.ct_sub1g_boardcast_power_avg = 0.0f;
            sys_param.ct2.power.power_factor = 0.0f;
        }
        if (sys_param.ct3.status.connect_status != CT_STATUS_ONLINE)
        {
            sys_param.ct3.power.fix_dir_power = 0.0f;
            sys_param.ct3.power.ct_sub1g_boardcast_power_avg = 0.0f;
            sys_param.ct3.power.power_factor = 0.0f;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void fault_detection_task(void)
 Input       : 无
 Output      : 无
 Description : 故障检测任务。
---------------------------------------------------------------------------*/
static void fault_detection_task(void)
{
    // ��̬���ϼ�����
    static uint16_t ac_fault_count = 0;
    static uint16_t ct1_fault_count = 0;
    static uint16_t ct2_fault_count = 0;
    static uint16_t ct3_fault_count = 0;
    static uint16_t v1p65_fault_count = 0;

    // ������ѹ���ϼ��?>380V �� <176V��
    if ((fabsf(sys_param.signal.ac_voltage) > FAULT_TH_AC_V_HIGH) ||
        (sys_param.grid.ua_vol_rms < FAULT_TH_AC_V_LOW))
    {
        ac_fault_count++;
        if (ac_fault_count >= FAULT_CONFIRM_COUNT)
        {
            if (sys_param.fault.bit.ac_sample == 0)
            {
                sys_param.fault.bit.ac_sample = 1;
                sys_param.fault_delay = 0;
            }
        }
    }
    else
    {
        ac_fault_count = 0; // ���������?
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ac_sample = 0;
    }

    // CT1�������������ϼ��?>60A��
    if (fabsf(sys_param.signal.ct1_current) > FAULT_TH_CT_I_HIGH)
    {
        ct1_fault_count++;
        if (ct1_fault_count >= FAULT_CONFIRM_COUNT)
        {
            if (sys_param.fault.bit.ct1_sample == 0)
            {
                sys_param.fault.bit.ct1_sample = 1;
                sys_param.fault_delay = 0;
            }
        }
    }
    else
    {
        ct1_fault_count = 0; // ���������?
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ct1_sample = 0;
    }

    // CT2�������������ϼ��?>60A��
    if (fabsf(sys_param.signal.ct2_current) > FAULT_TH_CT_I_HIGH)
    {
        ct2_fault_count++;
        if (ct2_fault_count >= FAULT_CONFIRM_COUNT)
        {
            if (sys_param.fault.bit.ct2_sample == 0)
            {
                sys_param.fault.bit.ct2_sample = 1;
                sys_param.fault_delay = 0;
            }
        }
    }
    else
    {
        ct2_fault_count = 0; // ���������?
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ct2_sample = 0;
    }

    // CT3�������������ϼ��?>60A��
    if (fabsf(sys_param.signal.ct3_current) > FAULT_TH_CT_I_HIGH)
    {
        ct3_fault_count++;
        if (ct3_fault_count >= FAULT_CONFIRM_COUNT)
        {
            if (sys_param.fault.bit.ct3_sample == 0)
            {
                sys_param.fault.bit.ct3_sample = 1;
                sys_param.fault_delay = 0;
            }
        }
    }
    else
    {
        ct3_fault_count = 0; // ���������?
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ct3_sample = 0;
    }

    // 1.65V�ο���ѹ���ϼ�⣨��Χ��⣩
    if (sys_param.signal.v1p65_voltage > FAULT_TH_V1P65_HIGH ||
        sys_param.signal.v1p65_voltage < FAULT_TH_V1P65_LOW)
    {
        v1p65_fault_count++;
        if (v1p65_fault_count >= FAULT_CONFIRM_COUNT)
        {
            if (sys_param.fault.bit.v1p65_sample == 0)
            {
                sys_param.fault.bit.v1p65_sample = 1;
                sys_param.fault_delay = 0;
            }
        }
    }
    else
    {
        v1p65_fault_count = 0; // ���������?
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.v1p65_sample = 0;
    }

    // ���¹��Ͻ��?
    if (sys_param.fault.data > 0)
    {
        sys_param.fault_result = 1;
    }
    else
    {
        sys_param.fault_result = 0;
    }
}

/*---------------------------------------------------------------------------
 Name        : void system_timer_management(void)
 Input       : 无
 Output      : 无
 Description : 系统定时器管理。
---------------------------------------------------------------------------*/
void system_timer_management(void)
{
    sys_param.timer.timer_1ms_count++;

    // ============= 1ms����ʱ���� =============
    if (sys_param.timer.timer_1ms_count >= TIMER_1MS_CYCLES) // 20 * 50us = 1ms
    {
        sys_param.timer.timer_1ms_count = 0; // ����1ms������

        // sub1g��ʱ������
        if (sys_param.sub1g.sw_version[0] == '\0') // δ�յ��汾��������ʱ
        {
            sys_param.sub1g.version_timer_ms++;
        }
        sys_param.sub1g.rssi_timer_ms++;

        // ============= 20ms����ʱ���� =============
        sys_param.timer.timer_20ms_count++;
        if (sys_param.timer.timer_20ms_count >= TIMER_20mS_CYCLES) // 1000ms = 1s
        {
            sys_param.timer.timer_20ms_count = 0;

            // ����20ms��־
            sys_param.flags.timer_20ms_flag = 1;
        }

        // ============= 1s����ʱ���� =============
        sys_param.timer.timer_1s_count++;
        if (sys_param.timer.timer_1s_count >= TIMER_1S_CYCLES) // 1000ms = 1s
        {
            sys_param.timer.timer_1s_count = 0;
            sys_param.fault_delay++; // ������ʱ����������

            static uint8_t count = 0;
            count++;
            if (count >= 10)
            {
                count = 0;
                sys_param.flags.timer_10s_flag = 1;
            }

            // ����1s��־
            sys_param.flags.timer_1s_flag = 1;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void INT_ADC_1_1_ISR(void)
 Input       : 无
 Output      : 无
 Description : ADC中断处理程序。
---------------------------------------------------------------------------*/
void ADC1_Handler(void) // 50USһ���ж�
{
    // ADC�������źŴ���
    if (ADC_GetStatus(CM_ADC1, ADC_FLAG_EOCA) == SET)
    {
        ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);

        // GPIO_SetPins(GPIO_PORT_F, GPIO_PIN_02);

        adc_sample_and_process();

        // ����ѹ�������ݻ�����
        voltage_and_current_buffer_record();

        // �����⣺ʹ��δ�˲���ԭʼ��ѹ������ LPF ������λ�ͺ��¹�������ƫ��
        // 2��������ͬ�����жϱ��������㹻�Ŀ������������� LPF
        zero_cross_detect(&sys_param.grid.zero_cross, sys_param.signal.ac_voltage);

        // ϵͳ��ʱ������
        system_timer_management();

        // ���������־λ������ѭ����������ҵ���߼�?
        set_task_flags_from_interrupt();

        // GPIO_ResetPins(GPIO_PORT_F, GPIO_PIN_02);
    }
    __DSB(); /* Arm Errata 838869 */
}

/*---------------------------------------------------------------------------
 Name        : void SysTick_Handler(void)
 Input       : 无
 Output      : 无
 Description : SysTick系统时钟中断处理程序。
---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    // ����1ms��ʱ��־
    sys_param.flags.timer_1ms_flag = 1;

    // ����ʶ��100ms��ʱ
    static uint8_t phase_identify_counter = 0;
    if (sys_param.state == SYS_PHASE_IDENTIFY)
    {
        phase_identify_counter++;
        if (phase_identify_counter >= 100) // 100ms
        {
            phase_identify_counter = 0;
            phase_identify_timer_100ms = 1;
        }
    }
    else
    {
        phase_identify_counter = 0;
        phase_identify_timer_100ms = 0;
    }

    // �̵����򿪶�ʱ������
    if (sys_param.grid.phase_id.relay_opening_pending)
    {
        sys_param.grid.phase_id.relay_open_timer_ms++;
    }

    // ========== δ����豸����������?(1ms) ==========
    // ����������Ч��δ����豸�������������
    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.inv_request_pair_list[i].is_valid)
        {
            sys_param.inv_request_pair_list[i].unpaired_updata_ms++;
        }

        if (sys_param.inv_request_pair_list[i].paired_unvalid_ms > 0)
        {
            sys_param.inv_request_pair_list[i].paired_unvalid_ms--;
            if (sys_param.inv_request_pair_list[i].paired_unvalid_ms == 0)
            {
                // ��δ����б���ɾ��?
                inv_request_pair_list_remove(sys_param.inv_request_pair_list[i].sub1g_addr);
            }
        }
        else
        {
            sys_param.inv_request_pair_list[i].paired_unvalid_ms = 0;
        }
    }

    // ���ʹ㲥�����ڼ���������
    sys_param.date_broadcast_counter++;

#ifdef DEBUG_ENABLE
    sys_param.timer.debug_1ms_count++;
#endif

    sys_param.mmi.led_count++;
    sys_param.mmi.display_timer_ms++;

    // Sub1G ͨ�ų�ʱ���?
    if (sys_param.sub1g.state == 4) // ֻ����ͨ������״̬�ż�ⳬ�?
    {
        sys_param.sub1g.timeout_count++;
        if (sys_param.sub1g.timeout_count >= 15000)
        {
            sys_param.sub1g.state = 3;
            sys_param.sub1g.timeout_count = 0;
        }
    }

    // ��������ʱ��
    sys_param.sub1g.reboot_count++;
    if (!g_ota_manager.disable_broadcast)
    {
        if (sys_param.sub1g.reboot_count >= 60000)
        {
            sub1g_reboot();
            sys_param.sub1g.reboot_count = 0;
        }
    }
    else
    {
        sys_param.sub1g.reboot_count = 0;
    }

    // FFT�ɼ��ӳٿ��ƣ�is_ffting=1��ȴ�?2��ſ�ʼ�ɼ�?
    static uint16_t fft_delay_count = 0;
    if (sys_param.fft_identify.is_ffting == 1)
    {
        if (fft_delay_count < 2000)
        {
            fft_delay_count++;

            // FFT�ȴ��ڼ�ǰ1�룺ÿ100ms��һ���ط���־������ѭ������ʵ�ʷ���
            if (fft_delay_count >= 100 && fft_delay_count <= 1000 &&
                (fft_delay_count % 100 == 0))
            {
                sys_param.fft_identify.retry_flag = 1;
            }
        }

        if (fft_delay_count >= 2000)
        {
            sys_param.fft_identify.enable_collect = 1;
        }
    }
    else
    {
        fft_delay_count = 0; // is_ffting=0ʱ���ü���
    }

    if (sys_param.fft_identify.boardcast_interval > 0)
    {
        sys_param.fft_identify.boardcast_interval--;
    }

    SysTick_IncTick();
    __DSB(); /* Arm Errata 838869 */
}

/*---------------------------------------------------------------------------
 Name        : void system_param_init(void)
 Input       : 无
 Output      : 无
 Description : 系统参数初始化。
---------------------------------------------------------------------------*/
void system_param_init(void)
{
    // һ�������������ṹ��
    memset(&sys_param, 0, sizeof(sys_param_t));

    // ��ʼ��ϵͳ��־λ
    system_flags_init();

    // ��ʼ��CT�Ƿ����߼��?
    ct_online_detect_init(&sys_param.ct1);
    ct_online_detect_init(&sys_param.ct2);
    ct_online_detect_init(&sys_param.ct3);

    power_calc_init(&sys_param.ct1.power);
    power_calc_init(&sys_param.ct2.power);
    power_calc_init(&sys_param.ct3.power);

    grid_manager_init(); // ��ʼ������������

    // Ĭ������sequence_k=1��CT1=A�࣬CT2=B���ͺ�120�㣬CT3=C�೬ǰ120�㣩
    sys_param.grid.phase_id.sequence_k = 1;
    sys_param.grid.phase_id.identification_valid = 1;
    update_ct_to_phase_mapping(1);

    // ���ʷ���̶�Ϊ��������ִ�з���������?
    sys_param.ct1.power.power_direction = 1;
    sys_param.ct1.power.direction_detect_complete = 1;
    sys_param.ct2.power.power_direction = 1;
    sys_param.ct2.power.direction_detect_complete = 1;
    sys_param.ct3.power.power_direction = 1;
    sys_param.ct3.power.direction_detect_complete = 1;

    ota_manager_init(); // ��ʼ��OTA������

    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // ��ʼ��΢���豸��Ϣ����
        memset(sys_param.paired_inv_info[i].device_sn, 0, SN_LENGTH + 1);
        memset(sys_param.paired_inv_info[i].device_sn, 0, sizeof(sys_param.paired_inv_info[i].device_sn));
        sys_param.paired_inv_info[i].sub1g_addr = 0;
        sys_param.paired_inv_info[i].siid = 0;

        // ��ʼ��δ����豸�б�?
        sys_param.inv_request_pair_list[i].is_valid = false;
        sys_param.inv_request_pair_list[i].sub1g_addr = 0;
        sys_param.inv_request_pair_list[i].unpaired_updata_ms = 0;
        sys_param.inv_request_pair_list[i].device_sn[0] = '\0';
        sys_param.inv_request_pair_list[i].product_model = 0;

        // ��ʼ���û�����б�?
        sys_param.user_pair_list[i].is_valid = false;
        sys_param.user_pair_list[i].device_sn[0] = '\0';
    }

    sys_param.anti_backflow_switch = 1; // Ĭ�Ͽ�������������

    // Ĭ���ǵ��෢��
    sys_param.is_three_phase = false;

    // ��ʼ�� Sub1G ״̬Ϊδ����
    sys_param.sub1g.state = 1;         // 1 = δ�����?
    sys_param.sub1g.timeout_count = 0; // ��ʱ����������
    sys_param.sub1g.reboot_count = 0;  // ͨ�ų�ʱ����������

    sys_param.sub1g.version_timer_ms = 0;
    sys_param.sub1g.rssi_timer_ms = 0;
    sys_param.sub1g.rssi = 0;
    sys_param.sub1g.ct_sub1g_addr = 0;
    sys_param.sub1g.sw_version[0] = '\0'; // �汾�ַ�����ʼ��Ϊ��
    sys_param.sub1g.channel_index = 0xFF; // CT�ŵ�ֵ��ʼ��

    // ��ʼ��slave�汾����
    sys_param.slave_version.inv_sub1g_version[0] = '\0';
    sys_param.slave_version.inv_800w_version[0] = '\0';
    sys_param.slave_version.inv_2500w_version[0] = '\0';
    sys_param.slave_version.slave_version_reported = false;

    // ΢������ʶ���ʼ��?
    sys_param.fft_identify.identified_ct = 0;
    sys_param.fft_identify.is_ffting = 0;
    sys_param.fft_identify.enable_collect = 0;
    sys_param.fft_identify.resend_cmd = false;
    sys_param.fft_identify.retry_flag = 0;
    sys_param.fft_identify.power = 100;
    sys_param.fft_identify.interval_time = 4;

    sys_param.fft_identify.consecutive_success_count = 0;
    sys_param.fft_identify.last_identified_ct = 0;
    sys_param.fft_identify.boardcast_interval = 0;
    sys_param.fft_identify.final_confirm_pending = false;
}

/*---------------------------------------------------------------------------
 Name        : void state_machine_partial_reset(void)
 Input       : 无
 Output      : 无
 Description : 状态机部分重置。
---------------------------------------------------------------------------*/
void state_machine_partial_reset(void)
{
    // ����CT���߼�����
    ct_online_detect_init(&sys_param.ct1);
    ct_online_detect_init(&sys_param.ct2);
    ct_online_detect_init(&sys_param.ct3);

    // ���ù��ʼ������?
    power_calc_init(&sys_param.ct1.power);
    power_calc_init(&sys_param.ct2.power);
    power_calc_init(&sys_param.ct3.power);

    // ���ù��ʷ����⣨�������򣬲����㣩
    sys_param.ct1.power.power_direction = 1;
    sys_param.ct1.power.direction_detect_complete = 1;
    sys_param.ct1.power.direction_sample_count = 0;
    sys_param.ct1.power.direction_power_sum = 0;

    sys_param.ct2.power.power_direction = 1;
    sys_param.ct2.power.direction_detect_complete = 1;
    sys_param.ct2.power.direction_sample_count = 0;
    sys_param.ct2.power.direction_power_sum = 0;

    sys_param.ct3.power.power_direction = 1;
    sys_param.ct3.power.direction_detect_complete = 1;
    sys_param.ct3.power.direction_sample_count = 0;
    sys_param.ct3.power.direction_power_sum = 0;

    //  ����CT RMSֵ
    sys_param.ct1.rms_value = 0;
    sys_param.ct2.rms_value = 0;
    sys_param.ct3.rms_value = 0;

    // ���ù�������������֣�?
    sys_param.grid.zero_cross.zero_cross_count = ZERO_CROSS_COUNT_TARGET / 2; // �������ּ������ӿ�ָ�?
    sys_param.grid.zero_cross.zero_cross_detected = 0;
    sys_param.grid.zero_cross.positive_zero_cross = 0;

    // ��������ʶ������ʱ����������ʶ������
    memset(sys_param.grid.phase_id.matching_degree, 0, sizeof(sys_param.grid.phase_id.matching_degree));
    memset(sys_param.grid.phase_id.power_factor, 0, sizeof(sys_param.grid.phase_id.power_factor));
    memset(sys_param.grid.phase_id.identify_history, 0, sizeof(sys_param.grid.phase_id.identify_history));
    sys_param.grid.phase_id.identify_count = 0;
    sys_param.grid.phase_id.consistent_count = 0;

    // ����FFTʶ�������?
    sys_param.fft_identify.identified_ct = 0;
    sys_param.fft_identify.is_ffting = 0;
    sys_param.fft_identify.enable_collect = 0;
    sys_param.fft_identify.resend_cmd = false;
    sys_param.fft_identify.retry_flag = 0;
    sys_param.fft_identify.consecutive_success_count = 0;
    sys_param.fft_identify.last_identified_ct = 0;
    sys_param.fft_identify.boardcast_interval = 0;
    sys_param.fft_identify.final_confirm_pending = false;

    // ����ϵͳ��־λ
    sys_param.flags.task.fault_check_ready = 0;
    sys_param.flags.rms_calc_ready = 0;
    sys_param.flags.task.power_calc_ready = 0;
    sys_param.flags.task.ct_phase_identify_ready = 0;

    // ���ù���״̬
    sys_param.fault.data = 0;
    sys_param.fault_result = 0;
    sys_param.fault_delay = 0;
}

/*---------------------------------------------------------------------------
 Name        : void ct_online_detect_process(ct_param_t *ct_param, float rms_value)
 Input       : ct_param - CT参数
               rms_value - RMS值
 Output      : 无
 Description : CT在线检测过程。
---------------------------------------------------------------------------*/
void ct_online_detect_process(ct_param_t *ct_param, float rms_value)
{
    if (ct_param == NULL)
        return;

    if (rms_value < CT_OFFLINE_THRESHOLD)
    {
        // RMSֵ����������ֵ
        ct_param->status.offline_count++;
        ct_param->status.online_count = 0; // ���ü���

        // ����Ƿ�ﵽ�����ж�����
        if (ct_param->status.offline_count >= CT_OFFLINE_COUNT_THRESHOLD)
        {
            ct_param->status.offline_count = CT_OFFLINE_COUNT_THRESHOLD;
            if (ct_param->status.connect_status != CT_STATUS_OFFLINE)
            {
                ct_param->status.connect_status = CT_STATUS_OFFLINE; // ״̬�ı䣺������/δ֪ -> ����
            }
        }
    }
    else if (rms_value > CT_ONLINE_THRESHOLD)
    {
        // RMSֵ����������ֵ
        ct_param->status.online_count++;
        ct_param->status.offline_count = 0; // �������߼���

        // ����Ƿ�ﵽ�����ж�����
        if (ct_param->status.online_count >= CT_ONLINE_COUNT_THRESHOLD)
        {
            ct_param->status.online_count = CT_ONLINE_COUNT_THRESHOLD;
            if (ct_param->status.connect_status != CT_STATUS_ONLINE)
            {
                ct_param->status.connect_status = CT_STATUS_ONLINE; // ״̬�ı䣺������/δ֪ -> ����
            }
        }
    }
    else
    {
        if (ct_param->status.offline_count > 0)
            ct_param->status.offline_count--;
        if (ct_param->status.online_count > 0)
            ct_param->status.online_count--;
    }
}

/*---------------------------------------------------------------------------
 Name        : void ct_power_direction_detect_process(ct_param_t *ct)
 Input       : ct - CT参数
 Output      : 无
 Description : CT功率方向检测过程。
---------------------------------------------------------------------------*/
void ct_power_direction_detect_process(ct_param_t *ct)
{
    if (ct == NULL)
        return;

    // ����Ѿ������ɣ�ֱ�ӷ���?
    if (ct->power.direction_detect_complete && ct->power.power_direction != 0)
    {
        return;
    }
    else if (ct->power.direction_detect_complete && ct->power.power_direction == 0)
    {
        // ���ü����ر���
        ct->power.direction_detect_complete = 0;
        ct->power.direction_power_sum = 0.0f;
        ct->power.direction_sample_count = 0;
    }

    // ������µĹ������ݿ���?
    if (ct->power.power_ready)
    {
        ct->power.direction_power_sum += ct->power.avg_power;
        ct->power.direction_sample_count++;

        // ����ռ���?250��������5s��������ƽ��ֵ���жϷ���
        if (ct->power.direction_sample_count >= 250)
        {
            float avg_power_50samples = ct->power.direction_power_sum / 250.0f;

            if (avg_power_50samples >= 0.0f)
            {
                ct->power.power_direction = 1.0f; // ������
            }
            else
            {
                ct->power.power_direction = -1.0f; // ��������Ҫȡ��
            }

            // ��Ǽ�����?
            ct->power.direction_detect_complete = 1;

            // ���ü��������ۼӺͣ�Ϊ��һ�ο��ܵ����¼����׼��?
            ct->power.direction_power_sum = 0.0f;
            ct->power.direction_sample_count = 0;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void system_flags_init(void)
 Input       : 无
 Output      : 无
 Description : 系统标志初始化。
---------------------------------------------------------------------------*/
void system_flags_init(void)
{
    memset(&sys_param.flags, 0, sizeof(system_flags_t));
}

/*---------------------------------------------------------------------------
 Name        : void ct_online_detect_init(ct_param_t *ct_param)
 Input       : ct_param - CT参数
 Output      : 无
 Description : CT在线检测初始化。
---------------------------------------------------------------------------*/
void ct_online_detect_init(ct_param_t *ct_param)
{
    if (ct_param == NULL)
    {
        return;
    }

    ct_param->status.offline_count = 0;
    ct_param->status.online_count = 0;
    ct_param->status.connect_status = CT_STATUS_UNKNOWN;
}

/*---------------------------------------------------------------------------
 Name        : void power_calc_init(power_calc_t *calc_power)
 Input       : calc_power - 功率计算结构体
 Output      : 无
 Description : 功率计算初始化。
---------------------------------------------------------------------------*/
void power_calc_init(power_calc_t *calc_power)
{
    if (calc_power == NULL)
    {
        return;
    }

    calc_power->sum_power = 0.0f;
    calc_power->power_sample_count = 0;
    calc_power->avg_power = 0.0f;
    calc_power->power_ready = 0;
}

/*---------------------------------------------------------------------------
 Name        : void grid_manager_init(void)
 Input       : 无
 Output      : 无
 Description : 电网管理器初始化。
---------------------------------------------------------------------------*/
void grid_manager_init(void)
{
    sys_param.state = SYS_INIT;

    // ��ʼ���������?
    sys_param.grid.zero_cross.positive_zero_cross = 0;
    sys_param.grid.zero_cross.frequency_valid = 0;

    // ����ӦƵ��Ĭ��ֵ��50Hz��׼�������������£�
    sys_param.grid.samples_per_cycle = 400;     // 50Hz: 20ms/50us
    sys_param.grid.phase_b_delay_samples = 133; // 400/3
    sys_param.grid.phase_c_delay_samples = 267; // 400*2/3

    // ��ʼ������ʶ�����?
    phase_identify_init(&sys_param.grid.phase_id);
}

void ct_power_direction_detect_init(ct_param_t *ct)
{
    if (ct == NULL)
        return;

    ct->power.power_direction = 0;           // ���ù��ʷ���
    ct->power.direction_detect_complete = 0; // ���ü����ɱ�־
}

void delay_us(uint16_t us)
{
    volatile uint16_t i = 0, j = 0;
    for (i = 0; i < us; i++)
    {
        for (j = 0; j < 15; j++)
        {
        }
    }
}

void delay_ms(uint16_t ms)
{
    volatile uint16_t i = 0, j = 0;
    for (i = 0; i < ms; i++)
    {
        delay_us(300);
    }
}

/*---------------------------------------------------------------------------
 Name        : void broadcast_three_phase_power(float *power_array)
 Input       : power_array - 功率数组
 Output      : 无
 Description : 广播三相功率。
---------------------------------------------------------------------------*/
static void broadcast_three_phase_power(float *power_array)
{
    if (g_ota_manager.disable_broadcast) // OTA�ڼ��ֹ��?
    {
        return;
    }

    static uint8_t current_slot = 0;    // ��ǰ��λ��0-7��
    static uint8_t broadcast_count = 0; // �ѹ㲥����

    // �����޹���ƫ��ֵ
    int16_t ct_to_grid_power[3] = {0};
    if (sys_param.power_work_mode == 2)
    {
        if (sys_param.is_three_phase)
        {
            // ���ࣺƽ������
            int16_t avg_power = sys_param.to_grid_power_limit / 3;
            ct_to_grid_power[0] = ct_to_grid_power[1] = ct_to_grid_power[2] = avg_power;
        }
        else
        {
            // ���ࣺ����sequence_kȷ����λ��ȫ������һ��
            int phase = (sys_param.grid.phase_id.sequence_k - 1) / 2;
            if (phase >= 0 && phase < 3)
            {
                ct_to_grid_power[phase] = sys_param.to_grid_power_limit;
            }
        }
    }

    // ������Ч��λ
    uint8_t attempts = 0;
    while (!sys_param.paired_inv_info[current_slot].is_valid && attempts < INV_DEVICE_MAX_NUM)
    {
        current_slot = (current_slot + 1) % INV_DEVICE_MAX_NUM;
        attempts++;
    }

    // ������в�λ�����?,ʹ�õ�ַ0(�㲥��ַ)
    uint32_t target_addr = (attempts < INV_DEVICE_MAX_NUM) ? sys_param.paired_inv_info[current_slot].sub1g_addr : 0;

    // ���������㲥�Ĺ���ֵ�������޹���ƫ�ƣ�
    int16_t broadcast_power_ct1 = (int16_t)(power_array[0] + ct_to_grid_power[0]);
    int16_t broadcast_power_ct2 = (int16_t)(power_array[1] + ct_to_grid_power[1]);
    int16_t broadcast_power_ct3 = (int16_t)(power_array[2] + ct_to_grid_power[2]);

    // ���¹㲥ƽ������
    sys_param.ct1.power.ct_sub1g_boardcast_power_avg = (float)broadcast_power_ct1;
    sys_param.ct2.power.ct_sub1g_boardcast_power_avg = (float)broadcast_power_ct2;
    sys_param.ct3.power.ct_sub1g_boardcast_power_avg = (float)broadcast_power_ct3;

    // �㲥���๦��
    sub1g_send_broadcast_three_phase_power(broadcast_power_ct1, broadcast_power_ct2, broadcast_power_ct3, target_addr);

    // �㲥������N�κ��л���λ
    broadcast_count++;
    if (broadcast_count >= SWITCH_INV_BOARCAST)
    {
        broadcast_count = 0;
        current_slot = (current_slot + 1) % INV_DEVICE_MAX_NUM;
    }
}

/*---------------------------------------------------------------------------
 Name        : float calculate_ct_boardcast_power_avg(void)
 Input       : 无
 Output      : 广播功率平均值
 Description : 计算CT广播功率平均值。
---------------------------------------------------------------------------*/
static void calculate_ct_boardcast_power_avg(uint8_t ct_index, bool direction_complete, float avg_power)
{
    if (!direction_complete)
    {
        // ������δ���?,�����ۼ���
        ct_power_accum[ct_index] = 0;
        return;
    }

    // �ۼӹ���
    ct_power_accum[ct_index] += avg_power;
}

/*---------------------------------------------------------------------------
 Name        : void boardcast_power_task(void)
 Input       : 无
 Output      : 无
 Description : 广播功率任务。
---------------------------------------------------------------------------*/
void boardcast_power_task(void)
{
    // �������������ÿ���������ڹ��ʼ�����ɺ󴥷�
    if (!sys_param.flags.task.power_cycle_ready)
        return;

    sys_param.flags.task.power_cycle_ready = 0;

    float power_array[3] = {0.0f, 0.0f, 0.0f};

    // �жϵ�ǰ�Ƿ�����������״̬
    if (sys_param.state == SYS_NORMAL_RUN)
    {
        // ========== ��������ģʽ:�㲥ʵ�����๦�� ==========

        // �ۼӹ���
        calculate_ct_boardcast_power_avg(0, sys_param.ct1.power.direction_detect_complete, sys_param.ct1.power.fix_dir_power);
        calculate_ct_boardcast_power_avg(1, sys_param.ct2.power.direction_detect_complete, sys_param.ct2.power.fix_dir_power);
        calculate_ct_boardcast_power_avg(2, sys_param.ct3.power.direction_detect_complete, sys_param.ct3.power.fix_dir_power);

        three_phase_broadcast_count++;

        if (three_phase_broadcast_count >= BOARDCAST_TIME)
        {
            bool all_direction_complete = sys_param.ct1.power.direction_detect_complete &&
                                          sys_param.ct2.power.direction_detect_complete &&
                                          sys_param.ct3.power.direction_detect_complete;

            if (all_direction_complete)
            {
                // ����ƽ������
                power_array[0] = ct_power_accum[0] / three_phase_broadcast_count;
                power_array[1] = ct_power_accum[1] / three_phase_broadcast_count;
                power_array[2] = ct_power_accum[2] / three_phase_broadcast_count;

                // ����ϵͳ�У������ߵ�CT��������Ϊ0
                if (!sys_param.is_three_phase)
                {
                    if (sys_param.ct1.status.connect_status != CT_STATUS_ONLINE)
                        power_array[0] = 0.0f;
                    if (sys_param.ct2.status.connect_status != CT_STATUS_ONLINE)
                        power_array[1] = 0.0f;
                    if (sys_param.ct3.status.connect_status != CT_STATUS_ONLINE)
                        power_array[2] = 0.0f;
                }
            }

            // �㲥����
            broadcast_three_phase_power(power_array);

            // ���ü��������ۼ���
            three_phase_broadcast_count = 0;
            ct_power_accum[0] = 0;
            ct_power_accum[1] = 0;
            ct_power_accum[2] = 0;
        }
    }
    else
    {
        // ����������״̬���㲥0����
        sys_param.ct1.power.ct_sub1g_boardcast_power_avg = 0.0f;
        sys_param.ct2.power.ct_sub1g_boardcast_power_avg = 0.0f;
        sys_param.ct3.power.ct_sub1g_boardcast_power_avg = 0.0f;

        // �ж��Ƿ���Ҫ�򿪼̵���(���ʷ�������ɺ�?2����)
        static uint8_t relay_slot = 0;
        static uint8_t broadcast_toggle = 0; // ����0=�㲥����, 1=�㲥�̵�������
        bool should_open = (sys_param.state == SYS_POWER_DIR_DETECT && sys_param.grid.phase_id.relay_opening_pending);
        should_open = true;
        if (broadcast_toggle == 0)
        {
            // �㲥0����
            power_array[0] = 0.0f;
            power_array[1] = 0.0f;
            power_array[2] = 0.0f;
            broadcast_three_phase_power(power_array);

            broadcast_toggle = 1; // �´ι㲥�̵�������
        }
        else
        {
            // �㲥�̵�������
            uint8_t attempts = 0;
            while (!sys_param.paired_inv_info[relay_slot].is_valid && attempts < INV_DEVICE_MAX_NUM)
            {
                relay_slot = (relay_slot + 1) % INV_DEVICE_MAX_NUM;
                attempts++;
            }

            if (attempts < INV_DEVICE_MAX_NUM)
            {
                uint32_t target_addr = sys_param.paired_inv_info[relay_slot].sub1g_addr;
                if (target_addr != 0)
                {
                    sub1g_send_set_power_switch(target_addr, should_open);
                }
            }

            relay_slot = (relay_slot + 1) % INV_DEVICE_MAX_NUM;
            broadcast_toggle = 0; // �´ι㲥����
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void broadcast_other_task(void)
 Input       : 无
 Output      : 无
 Description : 广播其他数据任务。
---------------------------------------------------------------------------*/
static void broadcast_other_task(void)
{
    // ���FFT��ز�����ǰ������?
    bool fft_conditions_met = (sys_param.state == SYS_NORMAL_RUN) && (sys_param.grid.phase_id.sequence_k > 0);

    // ��һ���ȼ�������Ƿ����4��ʶ����Ҫ��������ȷ��
    if (sys_param.fft_identify.final_confirm_pending &&
        sys_param.fft_identify.boardcast_interval == 0 &&
        fft_conditions_met)
    {
        sys_param.fft_identify.final_confirm_pending = false;

#ifdef FFT_DEBUG_PRINT
        printf("����������λ��Ϣ��΢��: CT%d\r\n", sys_param.fft_identify.identified_ct);
#endif

        // ������λ��Ϣ��΢��
        sub1g_send_set_inv_phase(sys_param.fft_identify.sub1g_addr, sys_param.fft_identify.identified_ct);

        // ���浽EEPROM
        uint8_t idx = find_inv_index_by_sub1g_addr(sys_param.fft_identify.sub1g_addr);
        if (idx < INV_DEVICE_MAX_NUM)
        {
            sys_param.paired_inv_info[idx].phase = sys_param.fft_identify.identified_ct;
            sys_param.paired_inv_info[idx].prop_changed = true;
            eeprom_update_device_phase(sys_param.fft_identify.sub1g_addr, sys_param.fft_identify.identified_ct);
        }

        // ʶ����ȫ��������������״̬
        sys_param.fft_identify.sub1g_addr = 0;
        sys_param.fft_identify.consecutive_success_count = 0;
        sys_param.fft_identify.last_identified_ct = 0;

        return; // ����������ɣ�ֱ�ӷ���?
    }

    // �ڶ����ȼ�����������ʶ������
    if (sys_param.fft_identify.resend_cmd &&
        sys_param.fft_identify.boardcast_interval == 0 &&
        fft_conditions_met)
    {
        sys_param.fft_identify.resend_cmd = false;

        uint16_t power = sys_param.fft_identify.power;

        // ���Ϳ�������ʶ������
        sub1g_send_enable_phase_identify(sys_param.fft_identify.sub1g_addr, 25, power, sys_param.fft_identify.interval_time);

        // �����������������ʶ��?
        sys_param.fft_identify.is_ffting = 1; // ��ʼ��һ��ʶ��

#ifdef FFT_DEBUG_PRINT
        printf("��������ʶ������: addr=0x%08X, power=%dW, interval=%d\r\n",
               sys_param.fft_identify.sub1g_addr,
               power,
               sys_param.fft_identify.interval_time);
        printf("�ȴ�2���ʼ�ɼ�?...\r\n");
#endif

        return; // ����������󷵻�?
    }

    // �������ȼ����ȴ�2���ڼ���ط�����?
    if (sys_param.fft_identify.retry_flag && fft_conditions_met)
    {
        sys_param.fft_identify.retry_flag = 0;

        uint16_t power = sys_param.fft_identify.power;
        sub1g_send_enable_phase_identify(sys_param.fft_identify.sub1g_addr, 25, power, sys_param.fft_identify.interval_time);

#ifdef FFT_DEBUG_PRINT
        printf("�ط�����ʶ������: addr=0x%08X, power=%dW\r\n", sys_param.fft_identify.sub1g_addr, power);
#endif
        return;
    }

    // �������ȼ������ڹ㲥����
    if (sys_param.date_broadcast_counter >= 20000)
    {
        sys_param.date_broadcast_counter = 0;

        // ��������ַ����Ƿ����?
        if (strlen(sys_param.time.date) < 10)
        {
            return; // ���ڸ�ʽ���󣬲��㲥
        }

        // �㲥����
        sub1g_send_broadcast_date(sys_param.time.date);

        // 40sһ�ι㲥�������߷����΢�����
        if (sys_param.is_three_phase)
        {
            static uint8_t timer_40s_cnt = 0;

            timer_40s_cnt++;
            if (timer_40s_cnt >= 2)
            {
                timer_40s_cnt = 0;
                sub1g_send_broadcast_phase_inv_count(sys_param.ct1.inv_device_cnt, sys_param.ct2.inv_device_cnt, sys_param.ct3.inv_device_cnt);
            }
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void clear_offline_inverter_data(uint8_t inv_idx)
 Input       : inv_idx - 逆变器索引
 Output      : 无
 Description : 清除离线逆变器数据。
---------------------------------------------------------------------------*/
static void clear_offline_inverter_data(uint8_t inv_idx)
{
    if (inv_idx >= INV_DEVICE_MAX_NUM)
        return;

    inv_device_t *inv = &sys_param.paired_inv_info[inv_idx];

    // �������״̬�͹�������?
    inv->work_state = 0;
    inv->grid_power = 0.0f;

    // ���PV����
    for (uint8_t pv_idx = 0; pv_idx < 4; pv_idx++)
    {
        inv->pv[pv_idx].state = 0;
        inv->pv[pv_idx].power = 0;
        inv->pv[pv_idx].voltage = 0.0f;
        inv->pv[pv_idx].current = 0.0f;
    }

    // ��հ汾�ţ�ʹ�豸��������ʱ�����汾�ϱ�?
    inv->sw_version[0] = '\0';
    inv->sub1g_version[0] = '\0';

    DEBUG_PRINTF("[Offline] Clear inv[%d] (0x%06X) data \r\n", inv_idx, inv->sub1g_addr);
}

static void cal_phase_inv_1s(void)
{
    // 36������͹����ۼӾ�̬����?
    static uint8_t power_calc_cnt = 0;
    static float ct1_power_sum = 0.0f;
    static float ct2_power_sum = 0.0f;
    static float ct3_power_sum = 0.0f;
    static uint8_t save_eep_intrval = 0;

    uint8_t ct1_inv_count = 0;  // CT1�෢�������?
    uint8_t ct2_inv_count = 0;  // CT2�෢�������?
    uint8_t ct3_inv_count = 0;  // CT3�෢�������?
    float ct1_inv_power = 0.0f; // CT1�๦��
    float ct2_inv_power = 0.0f; // CT2�๦��
    float ct3_inv_power = 0.0f; // CT3�๦��

    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].online_state == 2)
        {
            // OTA�ڼ䲻�ж������豸
            if (!g_ota_manager.disable_broadcast)
            {
                sys_param.paired_inv_info[i].offline_updata_ms++;
            }

            // ��������豸���������£����?1����δ�ϱ����ݣ���������
            if (sys_param.paired_inv_info[i].offline_updata_ms >= PAIRED_INV_ONLINE_TIMEOUT_S)
            {
                sys_param.paired_inv_info[i].offline_updata_ms = PAIRED_INV_ONLINE_TIMEOUT_S;
                if (sys_param.paired_inv_info[i].online_state == 2)
                {
                    sys_param.paired_inv_info[i].online_state = 1; // ����豸����?

                    clear_offline_inverter_data(i); // ��������豸�����ݻ���?

                    // ��������ѱ仯��Ŀ����Ϊ���ϱ���wifi
                    sys_param.paired_inv_info[i].prop_changed = true;
                }
            }
            else
            {
                // ͳ�����ڷ����΢������������ҹ��ʴ���1W��
                if (sys_param.paired_inv_info[i].grid_power > 1)
                {
                    if (sys_param.is_three_phase)
                    {
                        // ����ϵͳ��������λ(CT��)�ֱ�ͳ��
                        switch (sys_param.paired_inv_info[i].phase)
                        {
                        case 1: // CT1��
                            ct1_inv_count++;
                            ct1_inv_power += sys_param.paired_inv_info[i].grid_power;
                            break;
                        case 2: // CT2��
                            ct2_inv_count++;
                            ct2_inv_power += sys_param.paired_inv_info[i].grid_power;
                            break;
                        case 3: // CT3��
                            ct3_inv_count++;
                            ct3_inv_power += sys_param.paired_inv_info[i].grid_power;
                            break;
                        default: // phase == 0 (δʶ��)���ݲ�����
                            break;
                        }
                    }
                    else
                    {
                        // ����ϵͳ�����з����豸������A��
                        ct1_inv_count++;
                        ct1_inv_power += sys_param.paired_inv_info[i].grid_power;
                    }
                }
            }
        }
        else if (sys_param.paired_inv_info[i].is_valid == 0)
        {
            sys_param.paired_inv_info[i].online_state = 0; // û����Ե�INV�豸
        }

        // ��鲢ɾ����ʱ��δ����豸(����10��δ�յ��㲥)
        if (sys_param.inv_request_pair_list[i].is_valid)
        {
            // ����Ƿ��?(10�� = 10000ms)
            if (sys_param.inv_request_pair_list[i].unpaired_updata_ms >= UNPAIRED_DEVICE_TIMEOUT_MS)
            {
                // �����?
                sys_param.inv_request_pair_list[i].is_valid = false;
                sys_param.inv_request_pair_list[i].sub1g_addr = 0;
                sys_param.inv_request_pair_list[i].unpaired_updata_ms = 0;
                sys_param.paired_inv_info[i].grid_power = 0.0f;
            }
        }
    }

    // ���¸������ڷ����΢�����������
    sys_param.ct1.inv_device_cnt = ct1_inv_count;
    sys_param.ct2.inv_device_cnt = ct2_inv_count;
    sys_param.ct3.inv_device_cnt = ct3_inv_count;

    // CT1��CT2��CT3΢�淢�繦��
    sys_param.ct1.inv_power = ct1_inv_power;
    sys_param.ct2.inv_power = ct2_inv_power;
    sys_param.ct3.inv_power = ct3_inv_power;

    // CT1��CT2��CT3�ฺ�ع���
    sys_param.ct1.use_power = ct1_inv_power + sys_param.ct1.power.fix_dir_power;
    sys_param.ct2.use_power = ct2_inv_power + sys_param.ct2.power.fix_dir_power;
    sys_param.ct3.use_power = ct3_inv_power + sys_param.ct3.power.fix_dir_power;

    // �ۼ�ÿ�빦��ֵ
    ct1_power_sum += sys_param.ct1.power.fix_dir_power;
    ct2_power_sum += sys_param.ct2.power.fix_dir_power;
    ct3_power_sum += sys_param.ct3.power.fix_dir_power;
    power_calc_cnt++;

    // ÿ36�����һ�η�����?
    if (power_calc_cnt >= 36)
    {
        // 36��ķ�����?(Wh) = (�����ۼ�ֵ / 36) �� (36/3600)
        sys_param.ct1.power_consumption = ct1_power_sum / 3600.0f;
        sys_param.ct2.power_consumption = ct2_power_sum / 3600.0f;
        sys_param.ct3.power_consumption = ct3_power_sum / 3600.0f;

        sys_param.hmi.electricity_consumption = (uint32_t)(sys_param.hmi.electricity_consumption + sys_param.ct1.power_consumption + sys_param.ct2.power_consumption + sys_param.ct3.power_consumption);

        save_eep_intrval++;
        if (save_eep_intrval >= 10) // 360�� = 6����
        {
            save_eep_intrval = 0;

            static uint32_t last_saved_consumption = 0;
            if (sys_param.hmi.electricity_consumption != last_saved_consumption)
            {
                int ret = eeprom_save_elec_consumption();
                if (ret == 0)
                {
                    last_saved_consumption = sys_param.hmi.electricity_consumption;
                    DEBUG_PRINTF("EEPROM: Consumption saved: %u Wh\n", sys_param.hmi.electricity_consumption);
                }
                else
                {
                    DEBUG_PRINTF("EEPROM: Consumption save failed!\n");
                }
            }
        }

        // ���ü��������ۼ�ֵ
        power_calc_cnt = 0;
        ct1_power_sum = 0;
        ct2_power_sum = 0;
        ct3_power_sum = 0;
    }

    // ���ݹ���ģʽ��������״̬����
    float total_grid_power = sys_param.ct1.power.fix_dir_power + sys_param.ct2.power.fix_dir_power + sys_param.ct3.power.fix_dir_power;

    switch (sys_param.power_work_mode)
    {
    case 1:                                 // ����������ģʽ
        sys_param.anti_backflow_switch = 1; // ����������
        if (total_grid_power < (-30))       // 3��������-30W
        {
            sys_param.limit_state = 2; // ����ʧ��
        }
        else
        {
            sys_param.limit_state = 1; // ������
        }
        break;

    case 2:                                 // �޹��ʷ���ģʽ
        sys_param.anti_backflow_switch = 1; // ����������
        if (total_grid_power < -(sys_param.to_grid_power_limit))
        {
            sys_param.limit_state = 2; // ����ʧ��
        }
        else if (total_grid_power < -(sys_param.to_grid_power_limit) * 0.8f)
        {
            sys_param.limit_state = 1; // ������
        }
        else
        {
            sys_param.limit_state = 0; // ��������
        }
        break;

    case 3:                                 // ���ɷ���ģʽ
        sys_param.anti_backflow_switch = 0; // �رշ�����
        sys_param.limit_state = 0;          // ���ɷ�����
        break;

    default:
        sys_param.anti_backflow_switch = 1;
        sys_param.limit_state = 0;
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : void inv_comm_stats_1s_task(void)
 Input       : 无
 Output      : 无
 Description : 逆变器通讯统计任务，执行周期1秒。
---------------------------------------------------------------------------*/
static void inv_comm_stats_1s_task(void)
{
    // ���������?
    uint8_t bound_inv_count = 0;
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid)
        {
            bound_inv_count++;
        }
    }

    // ÿ120�루2���ӣ�����һ�ζ����ʺ�ƽ��RSSI
    if (bound_inv_count == 0)
    {
        return;
    }

    // 60��ͳ�ƴ�������������: 60000ms / 40ms = 1500��
    // N̨΢��ʱÿ̨�������� 1500/N ��
    // 10%�ݲ�: ʵ�հ��� >= �������� * 90% ����Ϊ100%����
    uint16_t expected_packets_per_inv = 1500 / bound_inv_count;
    uint16_t expected_with_tolerance = expected_packets_per_inv * 90 / 100;

    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        inv_device_t *inv = &sys_param.paired_inv_info[i];

        if (!inv->is_valid)
        {
            continue;
        }

        inv->stats_time_sec++;

        // ����60���ڵ�ͳ��
        if (inv->stats_time_sec >= 60)
        {
            uint16_t total_rx = inv->rx_0x50_count + inv->rx_0x52_count + inv->rx_0x54_count +
                                inv->rx_0x55_count + inv->rx_0x56_count + inv->rx_0x57_count;

            // ���㶪������
            if (total_rx >= expected_with_tolerance)
            {
                inv->plr = 0;
            }
            else
            {
                inv->plr = (uint8_t)(((expected_with_tolerance - total_rx) * 100) / expected_with_tolerance);
                if (inv->plr > 100)
                {
                    inv->plr = 100;
                }
            }

            DEBUG_PRINTF("device [%06X], 0x50:%d, 0x52:%d, 0x54:%d, 0x55:%d, 0x56:%d, 0x57:%d, Total=%d, Expected=%d, Plr=%d\r\n",
                         inv->sub1g_addr, inv->rx_0x50_count, inv->rx_0x52_count,
                         inv->rx_0x54_count, inv->rx_0x55_count, inv->rx_0x56_count, inv->rx_0x57_count,
                         total_rx, expected_packets_per_inv, inv->plr);

            // ������Զ�����?
            inv->stats_time_sec = 0;
            inv->rx_0x50_count = 0;
            inv->rx_0x52_count = 0;
            inv->rx_0x54_count = 0;
            inv->rx_0x55_count = 0;
            inv->rx_0x56_count = 0;
            inv->rx_0x57_count = 0;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void param_update_1s_task(void)
 Input       : 无
 Output      : 无
 Description : 参数更新任务，执行周期1秒。
---------------------------------------------------------------------------*/
static void param_update_1s_task(void)
{
    if (sys_param.flags.timer_1s_flag)
    {
        sys_param.flags.timer_1s_flag = 0;

        // ΢��ͨ��ͳ������
        inv_comm_stats_1s_task();

        // ���¸������ڷ����΢����������ʡ�������
        cal_phase_inv_1s();

        // ����hmi����
        hmi_update_all_params();

        // �ȼ��Ƶ���Ƿ��й���?
        if (sys_param.fault.bit.grid_frequency)
        {
            DEBUG_PRINTF("[State Machine] Grid frequency fault detected, SYS_FREQ_FAULT.\r\n");
        }

        // static uint8_t printf_intreval = 0;
        // printf_intreval++;
        // if (printf_intreval >= 4)
        // {
        //     printf_intreval = 0;
        //     // ��ӡCT��Чֵ�Լ�����״̬
        //     printf("CT1_Rms:%f ����:%d ����:%.2f, CT2_Rms:%f ����:%d ����:%.2f, CT3_Rms:%f ����:%d ����:%.2f�� ����ϵͳ:%d\r\n", sys_param.ct1.rms_value, sys_param.ct1.status.connect_status, sys_param.ct1.power.fix_dir_power, sys_param.ct2.rms_value, sys_param.ct2.status.connect_status, sys_param.ct2.power.fix_dir_power, sys_param.ct3.rms_value, sys_param.ct3.status.connect_status, sys_param.ct3.power.fix_dir_power, sys_param.is_three_phase);
        // }
        // ��ӡ����㲥�����Լ��Ƿ�������ģ�?
        // DEBUG_PRINTF("�㲥����:%.2f, %.2f, %.2f, ����ϵͳ:%d, to_grid=%d\r\n", sys_param.ct1.power.ct_sub1g_boardcast_power_avg, sys_param.ct2.power.ct_sub1g_boardcast_power_avg, sys_param.ct3.power.ct_sub1g_boardcast_power_avg, sys_param.is_three_phase, sys_param.to_grid_power_limit);

#ifdef FFT_DEBUG_PRINT
        if (sys_param.fft_identify.enable_collect == 1)
        {
            printf("����FFT�ɼ�:\r\n");
        }
#endif
    }
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_timer_task(void)
 Input       : 无
 Output      : 无
 Description : Sub1G定时器任务。上电3秒内获取，之后每2秒获取一次RSSI。
---------------------------------------------------------------------------*/
static void sub1g_timer_task(void)
{
    // ÿ3�뷢��0x41��ȡ�汾��Ϣ��ֱ���յ��汾�ظ�
    if (sys_param.sub1g.sw_version[0] == '\0')
    {
        if (sys_param.sub1g.version_timer_ms >= 3000)
        {
            sub1g_send_get_version();
            sys_param.sub1g.version_timer_ms = 0;
        }
    }

    // ÿ10�뷢��0x42��ȡRSSI
    if (sys_param.sub1g.rssi_timer_ms >= 10000)
    {
        sub1g_send_get_rssi();
        sys_param.sub1g.rssi_timer_ms = 0;
    }
}
