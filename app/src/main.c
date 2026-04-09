//
// Included Files
//

#include "arm_math.h"
#include "board.h"
#include "main.h"
#include "mmi.h"
#include "grid.h"
#include "wifi.h"
#include "sub1g.h"
#include "debug.h"
#include "eeprom.h"
#include "ota_max.h"
#include "fft.h"
//
// Globals
//
sys_param_t sys_param;
wifi_info_t wifi_info;

// 电压缓冲区（A相直接采样，B/C相通过相位延时从 last_ua 重构）
float ua_voltage_buffer[TOTAL_SAMPLES];
float last_ua_voltage_buffer[TOTAL_SAMPLES];

// 电流缓冲区
float current1_buffer[TOTAL_SAMPLES];
float current2_buffer[TOTAL_SAMPLES];
float current3_buffer[TOTAL_SAMPLES];

volatile uint8_t phase_identify_timer_100ms = 0; // 相序识别100ms定时标志
uint8_t buffer_filled = 0;

// 内部索引变量
static uint16_t buffer_index = 0;

/*---------------------------------------------------------------------------
 Name        : uint16_t get_voltage_buffer_index(void)
 Input       : 无
 Output      : 当前环形缓冲写指针
 Description : 供 grid.c 中 phase_matching_calculation 获取当前缓冲写指针，
               用于计算电流环形起点，与 last_ua 快照时间窗对齐。
---------------------------------------------------------------------------*/
uint16_t get_voltage_buffer_index(void)
{
    return buffer_index;
}

// 计算起点快照（ct_task 入口处锁定，避免中断推进 buffer_index 导致 RMS/功率窗口不一致）
static uint16_t s_calc_buf_snap = 0;

/*---------------------------------------------------------------------------
 Name        : uint16_t get_calc_buf_snap(void)
 Input       : 无
 Output      : ct_task 入口快照的 buffer_index
 Description : 供 grid.c 中 phase_identify_process / phase_matching_calculation
               获取与 last_ua 拷贝时完全一致的缓冲快照起点，确保电压窗口与
               电流窗口严格对齐，避免 buffer_index 持续推进带来的相位误差。
---------------------------------------------------------------------------*/
uint16_t get_calc_buf_snap(void)
{
    return s_calc_buf_snap;
}

// 静态全局变量：3个CT的累加功率和计数
static float ct_power_accum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t three_phase_broadcast_count = 0;

// 函数声明（内部函数）
static void fault_detection_task(void);
static void ct_power_calculate_task(void);
static void copy_ua_ring_to_last_ua_linear(uint16_t spc, uint16_t snap_idx);

static void boardcast_power_task(void);
static void broadcast_other_task(void);
static void param_update_1s_task(void);
static void sub1g_timer_task(void);
// static void state_machine_partial_reset(void);

/*---------------------------------------------------------------------------
 Name        : void main(void)
 Input       : No
 Output      : No
 Description : 主函数入口。执行设备初始化、GPIO配置、中断配置以及。
---------------------------------------------------------------------------*/
int main(void)
{
    //
    // SysConfig settings
    //
    board_init();
    boot_logo_print();

    // 初始化系统参数
    system_param_init();

    // 初始化三通道FFT模块
    fft_3ch_init();

    // run_eeprom_tests();
    int ret = eeprom_init_and_load_devices();
    if (ret == 0)
    {
        print_device_list(); // 显示所有设备

        // 上电时，如果微逆已识别相位，发送0x22告知微逆所在相
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
        // 检查并处理故障检测任务 - 50us的ADC中断中赋标志
        if (sys_param.flags.task.fault_check_ready)
        {
            fault_detection_task();
            sys_param.flags.task.fault_check_ready = 0;
        }

        // 检查并处理状态机任务 - 50us的ADC中断中赋标志
        if (sys_param.flags.task.state_machine_ready)
        {
            system_state_machine(&sys_param.grid, &sys_param.ct1, &sys_param.ct2, &sys_param.ct3);
            sys_param.flags.task.state_machine_ready = 0;
        }

        // 计算AC电网的电压有效值和频率
        grid_task();

        // 检查并处理LED更新任务
        mmi_task();

        // 检查并处理调试发送任务
        debug_task();

        // 检查并处理SN命令
        debug_sn_task();

        // 执行FFT分析
        fft_check_and_analyze();

        // CT计算有效值以及检测是否钳在线缆上，并计算3个CT上的功率
        ct_task();

        // 识别微逆在哪个CT上
        inv_phase_detect_fix_direction_task();

        // 广播三相/单相功率
        boardcast_power_task();

        // 检查是否需要FFT采样分析，并每10s广播一次今日日期
        broadcast_other_task();

        // UART1发送队列处理
        uart1_tx_queue_process();

        // 参数1s更新
        param_update_1s_task();

        // sub1g数据接收处理
        sub1g_rx_task();

        // sub1g定时器任务(上电3秒获取版本,每2秒获取RSSI)
        sub1g_timer_task();

        // 调用OTA任务（1ms周期）
        ota_manager_task();

        // UART1发送队列处理
        uart1_tx_queue_process();
    }
}

/*---------------------------------------------------------------------------
 Name        : void voltage_and_current_buffer_record(void)
 Input       : 无
 Output      : 无
 Description : 电压电流环形缓冲区填充函数，在ADC中断（50us）中调用。
               中断内仅做数据写入和环形索引管理，不执行任何乘除运算和 memcpy，
               不置位任何计算标志位。
               所有参数更新（samples_per_cycle、相位延迟、频率合法判断、
               rms_calc_ready、power_calc_ready）均在同一中断内的 zero_cross_detect() 完成。
               buffer_index 为真正的环形指针，绕回 TOTAL_SAMPLES 而非每周过零点清零。
               若缓冲绕满一圈期间未出现正向过零（频率过低），则置频率故障。
---------------------------------------------------------------------------*/
void voltage_and_current_buffer_record(void)
{
    // 写入环形缓冲（buffer_index 已保证在 [0, TOTAL_SAMPLES-1]，无需越界保护）
    ua_voltage_buffer[buffer_index] = sys_param.signal.ac_voltage_LPF;
    current1_buffer[buffer_index] = sys_param.signal.ct1_current_LPF;
    current2_buffer[buffer_index] = sys_param.signal.ct2_current_LPF;
    current3_buffer[buffer_index] = sys_param.signal.ct3_current_LPF;

    // 本圈是否已见到正向过零，用于绕回时检测低频故障
    static uint8_t s_zero_crossed_since_wrap = 0;
    if (sys_param.grid.zero_cross.positive_zero_cross)
    {
        s_zero_crossed_since_wrap = 1;
    }

    buffer_index++;

    // 环形绕回
    if (buffer_index >= TOTAL_SAMPLES)
    {
        buffer_index = 0;
        buffer_filled = 1;

        // 绕满一圈仍未出现正向过零：频率过低（< 45Hz），置频率故障
        if (!s_zero_crossed_since_wrap)
        {
            sys_param.fault.bit.grid_frequency = 1;
        }
        s_zero_crossed_since_wrap = 0; // 为下一圈复位
    }
}

/*---------------------------------------------------------------------------
 Name        : void system_state_machine(...)
 Input       : grid_mgr - 电网管理器
               ct1, ct2, ct3 - 三个CT参数
 Output      : 无
 Description : 电网状态机主函数（在while(1)中调用）
---------------------------------------------------------------------------*/
void system_state_machine(grid_manager_t *grid_mgr, ct_param_t *ct1, ct_param_t *ct2, ct_param_t *ct3)
{

    if (sys_param.restore_sys)
    {
        __NVIC_SystemReset();
    }

    // // 系统三/单相类型变化检测
    // if (grid_mgr->system_type_changed)
    // {
    //     // 系统类型变化,重新初始化
    //     DEBUG_PRINTF("[State Machine] System type changed, re-initializing...\r\n");

    //     sys_param.state = SYS_INIT;

    //     // 重置相序识别参数
    //     phase_identify_init(&sys_param.grid.phase_id);

    //     // 重置功率方向检测
    //     ct_power_direction_detect_init(&sys_param.ct1);
    //     ct_power_direction_detect_init(&sys_param.ct2);
    //     ct_power_direction_detect_init(&sys_param.ct3);

    //     grid_mgr->system_type_changed = false;
    //     return;
    // }

    // 状态机逻辑 - 只根据标志位做状态切换
    switch (sys_param.state)
    {
    case SYS_INIT: // Case 0: 检测AC电压过零和电网频率

        // 先检查频率是否有故障
        if (sys_param.fault.bit.grid_frequency)
        {
            sys_param.state = SYS_FREQ_FAULT;
            break;
        }

        if (grid_mgr->zero_cross.zero_cross_count >= ZERO_CROSS_COUNT_TARGET) // 检测到足够的过零次数且无故障，状态转换
        {
            sys_param.state = SYS_WAIT_CT;
        }
        break;

    case SYS_WAIT_CT: // Case 1: 等待CT插好

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

    case SYS_PHASE_IDENTIFY: // Case 2: 相序识别

        if (grid_mgr->phase_id.identification_valid) // 相序识别完成，直接进入正常运行
        {
            // 固定功率方向为正向，无需方向检测
            sys_param.ct1.power.direction_detect_complete = 1;
            sys_param.ct2.power.direction_detect_complete = 1;
            sys_param.ct3.power.direction_detect_complete = 1;

            // // 自动识别结果保存到EEPROM（本次运行有效，重启后若tag不匹配则回归默认1）
            // eeprom_save_set_param();

            printf("[State Machine] Auto phase identify done. CT Mapping: CT1->Phase %c, CT2->Phase %c, CT3->Phase %c\r\n",
                   'A' + grid_mgr->phase_id.ct_to_phase[0],
                   'A' + grid_mgr->phase_id.ct_to_phase[1],
                   'A' + grid_mgr->phase_id.ct_to_phase[2]);

            sys_param.state = SYS_NORMAL_RUN;
        }
        // else if (!grid_mgr->ct_connected) // CT断开，返回初始状态
        // {
        //     DEBUG_PRINTF("[State Machine] Ct Not Connected.\r\n");
        // }
        break;

    case SYS_POWER_DIR_DETECT: // Case 3: 功率方向检测

        // 检查三个CT的功率方向是否都已检测完成
        if (sys_param.ct1.power.direction_detect_complete &&
            sys_param.ct2.power.direction_detect_complete &&
            sys_param.ct3.power.direction_detect_complete)
        {
            if (!grid_mgr->phase_id.relay_opening_pending)
            {
                // 开始2秒继电器打开过程
                grid_mgr->phase_id.relay_opening_pending = 1;
                grid_mgr->phase_id.relay_open_timer_ms = 0;
            }
            else if (grid_mgr->phase_id.relay_open_timer_ms >= 2000)
            {
                // 2秒继电器打开过程完成，进入正常运行状态
                grid_mgr->phase_id.relay_opening_pending = 0;
                grid_mgr->phase_id.relay_open_timer_ms = 0;
                sys_param.state = SYS_NORMAL_RUN;
                printf("[State Machine] Power direction detection complete, entering SYS_NORMAL_RUN.\r\n");
            }
        }
        // else if (!grid_mgr->ct_connected) // CT断开，返回初始状态
        // {
        //     sys_param.state = SYS_INIT;
        //     state_machine_partial_reset(); // 部分重置参数
        //     grid_mgr->phase_id.identification_valid = 0;
        // }
        break;

    case SYS_NORMAL_RUN: // Case 4: 正常运行

        // if (!grid_mgr->ct_connected)
        // {
        //     // 检测到CT未插入，返回等待状态
        //     sys_param.state = SYS_INIT;
        //     state_machine_partial_reset(); // 部分重置参数
        //     grid_mgr->phase_id.identification_valid = 0;
        // }

        break;

    case SYS_FREQ_FAULT: // Case 5: 电网频率故障（超出45Hz-65Hz范围）
        // 频率故障由中断内 zero_cross_detect() 检测：合法时清除 fault.bit.grid_frequency
        // 此处连续检测故障位，恢复后回到 SYS_INIT 重新初始化
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
 Description : 功率方向检测和FFT数据采集任务
               - SYS_POWER_DIR_DETECT状态：执行功率方向检测
               - SYS_NORMAL_RUN状态：执行FFT数据采集
---------------------------------------------------------------------------*/
void inv_phase_detect_fix_direction_task(void)
{
    // 预先计算FFT采集条件，避免在每个CT处理中重复判断
    bool fft_collect_enabled = (sys_param.state == SYS_NORMAL_RUN) && (sys_param.grid.phase_id.sequence_k > 0) && (sys_param.fft_identify.enable_collect == 1);

    // CT1功率处理
    if (sys_param.ct1.power.power_ready)
    {
        // 在功率方向检测状态下进行功率方向检测
        if (sys_param.state == SYS_POWER_DIR_DETECT)
        {
            ct_power_direction_detect_process(&sys_param.ct1);
        }

        sys_param.ct1.power.power_ready = 0;

        // FFT数据采集（仅在满足前置条件时）
        if (fft_collect_enabled)
        {
            fft_collect_power_data_3ch(CT_CHANNEL_1, sys_param.ct1.power.fix_dir_power);
        }
    }

    // CT2功率处理
    if (sys_param.ct2.power.power_ready)
    {
        // 在功率方向检测状态下进行功率方向检测
        if (sys_param.state == SYS_POWER_DIR_DETECT)
        {
            ct_power_direction_detect_process(&sys_param.ct2);
        }

        sys_param.ct2.power.power_ready = 0;

        // FFT数据采集（仅在满足前置条件时）
        if (fft_collect_enabled)
        {
            fft_collect_power_data_3ch(CT_CHANNEL_2, sys_param.ct2.power.fix_dir_power);
        }
    }

    //  CT3功率处理
    if (sys_param.ct3.power.power_ready)
    {
        // 在功率方向检测状态下进行功率方向检测
        if (sys_param.state == SYS_POWER_DIR_DETECT)
        {
            ct_power_direction_detect_process(&sys_param.ct3);
        }

        sys_param.ct3.power.power_ready = 0;

        // FFT数据采集（仅在满足前置条件时）
        if (fft_collect_enabled)
        {
            fft_collect_power_data_3ch(CT_CHANNEL_3, sys_param.ct3.power.fix_dir_power);
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void ct_task(void)
 Input       : 无
 Output      : 无
 Description : CT任务处理函数。
               入口处快照 buffer_index，避免中断在运算过程中推进指针导致窗口漂移。
               先 copy ua 环形快照到 last_ua 线性缓冲，再计算 RMS，再计算功率和 PF，
               确保三者使用相同的时间窗，且 PF = P/(V_rms*I_rms) 使用本周期 RMS。
---------------------------------------------------------------------------*/
void ct_task(void)
{
    // 若本周期有 RMS 或功率计算任务，入口先锁定 buffer_index 快照
    if (sys_param.flags.rms_calc_ready || sys_param.flags.task.power_calc_ready)
    {
        s_calc_buf_snap = buffer_index; // 快照，后续计算均基于此起点
        uint16_t spc = sys_param.grid.samples_per_cycle;
        if (spc > 0 && spc <= TOTAL_SAMPLES && sys_param.grid.zero_cross.frequency_valid)
        {
            // 将 ua 环形缓冲最近 spc 点线性展开到 last_ua[0..spc-1]
            copy_ua_ring_to_last_ua_linear(spc, s_calc_buf_snap);
        }
    }

    // 1. 先计算 RMS（与功率触发同周期，先算 RMS 保证 PF 使用最新值）
    if (sys_param.flags.rms_calc_ready)
    {
        ct_rms_calculate();
        sys_param.flags.rms_calc_ready = 0;

        // 判断是否钳在电线上
        ct_online_detect_process(&sys_param.ct1, sys_param.ct1.rms_value);
        ct_online_detect_process(&sys_param.ct2, sys_param.ct2.rms_value);
        ct_online_detect_process(&sys_param.ct3, sys_param.ct3.rms_value);

        // 统计在线CT数量
        sys_param.grid.online_ct_count = 0;

        if (sys_param.ct1.status.connect_status == CT_STATUS_ONLINE)
            sys_param.grid.online_ct_count++;
        if (sys_param.ct2.status.connect_status == CT_STATUS_ONLINE)
            sys_param.grid.online_ct_count++;
        if (sys_param.ct3.status.connect_status == CT_STATUS_ONLINE)
            sys_param.grid.online_ct_count++;

        // 更新CT连接标志
        sys_param.grid.ct_connected = (sys_param.grid.online_ct_count > 0);

        // 判断系统类型（三相/单相）
        static bool last_is_three_phase = false;
        bool current_is_three_phase;

        // 单相可以变成三相，三相识别不可变为单相
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
            current_is_three_phase = sys_param.is_three_phase; // 保持原有状态
        }

        // 更新系统类型
        if (sys_param.grid.online_ct_count > 0)
        {
            sys_param.is_three_phase = current_is_three_phase;
            last_is_three_phase = current_is_three_phase;
        }
    }

    // 2. 再计算功率和 PF（依赖上面刚更新的 rms_value）
    if (sys_param.flags.task.power_calc_ready)
    {
        // 三相功率及功率因数计算任务
        ct_power_calculate_task();

        // 相序识别在此处调用：last_ua快照、s_calc_buf_snap、RMS均已在本轮准备就绪
        phase_identify_process(&sys_param.grid.phase_id);

        sys_param.flags.task.power_calc_ready = 0;

        // 每个电网周期功率计算完成后，置位广播触发标志
        sys_param.flags.task.power_cycle_ready = 1;
    }
}

/*---------------------------------------------------------------------------
 Name        : void adc_sample_and_process(void)
 Input       : 无
 Output      : 无
 Description : ADC采样和信号处理函数
---------------------------------------------------------------------------*/
void adc_sample_and_process(void)
{
    // ==========================原始信号采样==========================
    sys_param.signal.adc1_raw[0] = ADC_GetValue(CM_ADC1, ADC_CH0); // I_CT1
    sys_param.signal.adc1_raw[1] = ADC_GetValue(CM_ADC1, ADC_CH1); // I_CT2
    sys_param.signal.adc1_raw[2] = ADC_GetValue(CM_ADC1, ADC_CH2); // I_CT3
    sys_param.signal.adc1_raw[3] = ADC_GetValue(CM_ADC1, ADC_CH3); // V_AC
    sys_param.signal.adc1_raw[4] = ADC_GetValue(CM_ADC1, ADC_CH4); // V_1.65V

    // ==========================低通滤波==========================
    sys_param.signal.adc1_raw_LPF[0] = KLPF_Function_Float(sys_param.signal.adc1_raw[0], 0.3f, 0); // I_CT1滤波值
    sys_param.signal.adc1_raw_LPF[1] = KLPF_Function_Float(sys_param.signal.adc1_raw[1], 0.3f, 1); // I_CT2滤波值
    sys_param.signal.adc1_raw_LPF[2] = KLPF_Function_Float(sys_param.signal.adc1_raw[2], 0.3f, 2); // I_CT3滤波值
    sys_param.signal.adc1_raw_LPF[3] = KLPF_Function_Float(sys_param.signal.adc1_raw[3], 0.3f, 3); // V_AC滤波值
    sys_param.signal.adc1_raw_LPF[4] = KLPF_Function_Float(sys_param.signal.adc1_raw[4], 0.3f, 4); // V_1.65V滤波值

    // ==========================数据处理==========================
    // 交流电压转换：转换系数 ADC/4096*3300mV*0.2667(V/mV)
    sys_param.signal.ac_voltage = (float)((int)sys_param.signal.adc1_raw[3] - (int)sys_param.signal.adc1_raw[4]) * 0.2149f;

    // 三路电流互感器转换：转换系数 ADC/4096*3300mV*0.025(A/mV)
    sys_param.signal.ct1_current = (float)((int)sys_param.signal.adc1_raw[0] - (int)sys_param.signal.adc1_raw[4]) * 0.0201416f;
    sys_param.signal.ct2_current = (float)((int)sys_param.signal.adc1_raw[1] - (int)sys_param.signal.adc1_raw[4]) * 0.0201416f;
    sys_param.signal.ct3_current = (float)((int)sys_param.signal.adc1_raw[2] - (int)sys_param.signal.adc1_raw[4]) * 0.0201416f;

    // 1.65V参考电压转换：ADC/4096*3.3V
    sys_param.signal.v1p65_voltage = (float)sys_param.signal.adc1_raw[4] * 0.000806f;

    // ==========================滤波后的数据处理==========================
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
 Description : 有效值计算，使用 s_calc_buf_snap 作为环形缓冲起点，
               取最近 spc 个样本计算 RMS，支持50Hz/60Hz自适应。
---------------------------------------------------------------------------*/
void ct_rms_calculate(void)
{
    uint16_t spc = sys_param.grid.samples_per_cycle;
    uint16_t start = (uint16_t)((s_calc_buf_snap + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES);

    sys_param.grid.ua_vol_rms = calculate_rms_ring(ua_voltage_buffer, TOTAL_SAMPLES, start, spc);
    sys_param.ct1.rms_value = calculate_rms_ring(current1_buffer, TOTAL_SAMPLES, start, spc);
    sys_param.ct2.rms_value = calculate_rms_ring(current2_buffer, TOTAL_SAMPLES, start, spc);
    sys_param.ct3.rms_value = calculate_rms_ring(current3_buffer, TOTAL_SAMPLES, start, spc);
}

/*---------------------------------------------------------------------------
 Name        : void set_task_flags_from_interrupt(void)
 Input       : 无
 Output      : 无
 Description : 从中断设置任务标志位，让主循环处理复杂业务逻辑
---------------------------------------------------------------------------*/
void set_task_flags_from_interrupt(void)
{
    // 每次ADC中断都需要检查故障
    sys_param.flags.task.fault_check_ready = 1;

    // 每次ADC中断都需要处理状态机
    sys_param.flags.task.state_machine_ready = 1;

    // 注意：ct_phase_identify_ready 已废弃，相序识别由 ct_task 内 power_calc_ready 触发
}

/*---------------------------------------------------------------------------
 Name        : static void copy_ua_ring_to_last_ua_linear(uint16_t spc, uint16_t snap_idx)
 Input       : spc      - 本周期采样点数
               snap_idx - ct_task 入口快照的 buffer_index
 Output      : 无
 Description : 将 ua_voltage_buffer 环形缓冲中最近 spc 个样本线性展开拷贝到
               last_ua_voltage_buffer[0..spc-1]，供 B/C 相功率/相序计算使用。
               起点 = (snap_idx + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES
---------------------------------------------------------------------------*/
static void copy_ua_ring_to_last_ua_linear(uint16_t spc, uint16_t snap_idx)
{
    if (spc == 0 || spc > TOTAL_SAMPLES)
    {
        return;
    }
    uint16_t start = (uint16_t)((snap_idx + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES);
    for (uint16_t i = 0; i < spc; i++)
    {
        last_ua_voltage_buffer[i] = ua_voltage_buffer[(start + i) % TOTAL_SAMPLES];
    }
}

/*---------------------------------------------------------------------------
 Name        : static void ct_power_calculate_task(void)
 Input       : 无
 Output      : 无
 Description : 三相功率及功率因数计算任务（在主循环中调用）。
               运行条件：功率方向检测或正常运行状态 + 相序识别有效 + 缓冲区已填充
                        + frequency_valid + !frequency_fault。
               使用 s_calc_buf_snap 确定环形电流缓冲起点。
               A/B/C 三相电压均从 last_ua_voltage_buffer[0..spc-1]（线性快照）取值，
               保证与电流的时间窗完全对齐，消除 buffer_index 推进引起的相位漂移。
               PF = avg_power / (ua_vol_rms * ct_rms_value)，计算结果限幅到[-1,1]。
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

    // 环形电流缓冲起点（与 RMS 使用同一快照）
    uint16_t curr_start = (uint16_t)((s_calc_buf_snap + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES);

    float sum1 = 0.0f, sum2 = 0.0f, sum3 = 0.0f;

    for (uint16_t i = 0; i < spc; i++)
    {
        // 三相电压均从 last_ua 线性快照取值，保证与电流时间窗对齐
        float va = last_ua_voltage_buffer[i];
        float vb = last_ua_voltage_buffer[(i + spc - pb) % spc];
        float vc = last_ua_voltage_buffer[(i + spc - pc) % spc];

        float phase_voltage[3];
        phase_voltage[0] = va;
        phase_voltage[1] = vb;
        phase_voltage[2] = vc;

        // 电流从环形缓冲按快照起点取值
        uint16_t ci = (curr_start + i) % TOTAL_SAMPLES;
        sum1 += phase_voltage[ct1_phase] * current1_buffer[ci];
        sum2 += phase_voltage[ct2_phase] * current2_buffer[ci];
        sum3 += phase_voltage[ct3_phase] * current3_buffer[ci];
    }

    // ---- 结算三路有功功率 ----
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

    // ---- 计算功率因数 PF = P / (V_rms * I_rms) ----
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

    // ---- 单相系统中，不在线的CT功率/PF清零 ----
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
 Description : 系统故障检测主函数
               检测交流电压、三路电流、参考电压是否存在故障
---------------------------------------------------------------------------*/
static void fault_detection_task(void)
{
    // 静态故障计数器
    static uint16_t ac_fault_count = 0;
    static uint16_t ct1_fault_count = 0;
    static uint16_t ct2_fault_count = 0;
    static uint16_t ct3_fault_count = 0;
    static uint16_t v1p65_fault_count = 0;

    // 交流电压故障检测（>380V 或 <176V）
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
        ac_fault_count = 0; // 清零计数器
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ac_sample = 0;
    }

    // CT1电流互感器故障检测（>60A）
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
        ct1_fault_count = 0; // 清零计数器
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ct1_sample = 0;
    }

    // CT2电流互感器故障检测（>60A）
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
        ct2_fault_count = 0; // 清零计数器
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ct2_sample = 0;
    }

    // CT3电流互感器故障检测（>60A）
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
        ct3_fault_count = 0; // 清零计数器
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.ct3_sample = 0;
    }

    // 1.65V参考电压故障检测（范围检测）
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
        v1p65_fault_count = 0; // 清零计数器
        if (sys_param.fault_delay > FAULT_DELAY_S)
            sys_param.fault.bit.v1p65_sample = 0;
    }

    // 更新故障结果
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
 Description : 系统定时器管理
---------------------------------------------------------------------------*/
void system_timer_management(void)
{
    sys_param.timer.timer_1ms_count++;

    // ============= 1ms级定时任务 =============
    if (sys_param.timer.timer_1ms_count >= TIMER_1MS_CYCLES) // 20 * 50us = 1ms
    {
        sys_param.timer.timer_1ms_count = 0; // 重置1ms计数器

        // sub1g定时器递增
        if (sys_param.sub1g.sw_version[0] == '\0') // 未收到版本，继续计时
        {
            sys_param.sub1g.version_timer_ms++;
        }
        sys_param.sub1g.rssi_timer_ms++;

        // ============= 20ms级定时任务 =============
        sys_param.timer.timer_20ms_count++;
        if (sys_param.timer.timer_20ms_count >= TIMER_20mS_CYCLES) // 1000ms = 1s
        {
            sys_param.timer.timer_20ms_count = 0;

            // 设置20ms标志
            sys_param.flags.timer_20ms_flag = 1;
        }

        // ============= 1s级定时任务 =============
        sys_param.timer.timer_1s_count++;
        if (sys_param.timer.timer_1s_count >= TIMER_1S_CYCLES) // 1000ms = 1s
        {
            sys_param.timer.timer_1s_count = 0;
            sys_param.fault_delay++; // 故障延时计数器递增

            static uint8_t count = 0;
            count++;
            if (count >= 10)
            {
                count = 0;
                sys_param.flags.timer_10s_flag = 1;
            }

            // 设置1s标志
            sys_param.flags.timer_1s_flag = 1;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void INT_ADC_1_1_ISR(void)
 Input       : 无
 Output      : 无
 Description : ADC采样中断服务程序，周期为50us。
---------------------------------------------------------------------------*/
void ADC1_Handler(void) // 50US一次中断
{
    // ADC采样和信号处理
    if (ADC_GetStatus(CM_ADC1, ADC_FLAG_EOCA) == SET)
    {
        ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);

        // GPIO_SetPins(GPIO_PORT_F, GPIO_PIN_02);

        adc_sample_and_process();

        // 填充电压电流数据缓存区
        voltage_and_current_buffer_record();

        // 过零检测：使用未滤波的原始电压，避免 LPF 引入相位滞后导致过零点计数偏少
        // 2样本连续同符号判断本身已有足够的抗噪能力，无需 LPF
        zero_cross_detect(&sys_param.grid.zero_cross, sys_param.signal.ac_voltage);

        // 系统定时器管理
        system_timer_management();

        // 设置任务标志位，让主循环处理复杂业务逻辑
        set_task_flags_from_interrupt();

        // GPIO_ResetPins(GPIO_PORT_F, GPIO_PIN_02);
    }
    __DSB(); /* Arm Errata 838869 */
}

/*---------------------------------------------------------------------------
 Name        : void SysTick_Handler(void)
 Input       : 无
 Output      : 无
 Description : 1ms系统滴答中断处理
---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    // 设置1ms定时标志
    sys_param.flags.timer_1ms_flag = 1;

    // 相序识别100ms计时
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

    // 继电器打开定时器递增
    if (sys_param.grid.phase_id.relay_opening_pending)
    {
        sys_param.grid.phase_id.relay_open_timer_ms++;
    }

    // ========== 未配对设备计数器更新(1ms) ==========
    // 遍历所有有效的未配对设备，递增其计数器
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
                // 从未配对列表中删除
                inv_request_pair_list_remove(sys_param.inv_request_pair_list[i].sub1g_addr);
            }
        }
        else
        {
            sys_param.inv_request_pair_list[i].paired_unvalid_ms = 0;
        }
    }

    // 功率广播和日期计数器更新
    sys_param.date_broadcast_counter++;

#ifdef DEBUG_ENABLE
    sys_param.timer.debug_1ms_count++;
#endif

    sys_param.mmi.led_count++;
    sys_param.mmi.display_timer_ms++;

    // Sub1G 通信超时检测
    if (sys_param.sub1g.state == 4) // 只有在通信正常状态才检测超时
    {
        sys_param.sub1g.timeout_count++;
        if (sys_param.sub1g.timeout_count >= 15000)
        {
            sys_param.sub1g.state = 3;
            sys_param.sub1g.timeout_count = 0;
        }
    }

    // 不在升级时候
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

    // FFT采集延迟控制：is_ffting=1后等待2秒才开始采集
    static uint16_t fft_delay_count = 0;
    if (sys_param.fft_identify.is_ffting == 1)
    {
        if (fft_delay_count < 2000)
        {
            fft_delay_count++;

            // FFT等待期间前1秒：每100ms置一次重发标志，由主循环负责实际发送
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
        fft_delay_count = 0; // is_ffting=0时重置计数
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
 Description : 系统参数上电初始化函数，将所有参数清零
---------------------------------------------------------------------------*/
void system_param_init(void)
{
    // 一次性清零整个结构体
    memset(&sys_param, 0, sizeof(sys_param_t));

    // 初始化系统标志位
    system_flags_init();

    // 初始化CT是否在线检测
    ct_online_detect_init(&sys_param.ct1);
    ct_online_detect_init(&sys_param.ct2);
    ct_online_detect_init(&sys_param.ct3);

    power_calc_init(&sys_param.ct1.power);
    power_calc_init(&sys_param.ct2.power);
    power_calc_init(&sys_param.ct3.power);

    grid_manager_init(); // 初始化电网检测参数

    // 默认相序sequence_k=1（CT1=A相，CT2=B相滞后120°，CT3=C相超前120°）
    sys_param.grid.phase_id.sequence_k = 1;
    sys_param.grid.phase_id.identification_valid = 1;
    update_ct_to_phase_mapping(1);

    // 功率方向固定为正向，无需执行方向检测流程
    sys_param.ct1.power.power_direction = 1;
    sys_param.ct1.power.direction_detect_complete = 1;
    sys_param.ct2.power.power_direction = 1;
    sys_param.ct2.power.direction_detect_complete = 1;
    sys_param.ct3.power.power_direction = 1;
    sys_param.ct3.power.direction_detect_complete = 1;

    ota_manager_init(); // 初始化OTA管理器

    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // 初始化微逆设备信息数组
        memset(sys_param.paired_inv_info[i].device_sn, 0, SN_LENGTH + 1);
        memset(sys_param.paired_inv_info[i].device_sn, 0, sizeof(sys_param.paired_inv_info[i].device_sn));
        sys_param.paired_inv_info[i].sub1g_addr = 0;
        sys_param.paired_inv_info[i].siid = 0;

        // 初始化未配对设备列表
        sys_param.inv_request_pair_list[i].is_valid = false;
        sys_param.inv_request_pair_list[i].sub1g_addr = 0;
        sys_param.inv_request_pair_list[i].unpaired_updata_ms = 0;
        sys_param.inv_request_pair_list[i].device_sn[0] = '\0';
        sys_param.inv_request_pair_list[i].product_model = 0;

        // 初始化用户配对列表
        sys_param.user_pair_list[i].is_valid = false;
        sys_param.user_pair_list[i].device_sn[0] = '\0';
    }

    sys_param.anti_backflow_switch = 1; // 默认开启防逆流功能

    // 默认是单相发电
    sys_param.is_three_phase = false;

    // 初始化 Sub1G 状态为未连接
    sys_param.sub1g.state = 1;         // 1 = 未配对设备
    sys_param.sub1g.timeout_count = 0; // 超时计数器清零
    sys_param.sub1g.reboot_count = 0;  // 通信超时重启计数器

    sys_param.sub1g.version_timer_ms = 0;
    sys_param.sub1g.rssi_timer_ms = 0;
    sys_param.sub1g.rssi = 0;
    sys_param.sub1g.ct_sub1g_addr = 0;
    sys_param.sub1g.sw_version[0] = '\0'; // 版本字符串初始化为空
    sys_param.sub1g.channel_index = 0xFF; // CT信道值初始化

    // 初始化slave版本管理
    sys_param.slave_version.inv_sub1g_version[0] = '\0';
    sys_param.slave_version.inv_800w_version[0] = '\0';
    sys_param.slave_version.inv_2500w_version[0] = '\0';
    sys_param.slave_version.slave_version_reported = false;

    // 微逆相序识别初始化
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
 Description : CT断开时的部分重置，只重置运行时参数，保留EEPROM加载的数据
               保留: paired_inv_info, electricity_consumption, power_work_mode,
                     to_grid_power_limit, anti_backflow_switch, sub1g地址和版本,
                     slave_version, user_pair_list等
---------------------------------------------------------------------------*/
void state_machine_partial_reset(void)
{
    // 重置CT在线检测参数
    ct_online_detect_init(&sys_param.ct1);
    ct_online_detect_init(&sys_param.ct2);
    ct_online_detect_init(&sys_param.ct3);

    // 重置功率计算参数
    power_calc_init(&sys_param.ct1.power);
    power_calc_init(&sys_param.ct2.power);
    power_calc_init(&sys_param.ct3.power);

    // 重置功率方向检测（保持正向，不清零）
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

    //  重置CT RMS值
    sys_param.ct1.rms_value = 0;
    sys_param.ct2.rms_value = 0;
    sys_param.ct3.rms_value = 0;

    // 重置过零检测参数（部分）
    sys_param.grid.zero_cross.zero_cross_count = ZERO_CROSS_COUNT_TARGET / 2; // 保留部分计数，加快恢复
    sys_param.grid.zero_cross.zero_cross_detected = 0;
    sys_param.grid.zero_cross.positive_zero_cross = 0;

    // 重置相序识别运行时参数（保留识别结果）
    memset(sys_param.grid.phase_id.matching_degree, 0, sizeof(sys_param.grid.phase_id.matching_degree));
    memset(sys_param.grid.phase_id.power_factor, 0, sizeof(sys_param.grid.phase_id.power_factor));
    memset(sys_param.grid.phase_id.identify_history, 0, sizeof(sys_param.grid.phase_id.identify_history));
    sys_param.grid.phase_id.identify_count = 0;
    sys_param.grid.phase_id.consistent_count = 0;

    // 重置FFT识别管理器
    sys_param.fft_identify.identified_ct = 0;
    sys_param.fft_identify.is_ffting = 0;
    sys_param.fft_identify.enable_collect = 0;
    sys_param.fft_identify.resend_cmd = false;
    sys_param.fft_identify.retry_flag = 0;
    sys_param.fft_identify.consecutive_success_count = 0;
    sys_param.fft_identify.last_identified_ct = 0;
    sys_param.fft_identify.boardcast_interval = 0;
    sys_param.fft_identify.final_confirm_pending = false;

    // 重置系统标志位
    sys_param.flags.task.fault_check_ready = 0;
    sys_param.flags.rms_calc_ready = 0;
    sys_param.flags.task.power_calc_ready = 0;
    sys_param.flags.task.ct_phase_identify_ready = 0;

    // 重置故障状态
    sys_param.fault.data = 0;
    sys_param.fault_result = 0;
    sys_param.fault_delay = 0;
}

/*---------------------------------------------------------------------------
 Name        : void ct_online_detect_process(ct_param_t *ct_param, float rms_value)
 Input       : ct_param - CT参数结构体指针
               rms_value - 当前RMS有效值
 Output      : 无
 Description : 处理CT在线检测逻辑
---------------------------------------------------------------------------*/
void ct_online_detect_process(ct_param_t *ct_param, float rms_value)
{
    if (ct_param == NULL)
        return;

    if (rms_value < CT_OFFLINE_THRESHOLD)
    {
        // RMS值低于离线阈值
        ct_param->status.offline_count++;
        ct_param->status.online_count = 0; // 重置计数

        // 检查是否达到离线判断条件
        if (ct_param->status.offline_count >= CT_OFFLINE_COUNT_THRESHOLD)
        {
            ct_param->status.offline_count = CT_OFFLINE_COUNT_THRESHOLD;
            if (ct_param->status.connect_status != CT_STATUS_OFFLINE)
            {
                ct_param->status.connect_status = CT_STATUS_OFFLINE; // 状态改变：从在线/未知 -> 离线
            }
        }
    }
    else if (rms_value > CT_ONLINE_THRESHOLD)
    {
        // RMS值高于在线阈值
        ct_param->status.online_count++;
        ct_param->status.offline_count = 0; // 重置离线计数

        // 检查是否达到在线判断条件
        if (ct_param->status.online_count >= CT_ONLINE_COUNT_THRESHOLD)
        {
            ct_param->status.online_count = CT_ONLINE_COUNT_THRESHOLD;
            if (ct_param->status.connect_status != CT_STATUS_ONLINE)
            {
                ct_param->status.connect_status = CT_STATUS_ONLINE; // 状态改变：从离线/未知 -> 在线
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
 Input       : ct - CT参数结构体指针
 Output      : 无
 Description : CT功率方向检测处理函数，收集50个power.avg_power样本后判断方向
---------------------------------------------------------------------------*/
void ct_power_direction_detect_process(ct_param_t *ct)
{
    if (ct == NULL)
        return;

    // 如果已经检测完成，直接返回
    if (ct->power.direction_detect_complete && ct->power.power_direction != 0)
    {
        return;
    }
    else if (ct->power.direction_detect_complete && ct->power.power_direction == 0)
    {
        // 重置检测相关变量
        ct->power.direction_detect_complete = 0;
        ct->power.direction_power_sum = 0.0f;
        ct->power.direction_sample_count = 0;
    }

    // 如果有新的功率数据可用
    if (ct->power.power_ready)
    {
        ct->power.direction_power_sum += ct->power.avg_power;
        ct->power.direction_sample_count++;

        // 如果收集满250个样本（5s），计算平均值并判断方向
        if (ct->power.direction_sample_count >= 250)
        {
            float avg_power_50samples = ct->power.direction_power_sum / 250.0f;

            if (avg_power_50samples >= 0.0f)
            {
                ct->power.power_direction = 1.0f; // 正方向
            }
            else
            {
                ct->power.power_direction = -1.0f; // 负方向，需要取反
            }

            // 标记检测完成
            ct->power.direction_detect_complete = 1;

            // 重置计数器和累加和，为下一次可能的重新检测做准备
            ct->power.direction_power_sum = 0.0f;
            ct->power.direction_sample_count = 0;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void system_flags_init(void)
 Input       : 无
 Output      : 无
 Description : 初始化系统标志位管理器
---------------------------------------------------------------------------*/
void system_flags_init(void)
{
    memset(&sys_param.flags, 0, sizeof(system_flags_t));
}

/*---------------------------------------------------------------------------
 Name        : void ct_online_detect_init(ct_param_t *ct_param)
 Input       : ct_param
 Output      : 无
 Description : 初始化CT在线检测参数
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
 Input       : calc_power
 Output      : 无
 Description : 初始化功率计算参数
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
 Name        : grid_manager_init(void)
 Input       : 无
 Output      : 无
 Description : 初始化电网检测参数
---------------------------------------------------------------------------*/
void grid_manager_init(void)
{
    sys_param.state = SYS_INIT;

    // 初始化过零参数
    sys_param.grid.zero_cross.positive_zero_cross = 0;
    sys_param.grid.zero_cross.frequency_valid = 0;

    // 自适应频率默认值（50Hz基准，正向过零后会更新）
    sys_param.grid.samples_per_cycle = 400;     // 50Hz: 20ms/50us
    sys_param.grid.phase_b_delay_samples = 133; // 400/3
    sys_param.grid.phase_c_delay_samples = 267; // 400*2/3

    // 初始化相序识别参数
    phase_identify_init(&sys_param.grid.phase_id);
}

void ct_power_direction_detect_init(ct_param_t *ct)
{
    if (ct == NULL)
        return;

    ct->power.power_direction = 0;           // 重置功率方向
    ct->power.direction_detect_complete = 0; // 重置检测完成标志
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
 Input       : phase_count - 相数(3=三相)
               power_array - 功率数组
 Output      : No
 Description : 功率广播任务底层函数
               - 广播功率给所有微逆设备（地址0x0000）
               - 轮流指定哪个微逆上报数据
               - 每个微逆连续SWITCH_INV_BOARCAST次，然后轮换到下一个
               - 即使微逆不在线也会广播（带上其地址以接收上报）
---------------------------------------------------------------------------*/
static void broadcast_three_phase_power(float *power_array)
{
    if (g_ota_manager.disable_broadcast) // OTA期间禁止广播
    {
        return;
    }

    static uint8_t current_slot = 0;    // 当前槽位（0-7）
    static uint8_t broadcast_count = 0; // 已广播次数

    // 计算限功率偏移值
    int16_t ct_to_grid_power[3] = {0};
    if (sys_param.power_work_mode == 2)
    {
        if (sys_param.is_three_phase)
        {
            // 三相：平均分配
            int16_t avg_power = sys_param.to_grid_power_limit / 3;
            ct_to_grid_power[0] = ct_to_grid_power[1] = ct_to_grid_power[2] = avg_power;
        }
        else
        {
            // 单相：根据sequence_k确定相位，全加在哪一相
            int phase = (sys_param.grid.phase_id.sequence_k - 1) / 2;
            if (phase >= 0 && phase < 3)
            {
                ct_to_grid_power[phase] = sys_param.to_grid_power_limit;
            }
        }
    }

    // 查找有效槽位
    uint8_t attempts = 0;
    while (!sys_param.paired_inv_info[current_slot].is_valid && attempts < INV_DEVICE_MAX_NUM)
    {
        current_slot = (current_slot + 1) % INV_DEVICE_MAX_NUM;
        attempts++;
    }

    // 如果所有槽位都无效,使用地址0(广播地址)
    uint32_t target_addr = (attempts < INV_DEVICE_MAX_NUM) ? sys_param.paired_inv_info[current_slot].sub1g_addr : 0;

    // 计算真正广播的功率值（加上限功率偏移）
    int16_t broadcast_power_ct1 = (int16_t)(power_array[0] + ct_to_grid_power[0]);
    int16_t broadcast_power_ct2 = (int16_t)(power_array[1] + ct_to_grid_power[1]);
    int16_t broadcast_power_ct3 = (int16_t)(power_array[2] + ct_to_grid_power[2]);

    // 更新广播平均功率
    sys_param.ct1.power.ct_sub1g_boardcast_power_avg = (float)broadcast_power_ct1;
    sys_param.ct2.power.ct_sub1g_boardcast_power_avg = (float)broadcast_power_ct2;
    sys_param.ct3.power.ct_sub1g_boardcast_power_avg = (float)broadcast_power_ct3;

    // 广播三相功率
    sub1g_send_broadcast_three_phase_power(broadcast_power_ct1, broadcast_power_ct2, broadcast_power_ct3, target_addr);

    // 广播计数，N次后切换槽位
    broadcast_count++;
    if (broadcast_count >= SWITCH_INV_BOARCAST)
    {
        broadcast_count = 0;
        current_slot = (current_slot + 1) % INV_DEVICE_MAX_NUM;
    }
}

/*---------------------------------------------------------------------------
 Name        : calculate_ct_boardcast_power_avg
 Description : 三相模式CT功率累加(不独立计数)
 Input       : ct_index - CT索引(0/1/2)
               direction_complete - 方向检测完成标志
               avg_power - 平均功率
               broadcast_power_avg - 广播平均功率(输出)
---------------------------------------------------------------------------*/
static void calculate_ct_boardcast_power_avg(uint8_t ct_index, bool direction_complete, float avg_power)
{
    if (!direction_complete)
    {
        // 方向检测未完成,清零累加器
        ct_power_accum[ct_index] = 0;
        return;
    }

    // 累加功率
    ct_power_accum[ct_index] += avg_power;
}

/*---------------------------------------------------------------------------
 Name        : void boardcast_power_task(void)
 Input       : No
 Output      : No
 Description : 功率广播任务，每电网周期调用一次（由 power_cycle_ready 触发）
               - 在SYS_NORMAL_RUN状态：累加2个电网周期功率后广播（50Hz=40ms，60Hz=33ms）
               - 在SYS_POWER_DIR_DETECT且relay_opening_pending状态：轮询打开继电器
               - 在其他状态：轮询关闭继电器
               - 所有状态都会轮换槽位以接收微逆上报
---------------------------------------------------------------------------*/
void boardcast_power_task(void)
{
    // 检查运行条件：每个电网周期功率计算完成后触发
    if (!sys_param.flags.task.power_cycle_ready)
        return;

    sys_param.flags.task.power_cycle_ready = 0;

    float power_array[3] = {0.0f, 0.0f, 0.0f};

    // 判断当前是否在正常运行状态
    if (sys_param.state == SYS_NORMAL_RUN)
    {
        // ========== 正常运行模式:广播实际三相功率 ==========

        // 累加功率
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
                // 计算平均功率
                power_array[0] = ct_power_accum[0] / three_phase_broadcast_count;
                power_array[1] = ct_power_accum[1] / three_phase_broadcast_count;
                power_array[2] = ct_power_accum[2] / three_phase_broadcast_count;

                // 单相系统中，不在线的CT功率设置为0
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

            // 广播功率
            broadcast_three_phase_power(power_array);

            // 重置计数器和累加器
            three_phase_broadcast_count = 0;
            ct_power_accum[0] = 0;
            ct_power_accum[1] = 0;
            ct_power_accum[2] = 0;
        }
    }
    else
    {
        // 非正常运行状态，广播0功率
        sys_param.ct1.power.ct_sub1g_boardcast_power_avg = 0.0f;
        sys_param.ct2.power.ct_sub1g_boardcast_power_avg = 0.0f;
        sys_param.ct3.power.ct_sub1g_boardcast_power_avg = 0.0f;

        // 判断是否需要打开继电器(功率方向检测完成后2秒内)
        static uint8_t relay_slot = 0;
        static uint8_t broadcast_toggle = 0; // 交替0=广播功率, 1=广播继电器开关
        bool should_open = (sys_param.state == SYS_POWER_DIR_DETECT && sys_param.grid.phase_id.relay_opening_pending);
        should_open = true;
        if (broadcast_toggle == 0)
        {
            // 广播0功率
            power_array[0] = 0.0f;
            power_array[1] = 0.0f;
            power_array[2] = 0.0f;
            broadcast_three_phase_power(power_array);

            broadcast_toggle = 1; // 下次广播继电器开关
        }
        else
        {
            // 广播继电器开关
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
            broadcast_toggle = 0; // 下次广播功率
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void broadcast_other_task(void)
 Input       : 无
 Output      : 无
 Description : 广播FFT是否需要采样与分析，并每10秒调用一次日期广播任务
               - 广播日期给所有微逆设备（地址0x0000）
               - 每30s广播三相在线发电的微逆个数（仅三相模式）
---------------------------------------------------------------------------*/
static void broadcast_other_task(void)
{
    // 检查FFT相关操作的前置条件
    bool fft_conditions_met = (sys_param.state == SYS_NORMAL_RUN) && (sys_param.grid.phase_id.sequence_k > 0);

    // 第一优先级：检查是否完成4次识别，需要发送最终确认
    if (sys_param.fft_identify.final_confirm_pending &&
        sys_param.fft_identify.boardcast_interval == 0 &&
        fft_conditions_met)
    {
        sys_param.fft_identify.final_confirm_pending = false;

#ifdef FFT_DEBUG_PRINT
        printf("发送最终相位信息给微逆: CT%d\r\n", sys_param.fft_identify.identified_ct);
#endif

        // 发送相位信息给微逆
        sub1g_send_set_inv_phase(sys_param.fft_identify.sub1g_addr, sys_param.fft_identify.identified_ct);

        // 保存到EEPROM
        uint8_t idx = find_inv_index_by_sub1g_addr(sys_param.fft_identify.sub1g_addr);
        if (idx < INV_DEVICE_MAX_NUM)
        {
            sys_param.paired_inv_info[idx].phase = sys_param.fft_identify.identified_ct;
            sys_param.paired_inv_info[idx].prop_changed = true;
            eeprom_update_device_phase(sys_param.fft_identify.sub1g_addr, sys_param.fft_identify.identified_ct);
        }

        // 识别完全结束，清理所有状态
        sys_param.fft_identify.sub1g_addr = 0;
        sys_param.fft_identify.consecutive_success_count = 0;
        sys_param.fft_identify.last_identified_ct = 0;

        return; // 本次任务完成，直接返回
    }

    // 第二优先级：发送相序识别命令
    if (sys_param.fft_identify.resend_cmd &&
        sys_param.fft_identify.boardcast_interval == 0 &&
        fft_conditions_met)
    {
        sys_param.fft_identify.resend_cmd = false;

        uint16_t power = sys_param.fft_identify.power;

        // 发送开启相序识别命令
        sub1g_send_enable_phase_identify(sys_param.fft_identify.sub1g_addr, 25, power, sys_param.fft_identify.interval_time);

        // 发送命令后，启动本轮识别
        sys_param.fft_identify.is_ffting = 1; // 开始新一轮识别

#ifdef FFT_DEBUG_PRINT
        printf("发送相序识别命令: addr=0x%08X, power=%dW, interval=%d\r\n",
               sys_param.fft_identify.sub1g_addr,
               power,
               sys_param.fft_identify.interval_time);
        printf("等待2秒后开始采集...\r\n");
#endif

        return; // 发送完命令后返回
    }

    // 第三优先级：等待2秒期间的重发补偿
    if (sys_param.fft_identify.retry_flag && fft_conditions_met)
    {
        sys_param.fft_identify.retry_flag = 0;

        uint16_t power = sys_param.fft_identify.power;
        sub1g_send_enable_phase_identify(sys_param.fft_identify.sub1g_addr, 25, power, sys_param.fft_identify.interval_time);

#ifdef FFT_DEBUG_PRINT
        printf("重发相序识别命令: addr=0x%08X, power=%dW\r\n", sys_param.fft_identify.sub1g_addr, power);
#endif
        return;
    }

    // 第四优先级：日期广播任务
    if (sys_param.date_broadcast_counter >= 20000)
    {
        sys_param.date_broadcast_counter = 0;

        // 检查日期字符串是否有效
        if (strlen(sys_param.time.date) < 10)
        {
            return; // 日期格式错误，不广播
        }

        // 广播日期
        sub1g_send_broadcast_date(sys_param.time.date);

        // 40s一次广播三相在线发电的微逆个数
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
 Input       : inv_idx - 微逆索引
 Output      : 无
 Description : 清除离线微逆的上报数据缓存，避免WiFi读取到旧值
               保留设备身份信息、配置参数和累计发电量，清空版本号，使设备重新上线时触发版本上报
---------------------------------------------------------------------------*/
static void clear_offline_inverter_data(uint8_t inv_idx)
{
    if (inv_idx >= INV_DEVICE_MAX_NUM)
        return;

    inv_device_t *inv = &sys_param.paired_inv_info[inv_idx];

    // 清除工作状态和功率数据
    inv->work_state = 0;
    inv->grid_power = 0.0f;

    // 清除PV数据
    for (uint8_t pv_idx = 0; pv_idx < 4; pv_idx++)
    {
        inv->pv[pv_idx].state = 0;
        inv->pv[pv_idx].power = 0;
        inv->pv[pv_idx].voltage = 0.0f;
        inv->pv[pv_idx].current = 0.0f;
    }

    // 清空版本号，使设备重新上线时触发版本上报
    inv->sw_version[0] = '\0';
    inv->sub1g_version[0] = '\0';

    DEBUG_PRINTF("[Offline] Clear inv[%d] (0x%06X) data \r\n", inv_idx, inv->sub1g_addr);
}

static void cal_phase_inv_1s(void)
{
    // 36秒计数和功率累加静态变量
    static uint8_t power_calc_cnt = 0;
    static float ct1_power_sum = 0.0f;
    static float ct2_power_sum = 0.0f;
    static float ct3_power_sum = 0.0f;
    static uint8_t save_eep_intrval = 0;

    uint8_t ct1_inv_count = 0;  // CT1相发电计数器
    uint8_t ct2_inv_count = 0;  // CT2相发电计数器
    uint8_t ct3_inv_count = 0;  // CT3相发电计数器
    float ct1_inv_power = 0.0f; // CT1相功率
    float ct2_inv_power = 0.0f; // CT2相功率
    float ct3_inv_power = 0.0f; // CT3相功率

    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].online_state == 2)
        {
            // OTA期间不判断在线设备
            if (!g_ota_manager.disable_broadcast)
            {
                sys_param.paired_inv_info[i].offline_updata_ms++;
            }

            // 已有配对设备计数器更新，超时1分钟未上报数据，记作离线
            if (sys_param.paired_inv_info[i].offline_updata_ms >= PAIRED_INV_ONLINE_TIMEOUT_S)
            {
                sys_param.paired_inv_info[i].offline_updata_ms = PAIRED_INV_ONLINE_TIMEOUT_S;
                if (sys_param.paired_inv_info[i].online_state == 2)
                {
                    sys_param.paired_inv_info[i].online_state = 1; // 配对设备离线

                    clear_offline_inverter_data(i); // 清除离线设备的数据缓存

                    // 标记属性已变化，目的是为了上报给wifi
                    sys_param.paired_inv_info[i].prop_changed = true;
                }
            }
            else
            {
                // 统计正在发电的微逆个数（在线且功率大于1W）
                if (sys_param.paired_inv_info[i].grid_power > 1)
                {
                    if (sys_param.is_three_phase)
                    {
                        // 三相系统：根据相位(CT几)分别统计
                        switch (sys_param.paired_inv_info[i].phase)
                        {
                        case 1: // CT1相
                            ct1_inv_count++;
                            ct1_inv_power += sys_param.paired_inv_info[i].grid_power;
                            break;
                        case 2: // CT2相
                            ct2_inv_count++;
                            ct2_inv_power += sys_param.paired_inv_info[i].grid_power;
                            break;
                        case 3: // CT3相
                            ct3_inv_count++;
                            ct3_inv_power += sys_param.paired_inv_info[i].grid_power;
                            break;
                        default: // phase == 0 (未识别)，暂不计入
                            break;
                        }
                    }
                    else
                    {
                        // 单相系统：所有发电设备都计入A相
                        ct1_inv_count++;
                        ct1_inv_power += sys_param.paired_inv_info[i].grid_power;
                    }
                }
            }
        }
        else if (sys_param.paired_inv_info[i].is_valid == 0)
        {
            sys_param.paired_inv_info[i].online_state = 0; // 没有配对的INV设备
        }

        // 检查并删除超时的未配对设备(超过10秒未收到广播)
        if (sys_param.inv_request_pair_list[i].is_valid)
        {
            // 检查是否超时(10秒 = 10000ms)
            if (sys_param.inv_request_pair_list[i].unpaired_updata_ms >= UNPAIRED_DEVICE_TIMEOUT_MS)
            {
                // 清除设备
                sys_param.inv_request_pair_list[i].is_valid = false;
                sys_param.inv_request_pair_list[i].sub1g_addr = 0;
                sys_param.inv_request_pair_list[i].unpaired_updata_ms = 0;
                sys_param.paired_inv_info[i].grid_power = 0.0f;
            }
        }
    }

    // 更新各相正在发电的微逆个数、功率
    sys_param.ct1.inv_device_cnt = ct1_inv_count;
    sys_param.ct2.inv_device_cnt = ct2_inv_count;
    sys_param.ct3.inv_device_cnt = ct3_inv_count;

    // CT1、CT2、CT3微逆发电功率
    sys_param.ct1.inv_power = ct1_inv_power;
    sys_param.ct2.inv_power = ct2_inv_power;
    sys_param.ct3.inv_power = ct3_inv_power;

    // CT1、CT2、CT3相负载功率
    sys_param.ct1.use_power = ct1_inv_power + sys_param.ct1.power.fix_dir_power;
    sys_param.ct2.use_power = ct2_inv_power + sys_param.ct2.power.fix_dir_power;
    sys_param.ct3.use_power = ct3_inv_power + sys_param.ct3.power.fix_dir_power;

    // 累加每秒功率值
    ct1_power_sum += sys_param.ct1.power.fix_dir_power;
    ct2_power_sum += sys_param.ct2.power.fix_dir_power;
    ct3_power_sum += sys_param.ct3.power.fix_dir_power;
    power_calc_cnt++;

    // 每36秒计算一次发电量
    if (power_calc_cnt >= 36)
    {
        // 36秒的发电量(Wh) = (功率累加值 / 36) × (36/3600)
        sys_param.ct1.power_consumption = ct1_power_sum / 3600.0f;
        sys_param.ct2.power_consumption = ct2_power_sum / 3600.0f;
        sys_param.ct3.power_consumption = ct3_power_sum / 3600.0f;

        sys_param.hmi.electricity_consumption = (uint32_t)(sys_param.hmi.electricity_consumption + sys_param.ct1.power_consumption + sys_param.ct2.power_consumption + sys_param.ct3.power_consumption);

        save_eep_intrval++;
        if (save_eep_intrval >= 10) // 360秒 = 6分钟
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

        // 重置计数器和累加值
        power_calc_cnt = 0;
        ct1_power_sum = 0;
        ct2_power_sum = 0;
        ct3_power_sum = 0;
    }

    // 根据工作模式更新限流状态更新
    float total_grid_power = sys_param.ct1.power.fix_dir_power + sys_param.ct2.power.fix_dir_power + sys_param.ct3.power.fix_dir_power;

    switch (sys_param.power_work_mode)
    {
    case 1:                                 // 防逆流发电模式
        sys_param.anti_backflow_switch = 1; // 开启防逆流
        if (total_grid_power < (-30))       // 3相这里用-30W
        {
            sys_param.limit_state = 2; // 限流失败
        }
        else
        {
            sys_param.limit_state = 1; // 保护中
        }
        break;

    case 2:                                 // 限功率发电模式
        sys_param.anti_backflow_switch = 1; // 开启防逆流
        if (total_grid_power < -(sys_param.to_grid_power_limit))
        {
            sys_param.limit_state = 2; // 限流失败
        }
        else if (total_grid_power < -(sys_param.to_grid_power_limit) * 0.8f)
        {
            sys_param.limit_state = 1; // 限流中
        }
        else
        {
            sys_param.limit_state = 0; // 正常发电
        }
        break;

    case 3:                                 // 自由发电模式
        sys_param.anti_backflow_switch = 0; // 关闭防逆流
        sys_param.limit_state = 0;          // 自由发电中
        break;

    default:
        sys_param.anti_backflow_switch = 1;
        sys_param.limit_state = 0;
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : inv_comm_stats_1s_task
  Input       : 无
 Output      : 无
 Description :
---------------------------------------------------------------------------*/
static void inv_comm_stats_1s_task(void)
{
    // 秒计数递增
    uint8_t bound_inv_count = 0;
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid)
        {
            bound_inv_count++;
        }
    }

    // 每120秒（2分钟）计算一次丢包率和平均RSSI
    if (bound_inv_count == 0)
    {
        return;
    }

    // 60秒统计窗口内期望包数: 60000ms / 40ms = 1500包
    // N台微逆时每台期望包数 1500/N 包
    // 10%容差: 实收包数 >= 期望包数 * 90% 则认为100%在线
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

        // 计算60秒内的统计
        if (inv->stats_time_sec >= 60)
        {
            uint16_t total_rx = inv->rx_0x50_count + inv->rx_0x52_count + inv->rx_0x54_count +
                                inv->rx_0x55_count + inv->rx_0x56_count + inv->rx_0x57_count;

            // 计算丢包数量
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

            // 计算各自丢包率
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
 Description :
---------------------------------------------------------------------------*/
static void param_update_1s_task(void)
{
    if (sys_param.flags.timer_1s_flag)
    {
        sys_param.flags.timer_1s_flag = 0;

        // 微逆通信统计任务
        inv_comm_stats_1s_task();

        // 更新各相正在发电的微逆个数、功率、发电量
        cal_phase_inv_1s();

        // 更新hmi參數
        hmi_update_all_params();

        // 先检查频率是否有故障
        if (sys_param.fault.bit.grid_frequency)
        {
            DEBUG_PRINTF("[State Machine] Grid frequency fault detected, SYS_FREQ_FAULT.\r\n");
        }

        // static uint8_t printf_intreval = 0;
        // printf_intreval++;
        // if (printf_intreval >= 4)
        // {
        //     printf_intreval = 0;
        //     // 打印CT有效值以及在线状态
        //     printf("CT1_Rms:%f 在线:%d 功率:%.2f, CT2_Rms:%f 在线:%d 功率:%.2f, CT3_Rms:%f 在线:%d 功率:%.2f。 三相系统:%d\r\n", sys_param.ct1.rms_value, sys_param.ct1.status.connect_status, sys_param.ct1.power.fix_dir_power, sys_param.ct2.rms_value, sys_param.ct2.status.connect_status, sys_param.ct2.power.fix_dir_power, sys_param.ct3.rms_value, sys_param.ct3.status.connect_status, sys_param.ct3.power.fix_dir_power, sys_param.is_three_phase);
        // }
        // 打印三相广播功率以及是否是三相模式
        // DEBUG_PRINTF("广播功率:%.2f, %.2f, %.2f, 三相系统:%d, to_grid=%d\r\n", sys_param.ct1.power.ct_sub1g_boardcast_power_avg, sys_param.ct2.power.ct_sub1g_boardcast_power_avg, sys_param.ct3.power.ct_sub1g_boardcast_power_avg, sys_param.is_three_phase, sys_param.to_grid_power_limit);

#ifdef FFT_DEBUG_PRINT
        if (sys_param.fft_identify.enable_collect == 1)
        {
            printf("正在FFT采集:\r\n");
        }
#endif
    }
}

/*---------------------------------------------------------------------------
 Name        : void sub1g_timer_task(void)
 Input       : 无
 Output      : 无
 Description : SUB1G定时器任务
               - 上电3秒后每3秒发送0x41获取版本信息
               - 每2秒发送0x42获取RSSI
---------------------------------------------------------------------------*/
static void sub1g_timer_task(void)
{
    // 每3秒发送0x41获取版本信息，直到收到版本回复
    if (sys_param.sub1g.sw_version[0] == '\0')
    {
        if (sys_param.sub1g.version_timer_ms >= 3000)
        {
            sub1g_send_get_version();
            sys_param.sub1g.version_timer_ms = 0;
        }
    }

    // 每10秒发送0x42获取RSSI
    if (sys_param.sub1g.rssi_timer_ms >= 10000)
    {
        sub1g_send_get_rssi();
        sys_param.sub1g.rssi_timer_ms = 0;
    }
}
