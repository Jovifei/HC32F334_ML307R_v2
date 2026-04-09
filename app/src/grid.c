#include <string.h>
#include "debug.h"
#include "board.h"
#include "main.h"
#include "grid.h"
#include "arm_math.h"
#include "eeprom.h"

extern volatile uint8_t phase_identify_timer_100ms;

static float current1_rms = 0.0f, current2_rms = 0.0f, current3_rms = 0.0f;

// 相序映射表 - 定义6种可能的组合
const uint8_t phase_map[6][3] = {
    {0, 1, 2}, // σ1: Ua->CT1(ia), Ub->CT2(ib), Uc->CT3(ic)
    {0, 2, 1}, // σ2: Ua->CT1(ia), Ub->CT3(ic), Uc->CT2(ib)
    {1, 0, 2}, // σ3: Ua->CT2(ib), Ub->CT1(ia), Uc->CT3(ic)
    {1, 2, 0}, // σ4: Ua->CT2(ib), Ub->CT3(ic), Uc->CT1(ia)
    {2, 0, 1}, // σ5: Ua->CT3(ic), Ub->CT1(ia), Uc->CT2(ib)
    {2, 1, 0}  // σ6: Ua->CT3(ic), Ub->CT2(ib), Uc->CT1(ia)
};

/*---------------------------------------------------------------------------
 Name        : void grid_task(void)
 Input       : 无
 Output      : 无
 Description : 电网任务处理函数（主循环中调用）。
               承接从 zero_cross_detect 中断移出的除法和浮点运算：
               - phase_b_delay_samples = period_samples / 3（整数除法）
               - phase_c_delay_samples = period_samples * 2 / 3（整数乘除）
               - grid_frequency = N周期累计平均（浮点除法，每 FREQ_AVG_CYCLES 次更新）
               中断内置 period_updated=1 通知此函数有新数据。
---------------------------------------------------------------------------*/
void grid_task(void)
{
    zero_cross_detect_t *zc = &sys_param.grid.zero_cross;

    if (!zc->period_updated)
    {
        return;
    }
    zc->period_updated = 0u;

    uint32_t ps = zc->period_samples;

    // 整数除法：相位延迟采样点数
    sys_param.grid.phase_b_delay_samples = (uint16_t)(ps / 3u);
    sys_param.grid.phase_c_delay_samples = (uint16_t)(ps * 2u / 3u);

    // 浮点除法：N周期平均频率（FREQ_AVG_CYCLES 次累计后才更新显示值）
    if (zc->period_avg_count >= FREQ_AVG_CYCLES)
    {
        sys_param.grid.grid_frequency = (1000000.0f * (float)FREQ_AVG_CYCLES) / ((float)(zc->period_accum + FREQ_AVG_CYCLES) * (float)ADC_SAMPLE_PERIOD_US);
        zc->period_accum = 0u;
        zc->period_avg_count = 0u;
    }
}

/*---------------------------------------------------------------------------
 Name        : float calculate_rms(float *buffer, uint16_t count)
 Input       : buffer - 数据缓冲区
               count - 计算点数
 Output      : 有效值
 Description : 计算有效值（此文件内部使用）
---------------------------------------------------------------------------*/
float calculate_rms(float *buffer, uint16_t count)
{
    float sum_square = 0.0f;
    uint16_t i;

    for (i = 0; i < count; i++)
    {
        sum_square += buffer[i] * buffer[i];
    }
    return sqrtf(sum_square / count);
}

/*---------------------------------------------------------------------------
 Name        : float calculate_rms_ring(float *buffer, uint16_t total_size, uint16_t start_index, uint16_t count)
 Input       : buffer - 数据缓冲区
               total_size - 缓冲区总大小
               start_index - 起始索引
               count - 计算点数
 Output      : 有效值
 Description : 环形缓冲区有效值计算（用于按周期窗口计算）
---------------------------------------------------------------------------*/
float calculate_rms_ring(float *buffer, uint16_t total_size, uint16_t start_index, uint16_t count)
{
    float sum_square = 0.0f;
    uint16_t i;

    for (i = 0; i < count; i++)
    {
        float v = buffer[(start_index + i) % total_size];
        sum_square += v * v;
    }
    return (count > 0u) ? sqrtf(sum_square / (float)count) : 0.0f;
}

/*---------------------------------------------------------------------------
 Name        : float calculate_active_power(float *voltage, float *current, uint16_t count)
 Input       : voltage - 电压缓冲区
               current - 电流缓冲区
               count - 计算点数
 Output      : 有功功率
 Description : 计算有功功率
---------------------------------------------------------------------------*/
float calculate_active_power(float *voltage, float *current, uint16_t count)
{
    float sum_power = 0.0f;
    uint16_t i;

    for (i = 0; i < count; i++)
    {
        sum_power += voltage[i] * current[i];
    }
    return sum_power / count;
}

/*---------------------------------------------------------------------------
 Name        : void zero_cross_detect(zero_cross_detect_t *zc_detect, float voltage)
 Input       : zc_detect - 过零检测结构体指针
               voltage   - 原始（未滤波）电压瞬时值，避免 LPF 相位滞后导致计数偏差
 Output      : 无
 Description : 过零检测（在50us ADC中断中调用）。
               中断内仅执行整数加减和赋值，不做乘除和浮点运算：
               - 半周期计数（整数加法）
               - period_samples = last_half + half（整数加法）
               - samples_per_cycle 直接赋值（用于缓冲窗口，需每周期立即更新）
               - period_accum 累加（整数加法），period_updated 置1通知主循环

               以下运算移至主循环 grid_task() 执行：
               - grid_frequency（浮点除法）
               - phase_b_delay_samples（整数除法 ps/3）
               - phase_c_delay_samples（整数除法 ps*2/3）

               过零判断采用"连续两个同符号样本"，正弦波上升/下降斜率相同，
               正负过零延迟各+1采样，合并时完全抵消，无系统误差。
---------------------------------------------------------------------------*/
void zero_cross_detect(zero_cross_detect_t *zc_detect, float voltage)
{
    extern uint8_t buffer_filled;

    if (zc_detect == NULL)
    {
        return;
    }

    zc_detect->zero_sample_count++;

    // 超时保护：避免长时间无过零导致计数溢出
    if (zc_detect->zero_sample_count >= 800)
    {
        zc_detect->zero_sample_count = 800;
    }

    // ---- 正向过零（从负到正）：连续两个正值 + 守卫计数 ----
    if (zc_detect->position == 0 && voltage > 0.0f && zc_detect->last_voltage > 0.0f && zc_detect->zero_sample_count > GRID_HALF_PERIOD_MIN)
    {
        //  记录负半周时长
        zc_detect->last_half_period = zc_detect->half_period;
        zc_detect->half_period = zc_detect->zero_sample_count;
        zc_detect->zero_sample_count = 0u;
        zc_detect->position = 1u;

        zc_detect->positive_zero_cross = 1u;
        zc_detect->zero_cross_detected = 1u;

        if (sys_param.state == SYS_INIT)
        {
            zc_detect->zero_cross_count++;
        }
        else
        {
            zc_detect->zero_cross_count = 0u;
        }

        // 触发计算（参数已在上次负向过零时由主循环更新完毕）
        if (zc_detect->frequency_valid && !sys_param.grid.frequency_fault && buffer_filled)
        {
            sys_param.flags.rms_calc_ready = 1u;
            sys_param.flags.task.power_calc_ready = 1u;
        }
    }
    // ---- 负向过零（从正到负）：连续两个负值 + 守卫计数 ----
    else if (zc_detect->position == 1u && voltage < 0.0f && zc_detect->last_voltage < 0.0f && zc_detect->zero_sample_count > GRID_HALF_PERIOD_MIN)
    {
        // 整数赋值：记录正半周时长，合并完整周期
        zc_detect->last_half_period = zc_detect->half_period;
        zc_detect->half_period = zc_detect->zero_sample_count;
        zc_detect->zero_sample_count = 0u;
        zc_detect->position = 0u;

        zc_detect->zero_cross_detected = 1u;
        zc_detect->positive_zero_cross = 0u;

        // 完整周期 = 上次半周（负半周）+ 本次半周（正半周）
        uint32_t ps = zc_detect->last_half_period + zc_detect->half_period;

        if (ps >= (uint32_t)GRID_FREQ_SAMPLES_MIN && ps <= (uint32_t)GRID_FREQ_SAMPLES_MAX)
        {
            zc_detect->period_samples = ps;
            zc_detect->frequency_valid = 1u;
            sys_param.grid.frequency_fault = 0u;
            sys_param.fault.bit.grid_frequency = 0u;

            sys_param.grid.samples_per_cycle = (uint16_t)ps;

            // 累计供主循环计算平均频率（仅整数加法）
            zc_detect->period_accum += ps;
            zc_detect->period_avg_count++;
            zc_detect->period_updated = 1u; // 通知 grid_task 有新数据
        }
        else
        {
            // 频率超范围：整数赋值
            zc_detect->frequency_valid = 0u;
            sys_param.grid.frequency_fault = 1u;
            sys_param.fault.bit.grid_frequency = 1u;
            // 清除累计，防止污染下次平均
            zc_detect->period_accum = 0u;
            zc_detect->period_avg_count = 0u;
        }
    }
    else
    {
        zc_detect->zero_cross_detected = 0u;
        zc_detect->positive_zero_cross = 0u;
    }

    zc_detect->last_voltage = voltage;
}

/*---------------------------------------------------------------------------
 Name        : static float phase_matching_calculation(uint8_t phase_idx, uint8_t ct_idx, float total_current_rms)
 Input       : phase_idx ：A、B、C相；ct_idx：1、2、3个CT
 Output      :
 Description : 计算特定相和CT组合的匹配度。
               三相电压均从 last_ua_voltage_buffer[0..spc-1] 线性快照取值，
               保证 A/B/C 与电流使用同一时间窗，消除 buffer_index 推进引起的偏差。
               电流从环形缓冲按 curr_start 起点取值（由 phase_identify_process 计算）。
---------------------------------------------------------------------------*/
static float phase_matching_calculation(uint8_t phase_idx, uint8_t ct_idx, float total_current_rms)
{
    float current_rms;
    float active_power, apparent_power, power_factor;
    float *current_buffer;

    uint16_t spc = sys_param.grid.samples_per_cycle;
    uint16_t pb = sys_param.grid.phase_b_delay_samples;
    uint16_t pc = sys_param.grid.phase_c_delay_samples;

    // 三相对称：所有相电压RMS均等于A相RMS
    float voltage_rms = sys_param.grid.ua_vol_rms;

    // 选择对应的电流缓冲区和RMS
    switch (ct_idx)
    {
    case 0:
        current_buffer = current1_buffer;
        current_rms = current1_rms;
        break;
    case 1:
        current_buffer = current2_buffer;
        current_rms = current2_rms;
        break;
    case 2:
        current_buffer = current3_buffer;
        current_rms = current3_rms;
        break;
    default:
        return 0.0f;
    }

    if (voltage_rms < 0.1f || current_rms < CT_OFFLINE_THRESHOLD)
    {
        return 0.0f;
    }

    // 电流环形缓冲起点（使用 last_ua 拷贝时的快照，保证与电压窗口严格对齐）
    uint16_t curr_start = (uint16_t)((get_calc_buf_snap() + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES);

    // 计算有功功率：三相电压均从 last_ua 线性快照取值保证时间窗一致
    float sum_power = 0.0f;
    if (phase_idx == 0)
    {
        // A相：从 last_ua 线性快照直接取，delay=0
        for (uint16_t i = 0; i < spc; i++)
        {
            uint16_t ci = (curr_start + i) % TOTAL_SAMPLES;
            sum_power += last_ua_voltage_buffer[i] * current_buffer[ci];
        }
    }
    else if (phase_idx == 1)
    {
        // B相：last_ua 按 -120° 延迟重构
        for (uint16_t i = 0; i < spc; i++)
        {
            uint16_t ci = (curr_start + i) % TOTAL_SAMPLES;
            float vb = last_ua_voltage_buffer[(i + spc - pb) % spc];
            sum_power += vb * current_buffer[ci];
        }
    }
    else
    {
        // C相：last_ua 按 -240° 延迟重构
        for (uint16_t i = 0; i < spc; i++)
        {
            uint16_t ci = (curr_start + i) % TOTAL_SAMPLES;
            float vc = last_ua_voltage_buffer[(i + spc - pc) % spc];
            sum_power += vc * current_buffer[ci];
        }
    }
    active_power = sum_power / (float)spc;

    // 计算视在功率
    apparent_power = voltage_rms * current_rms;
    if (apparent_power < 1e-6f)
    {
        return 0.0f;
    }

    // 计算功率因数
    power_factor = active_power / apparent_power;
    if (fabsf(power_factor) > 1.0f)
    {
        power_factor = (power_factor > 0) ? 1.0f : -1.0f;
    }

    // 公式3: h = (RMS(in) / Σ RMS(il)) * |PF|
    return (current_rms / total_current_rms) * fabsf(power_factor);
}

/*---------------------------------------------------------------------------
 Name        : void update_ct_to_phase_mapping(uint8_t sequence_k)
 Input       : sequence_k: 相序编号(1-6)
 Output      : 无
 Description : 根据相序编号更新CT到相的映射关系
---------------------------------------------------------------------------*/
void update_ct_to_phase_mapping(uint8_t sequence_k)
{
    if (sequence_k >= 1 && sequence_k <= 6)
    {
        uint8_t idx = sequence_k - 1;
        for (uint8_t phase = 0; phase < 3; phase++)
        {
            uint8_t ct = phase_map[idx][phase];
            sys_param.grid.phase_id.ct_to_phase[ct] = phase;
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void wifi_set_phase_sequence(uint8_t sequence_k)
 Input       : sequence_k: 相序值(0-6), 0表示重新识别，1-6表示手动设置
 Output      : 无
 Description : WiFi手动设置相序，并保存到EEPROM
---------------------------------------------------------------------------*/
void wifi_set_phase_sequence(uint8_t sequence_k)
{
    if (sequence_k == 0)
    {
        // 触发自动相序识别：重置识别参数，直接进入识别状态
        sys_param.grid.phase_id.sequence_k = 0;
        sys_param.grid.phase_id.identification_valid = 0;

        // 重置识别历史
        memset(sys_param.grid.phase_id.identify_history, 0, sizeof(sys_param.grid.phase_id.identify_history));
        sys_param.grid.phase_id.identify_count = 0;
        sys_param.grid.phase_id.consistent_count = 0;
        sys_param.grid.phase_id.total_attempts = 0;

        // 保存到EEPROM（sequence_k=0，tag=0，下次重启tag不匹配会回归默认1）
        eeprom_save_set_param();

        // 直接进入相序识别状态，无需经过SYS_WAIT_CT
        sys_param.state = SYS_PHASE_IDENTIFY;

        DEBUG_PRINTF("[WiFi] Request auto phase identification, entering SYS_PHASE_IDENTIFY.\r\n");
    }
    else if (sequence_k >= 1 && sequence_k <= 6)
    {
        // 手动设置相序
        sys_param.grid.phase_id.sequence_k = sequence_k;
        sys_param.grid.phase_id.identification_valid = 1;

        // 更新CT到相的映射关系
        update_ct_to_phase_mapping(sequence_k);

        // 保存到EEPROM
        int ret = eeprom_save_set_param();
        if (ret == 0)
        {
            DEBUG_PRINTF("[WiFi] User set phase sequence_k=%d, saved to EEPROM successfully\r\n", sequence_k);
        }
        else
        {
            DEBUG_PRINTF("[WiFi] User set phase sequence_k=%d, but EEPROM save failed\r\n", sequence_k);
        }

        // 功率方向固定为正向，无需方向检测，映射更新后直接保持/进入NORMAL_RUN
        sys_param.ct1.power.direction_detect_complete = 1;
        sys_param.ct2.power.direction_detect_complete = 1;
        sys_param.ct3.power.direction_detect_complete = 1;
        sys_param.state = SYS_NORMAL_RUN;

        DEBUG_PRINTF("[WiFi] User set the phase sequence_k=%d: CT1->%c, CT2->%c, CT3->%c\r\n", sequence_k,
                     'A' + sys_param.grid.phase_id.ct_to_phase[0],
                     'A' + sys_param.grid.phase_id.ct_to_phase[1],
                     'A' + sys_param.grid.phase_id.ct_to_phase[2]);
    }
}

/*---------------------------------------------------------------------------
 Name        : void phase_identify_process(phase_identify_t *phase_id)
 Input       : phase_id - 相序识别结构体指针
 Output      : 无
 Description : 相序识别处理（在 ct_task 内 power_calc_ready 触发时调用）。
               所需数据（last_ua快照、s_calc_buf_snap、RMS）均已在本轮 ct_task
               内准备就绪，保证电压窗口与电流窗口严格对齐。
---------------------------------------------------------------------------*/
void phase_identify_process(phase_identify_t *phase_id)
{
    if (phase_id == NULL)
    {
        return;
    }

    // 只在SYS_PHASE_IDENTIFY状态才运行
    if (sys_param.state != SYS_PHASE_IDENTIFY)
    {
        return;
    }

    // 如果已经识别完成，不再进行识别
    if (phase_id->identification_valid)
    {
        return;
    }

    if (!phase_identify_timer_100ms) // 还没到100ms
    {
        return;
    }
    phase_identify_timer_100ms = 0; // 清除标志

    extern uint8_t buffer_filled;

    if (!buffer_filled)
    {
        return; // 缓冲区还没填充过
    }

    // 增加尝试次数（无上限，直到识别成功为止）
    phase_id->total_attempts++;

    // 获取CT的RMS值
    float ct1_rms = sys_param.ct1.rms_value;
    float ct2_rms = sys_param.ct2.rms_value;
    float ct3_rms = sys_param.ct3.rms_value;

    // 统计在线CT数量和判断是否满足功率阈值
    uint8_t online_ct_count = 0;
    uint8_t power_sufficient_count = 0;

    if (sys_param.ct1.status.connect_status == CT_STATUS_ONLINE)
    {
        online_ct_count++;
        if (ct1_rms >= CT_PHASE_IDENTIFY_THRESHOLD)
        {
            power_sufficient_count++;
        }
    }

    if (sys_param.ct2.status.connect_status == CT_STATUS_ONLINE)
    {
        online_ct_count++;
        if (ct2_rms >= CT_PHASE_IDENTIFY_THRESHOLD)
        {
            power_sufficient_count++;
        }
    }

    if (sys_param.ct3.status.connect_status == CT_STATUS_ONLINE)
    {
        online_ct_count++;
        if (ct3_rms >= CT_PHASE_IDENTIFY_THRESHOLD)
        {
            power_sufficient_count++;
        }
    }

    // 判断是否满足功率足够大，计算相序
    bool can_calculate = false;

    if (sys_param.is_three_phase)
    {
        // 三相模式:至少2个CT在线,且2个功率都满足阈值
        if (online_ct_count >= 2 && power_sufficient_count >= 2)
        {
            can_calculate = true;
        }
    }
    else
    {
        // 单相模式:1个CT在线,且功率满足阈值
        if (online_ct_count == 1 && power_sufficient_count >= 1)
        {
            can_calculate = true;
        }
    }

    if (!can_calculate)
    {
        // 功率不足,提示用户
        if (phase_id->total_attempts % 5 == 0)
        {
            DEBUG_PRINTF("[Phase ID] Power too low (Attempt %d), please increase load >%.0fW per CT\r\n",
                         phase_id->total_attempts, CT_PHASE_IDE_POWER_THRESHOLD);
        }
        return;
    }

    uint8_t best_sequence = 0;
    float max_matching_degree = -1.0f;

    // 使用与 last_ua 快照相同的起点计算电流RMS，保证与有功功率计算窗口一致
    uint16_t spc = sys_param.grid.samples_per_cycle;
    uint16_t snap = get_calc_buf_snap();
    uint16_t start = (uint16_t)((snap + TOTAL_SAMPLES - spc) % TOTAL_SAMPLES);

    current1_rms = calculate_rms_ring(current1_buffer, TOTAL_SAMPLES, start, spc);
    current2_rms = calculate_rms_ring(current2_buffer, TOTAL_SAMPLES, start, spc);
    current3_rms = calculate_rms_ring(current3_buffer, TOTAL_SAMPLES, start, spc);

    // 计算总电流有效值用于公式3
    float total_current_rms = current1_rms + current2_rms + current3_rms;

    // 计算6种组合的匹配度
    for (uint8_t seq = 0; seq < 6; seq++)
    {
        float matching_degree = 0.0f;

        // 计算三相匹配度度量值：F(σ)
        for (uint8_t u_phase = 0; u_phase < 3; u_phase++)
        {
            uint8_t ct_idx = phase_map[seq][u_phase];
            // 或者直接使用全局RMS变量
            matching_degree += phase_matching_calculation(u_phase, ct_idx, total_current_rms);
        }
        phase_id->matching_degree[seq] = matching_degree; // 6个组合的匹配度，方便调试，后期可删除

        // 找出最大匹配度
        if (matching_degree > max_matching_degree)
        {
            max_matching_degree = matching_degree;
            best_sequence = seq;
        }
    }

    // // 打印6个组合的匹配度，方便调试
    // for (uint8_t seq = 0; seq < 6; seq++)
    // {
    //     DEBUG_PRINTF("seq = %d, matching_degree = %f\r\n", seq, phase_id->matching_degree[seq]);
    // }

    // 更新识别结果
    uint8_t current_result = best_sequence + 1; // 返回值是0-5，算法的匹配关系是1-6哪种组合，需要加1

    // 保存到历史记录
    if (phase_id->identify_count < 10)
    {
        phase_id->identify_history[phase_id->identify_count] = current_result;
        phase_id->identify_count++;
        // DEBUG_PRINTF("[Phase ID]No.%d identify_result = %d\r\n",phase_id->identify_count, current_result);
    }
    else
    {
        // 历史记录已满，移动数组，保留最新10次
        for (uint8_t i = 0; i < 9; i++)
        {
            phase_id->identify_history[i] = phase_id->identify_history[i + 1];
        }
        phase_id->identify_history[9] = current_result;
    }

    // 检查连续4次是否一致
    if (phase_id->identify_count >= PHASE_IDENTIFY_CONSISTENT_COUNT)
    {
        // 检查最近4次是否完全一致
        uint8_t start_idx = phase_id->identify_count - PHASE_IDENTIFY_CONSISTENT_COUNT;
        uint8_t ref_value = phase_id->identify_history[start_idx];
        uint8_t match_count = 1;

        for (uint8_t i = start_idx + 1; i < phase_id->identify_count; i++)
        {
            if (phase_id->identify_history[i] == ref_value)
            {
                match_count++;
            }
        }

        if (match_count >= PHASE_IDENTIFY_CONSISTENT_COUNT)
        {
            phase_id->sequence_k = ref_value;
            phase_id->identification_valid = 1;
            phase_id->consistent_count = match_count;

            // 更新CT到相的映射关系
            for (uint8_t phase = 0; phase < 3; phase++)
            {
                uint8_t ct = phase_map[phase_id->sequence_k - 1][phase];
                phase_id->ct_to_phase[ct] = phase;
            }

            // // 保存识别结果到 EEPRO
            // eeprom_save_set_param();

            printf("[Phase ID] Identified sequence K = %d.\r\n", phase_id->sequence_k);
        }
    }
}

void phase_identify_init(phase_identify_t *phase_id)
{
    if (phase_id == NULL)
        return;

    phase_id->sequence_k = 0;
    phase_id->identification_valid = 0;

    // 初始化默认映射关系
    phase_id->ct_to_phase[0] = 0; // CT1 -> A相
    phase_id->ct_to_phase[1] = 1; // CT2 -> B相
    phase_id->ct_to_phase[2] = 2; // CT3 -> C相

    // 初始化匹配度数组
    memset(phase_id->matching_degree, 0, sizeof(phase_id->matching_degree));
    memset(phase_id->power_factor, 0, sizeof(phase_id->power_factor));

    // 初始化识别历史记录
    phase_id->identify_count = 0;
    phase_id->consistent_count = 0;
    phase_id->total_attempts = 0;
    memset(phase_id->identify_history, 0, sizeof(phase_id->identify_history));

    // 初始化继电器打开定时器
    phase_id->relay_open_timer_ms = 0;
    phase_id->relay_opening_pending = 0;
}
