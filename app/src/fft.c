#include "fft.h"
#include "main.h"
#include "sub1g.h"
#include "eeprom.h"
#include "twiddle_factors.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 3通道数据缓冲区
static int16_t FFT_data[CT_CHANNEL_MAX][N_FFT] = {0};
static fft_result_t fft_result[CT_CHANNEL_MAX] = {0};

// FFT工作缓冲区（共享）
static int16_t fft_real[N_FFT];
static int16_t fft_imag[N_FFT];
static uint32_t magnitude[FFT_BIN_COUNT];

/*---------------------------------------------------------------------------
 Name        : bit_reverse
 Input       : x - 输入值, bits - 位数
 Output      : 位反转后的值
 Description : FFT位反转函数
---------------------------------------------------------------------------*/
static uint16_t bit_reverse(uint16_t x, uint8_t bits)
{
    uint16_t rev = 0;
    for (int i = 0; i < bits; i++)
    {
        rev = (rev << 1) | (x & 1);
        x >>= 1;
    }
    return rev;
}

/*---------------------------------------------------------------------------
 Name        : fft_q15
 Input       : real_in - 输入实部数组(Q15格式), imag_out - 输出虚部数组
 Output      : 无
 Description : Q15定点FFT核心算法(Cooley-Tukey蝶形算法)
---------------------------------------------------------------------------*/
static void fft_q15(int16_t *real_in, int16_t *imag_out)
{
    // 1. 位反转重排
    for (int i = 0; i < N_FFT; i++)
    {
        uint16_t rev_i = bit_reverse(i, LOG2_N);
        fft_real[i] = real_in[rev_i];
        fft_imag[i] = 0;
    }

    // 2. 蝶形运算
    for (int stage = 1; stage <= LOG2_N; stage++)
    {
        int m = 1 << stage;          // 当前蝶形组长度
        int m2 = m >> 1;             // 半长
        int twiddle_step = 512 / m2; // 旋转因子步长

        for (int k = 0; k < N_FFT; k += m)
        {
            for (int j = 0; j < m2; j++)
            {
                // 计算旋转因子索引
                int index = j * twiddle_step;

                // 读取旋转因子 W = cos - j*sin
                int16_t wr = cos_table[index];
                int16_t wi = -sin_table[index];

                // 复数乘法: W * B = (wr - j*wi) * (real[k+j+m2] + j*imag[k+j+m2])
                // 结果: tr = wr*real - wi*imag, ti = wr*imag + wi*real
                int32_t tr = (int32_t)wr * fft_real[k + j + m2] -
                             (int32_t)wi * fft_imag[k + j + m2];
                int32_t ti = (int32_t)wr * fft_imag[k + j + m2] +
                             (int32_t)wi * fft_real[k + j + m2];

                // Q15右移15位还原
                tr >>= 15;
                ti >>= 15;

                // 保存原值（避免覆盖）
                int16_t temp_r = fft_real[k + j];
                int16_t temp_i = fft_imag[k + j];

                // 蝶形更新
                // A' = A + W*B
                fft_real[k + j] = temp_r + (int16_t)tr;
                fft_imag[k + j] = temp_i + (int16_t)ti;
                // B' = A - W*B
                fft_real[k + j + m2] = temp_r - (int16_t)tr;
                fft_imag[k + j + m2] = temp_i - (int16_t)ti;
            }
        }
    }

    // 3. 输出结果
    for (int i = 0; i < N_FFT; i++)
    {
        real_in[i] = fft_real[i];
        imag_out[i] = fft_imag[i];
    }
}

/*---------------------------------------------------------------------------
 Name        : fft_3ch_init
 Input       : 无
 Output      : 无
 Description : 初始化三通道FFT模块
---------------------------------------------------------------------------*/
void fft_3ch_init(void)
{
    for (int ch = 0; ch < CT_CHANNEL_MAX; ch++)
    {
        memset(FFT_data[ch], 0, sizeof(FFT_data[ch]));
        memset(&fft_result[ch], 0, sizeof(fft_result_t));
        fft_result[ch].state = FFT_IDLE;
    }
}

/*---------------------------------------------------------------------------
 Name        : fft_collect_power_data_3ch
 Input       : channel - CT通道号, power_value - 功率值(W)
 Output      : true-采集完成, false-继续采集
 Description : 采集指定CT通道的功率数据到FFT缓冲区
---------------------------------------------------------------------------*/
bool fft_collect_power_data_3ch(ct_channel_t channel, float power_value)
{
    if (channel >= CT_CHANNEL_MAX)
    {
        return false;
    }

    fft_result_t *result = &fft_result[channel];

    // 检查状态
    if (result->state != FFT_IDLE && result->state != FFT_COLLECTING)
    {
        return false;
    }

    // 开始采集
    if (result->state == FFT_IDLE)
    {
        result->state = FFT_COLLECTING;
        result->sample_count = 0;
    }

    // 将功率值转换为Q15格式 (功率范围 -15000W ~ +15000W)
    // Q15转换（±15000W）
    float scale = 2.184467f;
    int16_t power_q15 = (int16_t)(power_value * scale);

    // 限幅保护
    if (power_q15 > 32767)
    {
        power_q15 = 32767;
    }
    if (power_q15 < -32768)
    {
        power_q15 = -32768;
    }

    // 存储数据
    FFT_data[channel][result->sample_count++] = power_q15;

    // 检查是否采集完成
    if (result->sample_count >= N_FFT)
    {
        result->state = FFT_READY;
        return true;
    }

    return false;
}

/*---------------------------------------------------------------------------
 Name        : fft_analyze_power_spectrum_3ch
 Input       : channel - CT通道号
 Output      : true-检测到目标频率, false-未检测到
 Description : 执行FFT分析,检测计算出的理论频率是否为最大峰值
               理论频率 = 50/n Hz (n为相序识别间隔时间)
---------------------------------------------------------------------------*/
bool fft_analyze_power_spectrum_3ch(ct_channel_t channel)
{
    if (channel >= CT_CHANNEL_MAX)
    {
        return false;
    }

    fft_result_t *result = &fft_result[channel];

    if (result->state != FFT_READY)
    {
        return false;
    }

    result->state = FFT_ANALYZING;

    // 执行FFT（不去均值）
    fft_q15(FFT_data[channel], fft_imag);

    // 计算幅值平方 (只计算和存储4-14Hz范围的数据)
    float df = FFT_SAMPLE_RATE / N_FFT;         // 频率分辨率
    int min_bin = (int)(FFT_FREQ_MIN / df);     // 4Hz对应的最小bin
    int max_bin = (int)(FFT_FREQ_MAX / df) + 1; // 14Hz对应的最大bin

    // 计算4-14Hz范围内的幅值平方，存储到magnitude数组
    // magnitude[0]对应bin[min_bin], magnitude[i]对应bin[min_bin+i]
    for (int i = min_bin; i < max_bin && (i - min_bin) < FFT_BIN_COUNT; i++)
    {
        int32_t re = FFT_data[channel][i];
        int32_t im = fft_imag[i];
        magnitude[i - min_bin] = (uint32_t)(re * re + im * im);
    }

    // 找最大幅值(在magnitude数组中搜索)
    uint32_t max_mag = 0;
    int max_mag_idx = 0; // magnitude数组中的索引

    for (int i = 0; i < FFT_BIN_COUNT && (min_bin + i) < max_bin; i++)
    {
        if (magnitude[i] > max_mag)
        {
            max_mag = magnitude[i];
            max_mag_idx = i;
        }
    }

    // 映射回实际的bin索引
    int max_bin_idx = min_bin + max_mag_idx;

    // 计算最大频率
    float max_freq = max_bin_idx * df;

    // 计算理论频率: 1000/(2*n*10ms) = 50/n Hz，n只能是4、6、8、10、12
    uint8_t n = sys_param.fft_identify.interval_time;

    if (n == 0)
        n = 4; // 防止除零,默认为4

    float target_freq = 50.0f / n; // 理论频率: n=4->12.5Hz, n=6->8.33Hz, n=8->6.25Hz, n=10->5Hz, n=12->4.17Hz

    // 设置频率容差范围 0.12Hz
    float freq_min = target_freq - 0.5f;
    float freq_max = target_freq + 0.5f;

    // 保存结果
    result->max_freq = max_freq;
    result->max_magnitude = max_mag;
    result->is_target_freq_max = (max_freq >= freq_min && max_freq <= freq_max);
    result->state = FFT_COMPLETE;

#ifdef FFT_DEBUG_PRINT
    int min_bin_print = (int)(FFT_FREQ_MIN / df);
    int max_bin_print = (int)(FFT_FREQ_MAX / df) + 1;
    printf("=== CT%d FFT ===\n", channel + 1);
    printf("n=%d, Target: %.3f Hz (%.3f-%.3f Hz)\n", n, target_freq, freq_min, freq_max);
    printf("Analyzing frequency range: %.2f-%.2f Hz\n", FFT_FREQ_MIN, FFT_FREQ_MAX);
    // magnitude[i]对应实际的bin[min_bin+i]，频率为(min_bin+i)*df
    for (int i = 0; i < FFT_BIN_COUNT && (min_bin_print + i) < max_bin_print; i++)
    {
        printf("%.3f,%u\n", (min_bin_print + i) * df, magnitude[i]);
    }
    printf("Target_freq = %.3f Hz, Max: %.3f Hz (mag=%u), Result: %s\n",
           target_freq, max_freq, max_mag,
           result->is_target_freq_max ? "TRUE" : "FALSE");
#endif

    return result->is_target_freq_max;
}

/*---------------------------------------------------------------------------
 Name        : fft_reset_3ch
 Input       : channel - CT通道号
 Output      : 无
 Description : 复位指定CT通道的FFT模块
---------------------------------------------------------------------------*/
void fft_reset_3ch(ct_channel_t channel)
{
    if (channel >= CT_CHANNEL_MAX)
    {
        return;
    }

    memset(FFT_data[channel], 0, sizeof(FFT_data[channel]));
    fft_result[channel].sample_count = 0;
    fft_result[channel].state = FFT_IDLE;
    fft_result[channel].is_target_freq_max = false;
    fft_result[channel].max_freq = 0;
    fft_result[channel].max_magnitude = 0;
}

/*---------------------------------------------------------------------------
 Name        : fft_detect_all_channels
 Input       : detected_channels - 输出参数,检测到的通道掩码
 Output      : 检测到的通道数量
 Description : 检测所有CT通道,返回检测到目标频率的通道
---------------------------------------------------------------------------*/
uint8_t fft_detect_all_channels(uint8_t *detected_channels)
{
    uint8_t count = 0;
    *detected_channels = 0;

    for (int ch = 0; ch < CT_CHANNEL_MAX; ch++)
    {
        if (fft_result[ch].state == FFT_COMPLETE &&
            fft_result[ch].is_target_freq_max)
        {
            *detected_channels |= (1 << ch);
            count++;
        }
    }

    return count;
}

/*---------------------------------------------------------------------------
 Name        : fft_reset_all_channels
 Input       : 无
 Output      : 无
 Description : 复位所有通道的FFT模块
---------------------------------------------------------------------------*/
void fft_reset_all_channels(void)
{
    for (ct_channel_t ch = CT_CHANNEL_1; ch < CT_CHANNEL_MAX; ch++)
    {
        fft_result[ch].sample_count = 0;
        fft_result[ch].state = FFT_IDLE;
        fft_result[ch].max_freq = 0.0f;
        fft_result[ch].max_magnitude = 0;
        fft_result[ch].is_target_freq_max = false;
    }
}

/*---------------------------------------------------------------------------
 Name        : fft_check_and_analyze
 Input       : 无
 Output      : 无
 Description : 检查并分析所有通道FFT结果(在主循环中调用)
---------------------------------------------------------------------------*/
void fft_check_and_analyze(void)
{
    static uint8_t analysis_pending = 0; // bit0=CT1, bit1=CT2, bit2=CT3

    // 前置条件检查 - 快速返回以避免无谓运算
    if (sys_param.state != SYS_NORMAL_RUN || sys_param.grid.phase_id.sequence_k == 0 || sys_param.fft_identify.is_ffting == 0)
        return;

    // 检查各通道采集状态，只有当有通道完成采集时才进行位操作
    for (ct_channel_t ch = CT_CHANNEL_1; ch < CT_CHANNEL_MAX; ch++)
    {
        if (fft_result[ch].state == FFT_READY)
            analysis_pending |= (1 << ch);
    }

    // 所有通道都准备好后统一分析
    if (analysis_pending != 0x07) // 0b111 = CT1+CT2+CT3
        return;

    // 以下是所有通道都准备好后的分析逻辑
#ifdef FFT_DEBUG_PRINT
    printf("\n===3 CT collections completed and FFT analysis was conducted (consecutive success count: %d)===\r\n", sys_param.fft_identify.consecutive_success_count);
#endif

    // 分析所有通道
    for (ct_channel_t ch = CT_CHANNEL_1; ch < CT_CHANNEL_MAX; ch++)
    {
        fft_analyze_power_spectrum_3ch(ch);
    }

    // 统一检测结果
    uint8_t detected = 0;
    uint8_t count = fft_detect_all_channels(&detected);
    analysis_pending = 0;

#ifdef FFT_DEBUG_PRINT
    printf(" 检测结果:\r\n");
    printf("  CT1: %s\r\n", (detected & 0x01) ? "检测到目标频率" : "未检测到");
    printf("  CT2: %s\r\n", (detected & 0x02) ? "检测到目标频率" : "未检测到");
    printf("  CT3: %s\r\n", (detected & 0x04) ? "检测到目标频率" : "未检测到");
    printf("  共%d个通道检测到\r\n\n", count);
#endif

    // count==0：没有CT检测到目标频率，重置连续成功计数
    if (count == 0)
    {
        sys_param.fft_identify.consecutive_success_count = 0;
        sys_param.fft_identify.last_identified_ct = 0;

#ifdef FFT_DEBUG_PRINT
        printf("[FFT]Identification failed (detected 0 CTs), reset consecutive success count, wait for 5 seconds then retry\r\n");
#endif

        sys_param.fft_identify.enable_collect = 0;
        sys_param.fft_identify.is_ffting = 0;
        sys_param.fft_identify.resend_cmd = true;
        sys_param.fft_identify.boardcast_interval = 5000;
        fft_reset_all_channels();
        return;
    }

    // count>=1：至少1个CT检测到目标频率
    uint8_t current_ct = 0;

    if (count == 1)
    {
        // 只有1个CT，直接确定
        if (detected & 0x01)
            current_ct = 1;
        else if (detected & 0x02)
            current_ct = 2;
        else if (detected & 0x04)
            current_ct = 3;
    }
    else
    {
        // 多个CT都检测到目标频率，选幅值最大的，不重置连续成功计数
        uint32_t best_mag = 0;
        for (int ch = 0; ch < CT_CHANNEL_MAX; ch++)
        {
            if ((detected & (1 << ch)) && fft_result[ch].max_magnitude > best_mag)
            {
                best_mag = fft_result[ch].max_magnitude;
                current_ct = (uint8_t)(ch + 1);
            }
        }
#ifdef FFT_DEBUG_PRINT
        printf("[FFT]Multiple CTs detected target freq (count=%d), select CT%d by max magnitude=%u\r\n",
               count, current_ct, best_mag);
#endif
    }

    {
        // 检查是否与上次识别的CT相同
        if (current_ct == sys_param.fft_identify.last_identified_ct)
        {
            // 相同，增加连续成功次数
            sys_param.fft_identify.consecutive_success_count++;
        }
        else
        {
            // 不同，重置连续成功次数
            sys_param.fft_identify.consecutive_success_count = 1;
            sys_param.fft_identify.last_identified_ct = current_ct;
        }

#ifdef FFT_DEBUG_PRINT
        printf("检测到CT%d，连续成功%d次 (需要4次)\r\n",
               current_ct, sys_param.fft_identify.consecutive_success_count);
#endif

        // 达到4次连续成功才算真正识别成功
        if (sys_param.fft_identify.consecutive_success_count >= 4)
        {
            // 检查识别到的CT是否在线
            ct_param_t *identified_ct_ptr = NULL;
            if (current_ct == 1)
                identified_ct_ptr = &sys_param.ct1;
            else if (current_ct == 2)
                identified_ct_ptr = &sys_param.ct2;
            else if (current_ct == 3)
                identified_ct_ptr = &sys_param.ct3;

            // 如果识别到的CT不在线，重置识别结果
            if (identified_ct_ptr != NULL &&
                identified_ct_ptr->status.connect_status != CT_STATUS_ONLINE)
            {
#ifdef FFT_DEBUG_PRINT
                printf("[FFT]Warning: The identified CT%d is not online. Ignore this recognition result\r\n", current_ct);
#endif
                // 重置连续成功计数
                sys_param.fft_identify.consecutive_success_count = 0;
                sys_param.fft_identify.last_identified_ct = 0;

                // 停止当前采集和识别
                sys_param.fft_identify.enable_collect = 0;
                sys_param.fft_identify.is_ffting = 0;
                sys_param.fft_identify.resend_cmd = true;

                // 设置5秒等待后重新发送命令
                sys_param.fft_identify.boardcast_interval = 5000;

                fft_reset_all_channels();
                return; // 提前返回，不发送相位信息
            }

            // CT在线，识别有效
            sys_param.fft_identify.identified_ct = current_ct;

#ifdef FFT_DEBUG_PRINT
            printf("[FFT]Identification successful: CT%d (consecutive 4 confirmations and online), wait for 5 seconds to send phase information\r\n",
                   sys_param.fft_identify.identified_ct);
#endif

            // 停止采集和识别
            sys_param.fft_identify.enable_collect = 0;
            sys_param.fft_identify.is_ffting = 0; // 识别完成
            sys_param.fft_identify.resend_cmd = false;

            // 设置5秒等待，等待后发送最终确认
            sys_param.fft_identify.boardcast_interval = 5000;
            sys_param.fft_identify.final_confirm_pending = true;

            fft_reset_all_channels();
        }
        else
        {
            // 未达到4次，继续下一轮检测

#ifdef FFT_DEBUG_PRINT
            printf("[FFT]Wait for 5 seconds then continue next round of identification\r\n");
#endif

            // 停止当前采集和识别
            sys_param.fft_identify.enable_collect = 0;
            sys_param.fft_identify.is_ffting = 0; // 本轮结束
            sys_param.fft_identify.resend_cmd = true;

            // 设置5秒等待后重新发送命令
            sys_param.fft_identify.boardcast_interval = 5000;

            fft_reset_all_channels();
        }
    }
}
