#ifndef FFT_H
#define FFT_H

#include <stdint.h>
#include <stdbool.h>

// FFT 参数配置
#define N_FFT 512            // FFT点数
#define LOG2_N 9             // log2(512) = 9
#define FFT_SAMPLE_RATE 50.0f // 采样频率: 50Hz (20ms采样)
#define FFT_BIN_COUNT 103     // 4~14Hz bin数量
#define FFT_FREQ_MIN 4.0f     // FFT分析最低频率 (Hz)
#define FFT_FREQ_MAX 14.0f    // FFT分析最高频率 (Hz)
// FFT相序识别间隔时间n的有效值: 4, 6, 8, 10, 12
// 对应的理论频率: n=4->12.5Hz, n=6->8.33Hz, n=8->6.25Hz, n=10->5Hz, n=12->4.17Hz
// 分析频率范围: 4-14Hz

// Q15 定点数格式宏
#define Q15(x) ((int16_t)((x) * 32767.0f))
#define DEQ15(x) ((float)(x) / 32767.0f)

// CT通道
typedef enum
{
    CT_CHANNEL_1 = 0,
    CT_CHANNEL_2 = 1,
    CT_CHANNEL_3 = 2,
    CT_CHANNEL_MAX = 3
} ct_channel_t;

// FFT状态
typedef enum
{
    FFT_IDLE = 0,
    FFT_COLLECTING,
    FFT_READY,
    FFT_ANALYZING,
    FFT_COMPLETE
} fft_state_t;

// FFT结果
typedef struct
{
    bool is_target_freq_max;
    float max_freq;
    uint32_t max_magnitude;
    uint16_t sample_count;
    fft_state_t state;
} fft_result_t;

void fft_3ch_init(void);
bool fft_collect_power_data_3ch(ct_channel_t channel, float power_value);
bool fft_analyze_power_spectrum_3ch(ct_channel_t channel);

void fft_reset_3ch(ct_channel_t channel);
uint8_t fft_detect_all_channels(uint8_t *detected_channels);
void fft_reset_all_channels(void);
void fft_check_and_analyze(void);

#endif
