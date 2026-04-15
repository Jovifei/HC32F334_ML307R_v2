/*lic*/
#ifndef __MAIN_H__
#define __MAIN_H__

#include "hc32_ll.h"
#include <stdio.h>
#include <stdbool.h>


#define BOARDCAST_TIME 2 // 广播时间间隔，单位：电网周期数（2个周期后广播，50Hz=40ms，60Hz=33ms）

// #define DEBUG_ENABLE // 调试功能开关，0为关闭，1为开启
// #define FFT_DEBUG_PRINT (1)

// ================= 允许打印宏 =================
// #define PRINTF_ENABLE

#ifdef PRINTF_ENABLE
#define DEBUG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...) ((void)0)
#endif


// 4G妯″潡璋冭瘯鏃ュ織
#define DEBUG_4G_ENABLE

#ifdef DEBUG_4G_ENABLE
#define DEBUG_4G_PRINTF(fmt, ...) printf("[ML307R]" fmt, ##__VA_ARGS__)
#else
#define DEBUG_4G_PRINTF(fmt, ...) ((void)0)
#endif


// ================= 版本及SN =================
#define SN_LENGTH 15              // 微逆和CTSN长度15
#define VERSION_STRING_MAX_LEN 10 // 版本号字符串最大长度
#define SW_VERSION "V1.1.12"      // 初始版本
#define HW_VERSION "V1.0.0"       // 初始版本

// ================= 故障类参数阈值 =================
#define FAULT_DELAY_S (10)          // 故障恢复延时时间(秒)
#define FAULT_TH_V1P65_LOW (1.55f)  // 1.65V参考电压故障下限阈值(V)
#define FAULT_TH_V1P65_HIGH (1.75f) // 1.65V参考电压故障上限阈值(V)
#define FAULT_TH_AC_V_HIGH (380.0f) // AC交流电压故障上限阈值(V_瞬时值)
#define FAULT_TH_AC_V_LOW (176.0f)  // AC交流电压故障下限阈值(V_有效值)
#define FAULT_TH_CT_I_HIGH (60.0f)  // CT电流故障上限阈值(A_瞬时值)
#define FAULT_CONFIRM_COUNT (800)   // 故障确认次数阈值，连续检测次数(40ms)

// ================= 有效值计算参数配置 =================
#define NOMINAL_VOLTAGE (220.0f)  // 标称电压(V_RMS)
#define ADC_SAMPLE_PERIOD_US (50) // ADC采样周期 (us)

// 自适应50Hz/60Hz：缓冲区静态上限，覆盖45Hz最坏情况(444点)，留余量
// 50Hz: 400点/周期  60Hz: 333点/周期  45Hz(最慢): 444点/周期
#define MAX_SAMPLES_PER_CYCLE (500) // 静态数组最大容量
#define TOTAL_SAMPLES (MAX_SAMPLES_PER_CYCLE)

// ================= 过零检测计算参数配置 =================
// 过零检测相关定义
#define ZERO_CROSS_THRESHOLD (0.0f)   // 过零检测阈值(V)
#define ZERO_CROSS_COUNT_TARGET (300) // 需要检测的过零次数300*20ms=6s
#define GRID_FREQ_MIN (45.0f)         // 最小电网频率(Hz)
#define GRID_FREQ_MAX (65.0f)         // 最大电网频率(Hz)

// 根据电网频率范围和采样周期，计算每个周期的采样点数范围
#define GRID_FREQ_SAMPLES_MAX ((int)(1000000.0f / (GRID_FREQ_MIN * ADC_SAMPLE_PERIOD_US) + 0.5f)) // 45Hz对应的最大采样点数
#define GRID_FREQ_SAMPLES_MIN ((int)(1000000.0f / (GRID_FREQ_MAX * ADC_SAMPLE_PERIOD_US) + 0.5f)) // 65Hz对应的最小采样点数

#define GRID_HALF_PERIOD_MIN (100u) // 过零检测的半周期不检测过零
#define FREQ_AVG_CYCLES (8u)        // N个周期计算频率的平均频率

// ================= CT在线检测参数定义 ================
// CT在线检测功率阈值(W) - 可根据实际需求调整
#define CT_ONLINE_POWER_THRESHOLD (15.0f)    // CT在线判断功率阈值
#define CT_OFFLINE_POWER_THRESHOLD (10.0f)   // CT离线判断功率阈值
#define CT_PHASE_IDE_POWER_THRESHOLD (60.0f) // CT相序识别功率阈值

// CT在线/离线/相序识别时电流阈值(A_RMS)
#define CT_ONLINE_THRESHOLD (CT_ONLINE_POWER_THRESHOLD / (NOMINAL_VOLTAGE))            // CT在线判断电流阈值
#define CT_OFFLINE_THRESHOLD (CT_OFFLINE_POWER_THRESHOLD / (NOMINAL_VOLTAGE))          // CT离线判断电流阈值
#define CT_PHASE_IDENTIFY_THRESHOLD (CT_PHASE_IDE_POWER_THRESHOLD / (NOMINAL_VOLTAGE)) // 相序识别功率阈值

// CT在线/离线判断计数阈值
#define CT_OFFLINE_COUNT_THRESHOLD (100) // 离线判断：连续N次低于阈值(次数)
#define CT_ONLINE_COUNT_THRESHOLD (50)   // 在线判断：连续N次高于阈值(次数)

// ================= 相序识别参数定义 ==================
#define PHASE_IDENTIFY_MAX_ATTEMPTS (40)    // 相序识别最大尝试次数
#define PHASE_IDENTIFY_CONSISTENT_COUNT (6) // 相序识别连续一致次数

// ================= 微逆识别参数定义 ==================
#define FFT_IDENTIFY_MAX_RETRY (1) // FFT相位识别最大重试次数

// ================= 系统时序管理常量 =================
#define TIMER_1MS_CYCLES (20)  // 1ms对应的50us周期数
#define TIMER_20mS_CYCLES (20) // 20ms对应的1ms周期数
#define TIMER_1S_CYCLES (1000) // 1s对应的1ms周期数
#define POWER_2S_CYCLES (100)  // 2s对应的电网周期数

// ================= LED状态机常量 =================
#define LED_BLINK_SLOW_PERIOD (1000)       // 慢闪周期(ms)
#define LED_BLINK_FAST_PERIOD (200)        // 快闪周期(ms)
#define LED_BLINK_ALTERNATING_PERIOD (600) // 交替闪烁周期(ms)
#define LED_ON_RATIO (0.5f)                // LED交替亮的时间比例

// 缓冲区大小定义
#define COM_RX_BUFFER_SIZE 256 // 接收缓冲区大小
#define COM_TX_BUFFER_SIZE 256 // 发送缓冲区大小
#define INV_REPORT_51_SIZE 20  // 51数据上报最大长度

// ================= 连接多sub1g设备 =================
#define INV_DEVICE_MAX_NUM 8                      // 最大设备数量
#define USER_PAIR_LIST_MAX_NUM INV_DEVICE_MAX_NUM // 用户配对列表最大数量
#define PRODUCT_MODEL_MAX_LEN 10                  // 产品型号最大长度

// ================= 微逆设备SIID和PIID定义 =================
// INV设备的SIID范围: 4-11
#define SIID_MIN 4  // 微逆设备SIID起始值
#define SIID_MAX 11 // 微逆设备SIID结束值

// 产品型号定义，后续可扩展其他产品型号...
#define PRODUCT_MODEL_CODE_MI800S 1  // GE-MI800S
#define PRODUCT_MODEL_CODE_MI2500S 2 // GE-MI2500S

// ================= 未配对设备列表相关定义 =================
#define UNPAIRED_DEVICE_MAX_NUM INV_DEVICE_MAX_NUM // INV请求配对列表最大数量
#define UNPAIRED_DEVICE_TIMEOUT_MS 50000           // INV请求配对超时时间(50秒)
#define PAIRED_INV_ONLINE_TIMEOUT_S 180            // 已经配对微逆，是否在线判断的超时时间(60秒)，单位s
#define SWITCH_INV_BOARCAST 1                      // 同一个微逆广播几次，切换到下一个微逆

// ================= 避免UART1串口的冲突定义环形缓存区 =================
#define UART1_TX_QUEUE_SIZE 2
#define UART1_TX_MSG_MAX_LEN 100

// 微逆设备属性PIID定义
#define INV_PIID_ONLINE_STATE 1      // 在线状态 (uint8_t) 0未配对，1配对未在线，2在线
#define INV_PIID_DEVICE_SN 2         // 设备序列号(char)
#define INV_PIID_SW_VERSION 3        // 软件版本(char)
#define INV_PIID_SUB1G_VERSION 4     // 微逆sub1g版本(char)
#define INV_PIID_PRODUCT_MODEL 5     // 产品型号(char)
#define INV_PIID_WORK_STATE 6        // 工作状态(uint8_t)
#define INV_PIID_GRID_POWER 7        // 发电功率(float)
#define INV_PIID_TODAY_ENERGY 8      // 今日发电量(float)
#define INV_PIID_LIFETIME_ENERGY 9   // 累计发电量(float)
#define INV_PIID_ANTIFLOW_ENABLE 10  // 防逆流开关(bool)
#define INV_PIID_INV_POWER_ENABLE 11 // 发电开关(bool)
#define INV_PIID_TODAY_POWER_TIME 12 // 今日发电时长(float)
#define INV_PIID_FAULT_PARAM 13      // 故障参数(uint32_t)
#define INV_PIID_TEMPERATURE 14      // 微逆内部温度(float)
#define INV_PIID_PV1_VOLTAGE 16      // PV1平均电压(float)
#define INV_PIID_PV1_CURRENT 17      // PV1平均电流(float)
#define INV_PIID_PV2_VOLTAGE 18      // PV2平均电压(float)
#define INV_PIID_PV2_CURRENT 19      // PV2平均电流(float)
#define INV_PIID_GRID_VOLTAGE 20     // 电网电压(float)
#define INV_PIID_GRID_FREQUENCY 21   // 电网频率(float)
#define INV_PIID_POWER_PHASE 22      // 所在的相序(uint8_t)
#define INV_PIID_POWER_LIMIT 24      // 发电功率限制(uint32_t) 瓦特
#define INV_PIID_CONNECTION_POINT 25 // 接入点(uint8_t) 0:配电箱 1:末端插座
#define INV_PIID_PV1_POWER 26        // PV1功率(uint16_t) 瓦特 只读
#define INV_PIID_PV2_POWER 27        // PV2功率(uint16_t) 瓦特 只读
#define INV_PIID_PV3_POWER 28        // PV3功率(uint16_t) 瓦特 只读
#define INV_PIID_PV4_POWER 29        // PV4功率(uint16_t) 瓦特 只读
// #define INV_PIID_PV5_POWER 30        // PV5功率(uint16_t) 瓦特 只读
// #define INV_PIID_PV6_POWER 31        // PV6功率(uint16_t) 瓦特 只读
#define INV_PIID_FFT_ENABLE 32       // FFT相位识别开关(bool) 只读
#define INV_PIID_PV_NUM 33           // PV个数 只读
#define INV_PIID_SUBG_ADDR 34        // Sub1g地址
#define INV_PIID_CHANNEL_INDEX 35    // Sub1g工作信道索引(uint8_t) 只读
#define INV_PIID_PACKET_LOSS_RATE 37 // 丢包率(uint8_t) 0-100% 只读

// ================= 枚举定义 =================

// CT是否卡在线上定义
typedef enum
{
    CT_STATUS_UNKNOWN = 0, // 未知状态（初始化阶段）
    CT_STATUS_OFFLINE,     // 离线（未检测到足够电流）
    CT_STATUS_ONLINE       // 在线（检测到足够电流）
} ct_online_t;

// 系统运行状态机定义
typedef enum
{
    SYS_INIT = 0,         // 初始化，检测电网过零和频率
    SYS_WAIT_CT,          // 等待CT全部插入
    SYS_PHASE_IDENTIFY,   // 执行相序识别
    SYS_POWER_DIR_DETECT, // 功率方向检测
    SYS_NORMAL_RUN,       // 正常运行，计算功率和防逆流
    SYS_FREQ_FAULT,       // 电网频率故障，超出45Hz-65Hz范围
} sys_run_t;

// LED状态定义
typedef enum
{
    LED_STATE_OFF = 0,          // 灯灭
    LED_STATE_RED_ON,           // 红灯常亮
    LED_STATE_GREEN_ON,         // 绿灯常亮
    LED_STATE_RED_BLINK_SLOW,   // 红灯慢闪
    LED_STATE_RED_BLINK_FAST,   // 红灯快闪
    LED_STATE_GREEN_BLINK_SLOW, // 绿灯慢闪
    LED_STATE_GREEN_BLINK_FAST, // 绿灯快闪
    LED_STATE_ALTERNATING_SLOW, // 红绿交替慢闪
    LED_STATE_ALTERNATING_FAST  // 红绿交替快闪
} led_state_t;

// ================= 系统标志位管理结构体 =================

// 任务执行标志位结构体
typedef struct
{
    uint8_t ct_phase_identify_ready; // 相序识别就绪
    uint8_t fault_check_ready;       // 故障检查就绪
    uint8_t state_machine_ready;     // 状态机处理就绪
    uint8_t power_calc_ready;        // 功率计算就绪（每个电网周期置位一次）
    uint8_t power_cycle_ready;       // 功率周期完成标志
} task_flags_t;

// 系统标志位管理器
typedef struct
{
    uint8_t rms_calc_ready;      // 有效值计算完成
    uint8_t zero_cross_detected; // 过零检测完成
    uint8_t timer_1ms_flag;      // 1ms定时器标志
    uint8_t timer_1s_flag;       // 1s定时器标志
    uint8_t timer_10s_flag;      // 10s定时器标志
    uint8_t timer_20ms_flag;     // 20ms定时器标志

    task_flags_t task; // 任务标志位
} system_flags_t;

// ================= 数据结构定义 =================

// 故障信息结构体
typedef union
{
    struct
    {
        uint32_t v1p65_sample : 1;
        uint32_t ac_sample : 1;
        uint32_t ct1_sample : 1;
        uint32_t ct2_sample : 1;
        uint32_t ct3_sample : 1;
        uint32_t sub1g_comm : 1;
        uint32_t grid_frequency : 1; // 电网频率超范围故障 (45Hz-65Hz)

    } bit;
    uint32_t data;
} fault_t;

// CT是否在线检测结构体
typedef struct
{
    uint16_t offline_count;     // 离线计数器
    uint16_t online_count;      // 在线计数器
    ct_online_t connect_status; // 当前连接状态
} ct_online_detect_t;

// 功率计算结构体
typedef struct
{
    uint8_t power_ready; // 功率计算完成标志

    // 计数器
    uint16_t power_sample_count; // 功率采样计数器（瞬时功率采样点数）

    // 功率计算值
    float sum_power;        // 瞬时功率累积和 (W·采样点)
    float avg_power;        // 单周期平均有功功率
    float fix_dir_power;    // 修正方向后的功率值 (W)
    int8_t power_direction; // 功率方向，正为1，负为-1
    float power_factor;     // 功率因数 PF = P / (V_rms * I_rms)，范围 [-1, 1]
    float ct_sub1g_boardcast_power_avg;

    // 功率方向检测字段
    uint8_t direction_detect_complete; // 功率方向检测完成标志
    uint16_t direction_sample_count;   // 已收集样本数量 (0-50)
    float direction_power_sum;         // 功率样本累积和
} power_calc_t;

// 过零检测结构体
typedef struct
{
    float last_voltage;          // 上一次电压瞬时值 (V)
    uint8_t zero_cross_detected; // 过零检测标志
    uint8_t positive_zero_cross; // 正向过零标志（从负到正）

    uint8_t frequency_valid; // 电网频率有效标志

    uint32_t zero_cross_count;  // 过零计数（用于初始化状态）
    uint32_t zero_sample_count; // 半周过零间隔采样计数

    // 正/负半周分别计时，在负向过零时合并为完整周期
    // 两个半周延迟相加可完全抵消 LPF 引入的过零点偏移
    uint32_t half_period;      // 本半周采样点数（正向过零时更新负半周，负向过零时更新正半周）
    uint32_t last_half_period; // 上一个半周采样点数
    uint8_t position;          // 当前半周位置：0=负半周（等待正向过零），1=正半周（等待负向过零）

    uint32_t period_samples; // 单周期采样点数（每负向过零更新，供 samples_per_cycle 使用）

    // 中断内只做整数加法，主循环做除法和浮点运算
    uint32_t period_accum;    // 累计周期采样点数
    uint8_t period_avg_count; // 已累计周期数
    uint8_t period_updated;   // 新周期就绪：中断置1，grid_task 读后清0
} zero_cross_detect_t;

// 相序识别结果
typedef struct
{
    uint8_t sequence_k;           // 相序识别完成后的最佳相序组合编号1-6哪种组合：方便调试后期删除
    uint8_t ct_to_phase[3];       // CT到相的映射关系 [0]=CT1对应的相, [1]=CT2对应的相, [2]=CT3对应的相
    uint8_t identification_valid; // 识别结果有效标志

    float matching_degree[6]; // 6种组合的匹配度：方便调试后期删除
    float power_factor[3];    // 三相功率因数 [0]=Pf1, [1]=Pf2, [2]=Pf3

    // 多次识别确认相关
    uint8_t identify_history[10]; // 记录最近10次识别结果（1-6）
    uint8_t identify_count;       // 当前已识别次数
    uint8_t consistent_count;     // 一致的识别次数
    uint8_t total_attempts;       // 相序识别总尝试次数

    // 继电器打开定时器
    uint16_t relay_open_timer_ms;  // 继电器打开定时器(ms)，方向检测完成后持续2秒广播打开继电器
    uint8_t relay_opening_pending; // 继电器打开等待标志，1=正在打开继电器
} phase_identify_t;

// 电网管理结构体
typedef struct
{
    zero_cross_detect_t zero_cross; // 过零检测
    phase_identify_t phase_id;      // 相序识别结果
    uint8_t ct_connected;           // 所有CT都连接标志
    uint8_t online_ct_count;        // 在线CT数量(0-3)
    uint8_t system_type_changed;    // 系统类型变化标志(单相/三相切换)
    uint8_t frequency_fault;        // 频率故障标志：1=频率超出45-65Hz范围

    // 电网参数
    float ua_vol_rms;     // A相电压有效值 (V_RMS)
    float grid_frequency; // 电网频率 (Hz)

    // 自适应50Hz/60Hz运行时参数 —— 每个正向过零点更新一次
    uint16_t samples_per_cycle;     // 本周期实测采样点数 (初始=400)
    uint16_t phase_b_delay_samples; // B相重构延迟点数 = samples_per_cycle/3  (初始=133)
    uint16_t phase_c_delay_samples; // C相重构延迟点数 = samples_per_cycle*2/3 (初始=267)
} grid_manager_t;

// FFT识别管理结构体
typedef struct
{
    uint8_t enable_collect; // 使能FFT采集
    uint8_t is_ffting;      // FFT分析中标志
    uint8_t interval_time;  // 相序识别间隔时间(n值)
    uint8_t identified_ct;  // 识别到的CT号(1/2/3)
    uint16_t power;         // 基础功率值
    uint32_t sub1g_addr;    // 目标微逆地址
    bool resend_cmd;        // 重发命令标志
    uint8_t retry_flag;     // 等待2秒期间100ms重发补偿标志

    // 连续识别验证
    uint8_t consecutive_success_count; // 连续成功次数(需达到4次)
    uint8_t last_identified_ct;        // 上次识别到的CT号
    uint16_t boardcast_interval;       // 广播间隔计数器(ms)
    bool final_confirm_pending;        // 4次成功后等待发送确认标志
} fft_identify_manager_t;

// CT参数结构体
typedef struct
{
    float rms_value;           // CT电流有效值 (A_RMS) - 基于400个采样点计算
    power_calc_t power;        // 功率计算管理
    ct_online_detect_t status; // 在线状态检测

    float inv_power;         // 微逆发电功率：W
    float use_power;         // 在用电功率：W
    float power_consumption; // 统计用电量：Wh
    uint8_t inv_device_cnt;  // 正在发电的微逆个数
} ct_param_t;

// 信号采样结构体
typedef struct
{
    // ADC采样参数
    uint16_t adc1_raw[6];     // ADC原始采样值
    uint16_t adc1_raw_LPF[6]; // 滤波后的ADC原始值

    // ADC采样后数值处理参数
    float ac_voltage;        // AC电压值
    float ac_voltage_LPF;    // 滤波后的AC电压值
    float ct1_current;       // CT1电流值
    float ct1_current_LPF;   // 滤波后的CT1电流值
    float ct2_current;       // CT2电流值
    float ct2_current_LPF;   // 滤波后的CT2电流值
    float ct3_current;       // CT3电流值
    float ct3_current_LPF;   // 滤波后的CT3电流值
    float v1p65_voltage;     // V1.65电压值
    float v1p65_voltage_LPF; // 滤波后的V1.65电压值
} signal_data_t;

// 人机交互结构体
typedef struct
{
    // LED状态机
    led_state_t led_state; // 当前LED状态
    uint32_t led_timer;    // LED定时器
    uint32_t led_count;    // LED计数
    uint8_t led_phase;     // LED相位
    uint32_t display_timer_ms;
    uint8_t display_receive_flag;
} mmi_t;

// 系统定时器结构体
typedef struct
{
    uint16_t timer_1ms_count;  // 1ms定时器计数
    uint16_t timer_1s_count;   // 1s定时器计数
    uint16_t timer_20ms_count; // 10ms定时器计数
    uint16_t debug_1ms_count;  // 调试计数
} system_timer_t;

typedef struct
{
    uint8_t state;  // 0-不在线，1-在线
    uint16_t power; // 0-650W
    float voltage;  // 0-100V,一位小数
    float current;  // 0-100A,一位小数
} pv_state_t;

// 0x51命令数据缓存结构
typedef struct
{
    uint8_t valid;                    // 数据有效标志
    uint8_t inv_index;                // 对应的微逆索引(0-7)8个微逆
    uint8_t data[INV_REPORT_51_SIZE]; // 原始数据缓存
    uint8_t data_len;                 // 实际数据长度
} inv_0x51_report_t;

// 存储已匹配的微逆设备的信息
typedef struct
{
    // 基础信息_存EEPROM
    uint8_t siid;                                  // 服务实例ID (4-11)
    uint8_t phase;                                 // 微逆所在相位 (1字节, 0=未识别, 1=A相, 2=B相, 3=C相)
    uint32_t sub1g_addr;                           // 微逆sub1g的地址
    char device_sn[SN_LENGTH + 1];                 // 微逆的SN // piid:2
    char product_model[PRODUCT_MODEL_MAX_LEN + 1]; // 产品型号 // piid:5

    // 缓存微逆信息
    char sw_version[VERSION_STRING_MAX_LEN + 1]; // 软件版本号 // piid:3
    // char hw_version[VERSION_STRING_MAX_LEN + 1];    // 硬件版本号 // piid:4
    char sub1g_version[VERSION_STRING_MAX_LEN + 1]; // sub1g版本号

    // 状态信息
    uint8_t online_state;      // 在线状态(0：没有配对的微逆, 1：不在线，2：在线) // piid:1
    uint8_t work_state;        // 工作状态 // piid:6
    uint8_t connection_point;  // 连接点: 0：配电箱1：末端插座
    float grid_power;          // 发电功率(W) // piid:7
    float today_power_time;    // 今日发电时长(h) // piid:8
    float today_energy;        // 今日发电量(Wh) // piid:9
    float lifetime_energy;     // 累计发电量(Wh) // piid:10
    float ambient_temperature; // 微逆内部温度
    uint8_t antiflow_enable;   // 微逆防逆流开关状态
    uint8_t power_enable;      // 微逆发电开关状态

    uint16_t power_limit; // 发电功率限制 // piid 24
    float grid_frequency; // 电网频率(Hz) // piid:15
    float grid_voltage;   // 电网电压(V) // piid:16
    uint32_t fault_param; // 故障参数 // piid:17

    // 设备有效性标志
    bool is_valid;         // 设备槽位是否有效(是否已绑定设备)
    bool prop_changed;     // 属性变化标志 - 用于立即上报
    bool settings_changed; // 设置变化标志 - 用于立即上报

    // PV参数
    pv_state_t pv[4];
    uint8_t pv_num;        // 微逆连接的PV个数
    int inv_rssi;          // 微逆接收的rssi
    uint8_t channel_index; // 微逆sub1g工作信道索引 // piid:35

    // 通信统计信息
    uint16_t stats_time_sec; // 统计时间计数器(秒)
    uint16_t rx_0x50_count;  // 0x50接收次数
    uint16_t rx_0x52_count;  // 0x52接收次数
    uint16_t rx_0x54_count;  // 0x54接收次数
    uint16_t rx_0x55_count;  // 0x55接收次数
    uint16_t rx_0x56_count;  // 0x56接收次数
    uint16_t rx_0x57_count;  // 0x57接收次数
    uint8_t plr;             // 丢包率 (0-100)
    bool stats_started;      // 统计是否已开始

    // 非上报后台参数
    uint16_t offline_updata_ms; // 收到在线状态的时间，判断离线
} inv_device_t;

// 用户配对请求结构体（用户通过APP指定要配对的设备SN，保存下来的）
typedef struct
{
    char device_sn[SN_LENGTH + 1]; // 设备SN
    bool is_valid;                 // 是否有效
} user_pair_request_t;

// 未配对设备信息结构体
typedef struct
{
    uint32_t sub1g_addr;           // SUB1G地址
    char device_sn[SN_LENGTH + 1]; // 设备SN (15字节+结束符)
    uint8_t product_model;         // 产品型号 (1=800W微逆)
    uint32_t unpaired_updata_ms;   // 最后收到广播的时间戳(ms)
    uint16_t paired_unvalid_ms;
    bool is_valid; // 槽位是否有效
} unpaired_device_t;

typedef struct
{
    // 详细的时间
    char date_time[20];
    // 日期
    char date[11];
    // 日期时间
    char time[9];
    // 当前存储的日号(1-31),用于检测日期变更
    uint8_t today_date;

} TimeType;

typedef struct
{
    uint8_t state;         // 0-不在线，1-在线
    char short_sn[7];      // sn后6位
    int8_t comm_rssi;      // 通信信号强度
    uint8_t comm_plr;      // 通信丢包率
    uint8_t power_en;      // 功率开关： 0-关闭，1-开启
    uint16_t power_limit;  // 功率限制：0-2500W
    uint8_t power_mode;    // 工作模式：0-防逆流发电，1-自由发电，2-限流发电
    uint16_t power_output; // 输出功率：0-2500W
    // pv_state_t pv[4];     // 4块光伏板
} inv_state_t;

typedef struct
{
    uint32_t electricity_generation;     // 发电总量wh
    uint32_t electricity_consumption;    // 用电总量wh
    int16_t power_l1;                    // ct1相功率w
    int16_t power_l2;                    // ct2相功率w
    int16_t power_l3;                    // ct3相功率w
    uint8_t subg_state;                  // subg状态0-无配对设备，1-有配对设备不在线，2-有配对设备在线
    uint8_t subg_connected;              // subg连接设备数量0-8
    uint8_t subg_disconnected;           // subg断开设备数量0-8
    uint8_t subg_discovered;             // subg发现设备数量0-8
    uint8_t subg_antibackflow;           // subg防逆流状态0-关闭 1-开启正常，2-开启有逆流
    inv_state_t inv[INV_DEVICE_MAX_NUM]; // 8个微逆状态
} hmi_param_t;

// ================= 主系统参数结构体 =================
typedef struct
{
    // 系统管理
    system_flags_t flags; // 系统标志位管理器
    system_timer_t timer; // 系统定时器
    sys_run_t state;      // 系统运行当前状态

    // 信号和故障
    signal_data_t signal;  // 信号采样数据
    fault_t fault;         // 故障信息
    uint16_t fault_result; // 故障结论
    uint16_t fault_delay;  // 故障延时

    grid_manager_t grid; // 电网管理器

    // CT参数
    ct_param_t ct1; // CT1参数
    ct_param_t ct2; // CT2参数
    ct_param_t ct3; // CT3参数

    // 微逆管理
    fft_identify_manager_t fft_identify; // fft分析微逆

    // 人机交互
    mmi_t mmi; // 人机交互

    hmi_param_t hmi;

    // wifi获取的真实时间
    TimeType time;

    // wifi通信参数
    struct
    {
        uint16_t restore_wifi_cmd;
        uint16_t clear_inverter_data_cmd;
        uint8_t clear_inv_step_count; // 步骤计数器(0-39, 共40步=8设备*5轮)
    } wifi;

    struct
    {
        uint8_t state;          // 0-模块未配置，1-模块未配对设备，2-未发现已配对设备，3-配对设备已连接但通信不畅，4-从设备通信正常
        uint16_t timeout_count; // 通信超时计数器(ms)
        uint16_t reboot_count;  // 重启计数器(ms)

        uint32_t ct_sub1g_addr;                      // ct的sub1g的地址
        char sw_version[VERSION_STRING_MAX_LEN + 1]; // ct的sub1g的版本号
        int8_t rssi;                                 // ct的sub1g的RSSI值
        uint8_t channel_index;                       // ct的sub1g工作信道索引

        uint16_t version_timer_ms; // 获取版本定时器(ms)
        uint16_t rssi_timer_ms;    // 获取RSSI定时器(ms)
    } sub1g;

    // slave最旧的版本号统计
    struct
    {
        char inv_sub1g_version[VERSION_STRING_MAX_LEN + 1]; // 最旧的微逆Sub1G版本(类型4)
        char inv_800w_version[VERSION_STRING_MAX_LEN + 1];  // 最旧的800W微逆MCU版本(类型5)
        char inv_2500w_version[VERSION_STRING_MAX_LEN + 1]; // 最旧的2500W微逆MCU版本(类型6)
        bool slave_version_reported;                        // slave_version已上报标志
    } slave_version;

    // 微逆信息储存
    inv_device_t paired_inv_info[INV_DEVICE_MAX_NUM];                 // 已配对的微逆设备信息数组(最多8个)
    unpaired_device_t inv_request_pair_list[UNPAIRED_DEVICE_MAX_NUM]; // INV请求配对列表
    user_pair_request_t user_pair_list[USER_PAIR_LIST_MAX_NUM];       // 用户配对列表

    bool is_three_phase; // 是否单相发电

    float ct_today_power_time;    // ct统计今日发电量时间累计
    float ct_today_energy;        // ct统计今日发电量
    uint8_t limit_state;          // 限流状态: 0正常发电 1限流中 2限流失败
    uint8_t power_work_mode;      // 工作模式: 1防逆流发电，2限功率发电，3自由发电模式
    uint16_t to_grid_power_limit; // 功率限制值，允许向电网馈入的最大功率值
    uint8_t anti_backflow_switch; // 防逆流开关状态

    // 功率广播计数器(由SysTick_Handler每1ms递增)
    uint32_t date_broadcast_counter; // 日期广播计数器(ms)

    bool restore_sys;         // 是否需要恢复系统
    bool flash_sn_com_normal; // SN读到正常标志

    // 0x51数据上报缓存
    inv_0x51_report_t inv_0x51_report;
} sys_param_t;

// ================= 全局变量声明 =================
extern sys_param_t sys_param;

// 电压和电流缓冲区声明
extern float ua_voltage_buffer[TOTAL_SAMPLES];
extern float last_ua_voltage_buffer[TOTAL_SAMPLES];
extern float current1_buffer[TOTAL_SAMPLES];
extern float current2_buffer[TOTAL_SAMPLES];
extern float current3_buffer[TOTAL_SAMPLES];

// ================= 函数声明 =================

// 系统初始化函数
void system_param_init(void);
void grid_manager_init(void);

// 初始化函数
void ct_online_detect_init(ct_param_t *ct_param);
void power_calc_init(power_calc_t *calc_power);
void ct_power_direction_detect_init(ct_param_t *ct);

// 中断相关函数
void adc_sample_and_process(void);
void voltage_and_current_buffer_record(void);
void system_timer_management(void);
uint16_t get_voltage_buffer_index(void);
uint16_t get_calc_buf_snap(void);

// 主循环任务函数
void ct_task(void);
void inv_phase_detect_fix_direction_task(void);
void system_state_machine(grid_manager_t *grid_mgr, ct_param_t *ct1, ct_param_t *ct2, ct_param_t *ct3);

// 计算函数
void ct_rms_calculate(void);

// 检测和处理函数
void ct_online_detect_process(ct_param_t *ct_param, float rms_value);
void ct_power_direction_detect_process(ct_param_t *ct);

// 系统标志位管理函数
void system_flags_init(void);
void set_task_flags_from_interrupt(void);

void delay_us(uint16_t us);
void delay_ms(uint16_t ms);

#endif /* __MAIN_H__ */

/*eof*/
