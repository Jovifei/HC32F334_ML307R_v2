#ifndef ML307R_H
#define ML307R_H

#include <stdbool.h>

/*============================================================================
 ML307R 4G模块类型定义
============================================================================*/

/**
 * ML307R模块状态枚举
 * - ML307R_STATE_INIT: 初始化状态
 * - ML307R_STATE_SIM_CHECK: SIM卡检查状态
 * - ML307R_STATE_REGISTERED: 网络已注册
 * - ML307R_STATE_DIAL: PDP拨号状态
 * - ML307R_STATE_CONNECTED: 连接成功
 * - ML307R_STATE_ERROR: 错误状态
 */
typedef enum {
    ML307R_STATE_INIT = 0,
    ML307R_STATE_SIM_CHECK,
    ML307R_STATE_REGISTERED,
    ML307R_STATE_DIAL,
    ML307R_STATE_CONNECTED,
    ML307R_STATE_ERROR
} ml307r_state_t;

/**
 * MQTT客户端状态枚举
 * - MQTT_STATE_DISCONNECTED: 断开连接
 * - MQTT_STATE_CONNECTING: 连接中
 * - MQTT_STATE_CONNECTED: 已连接
 * - MQTT_STATE_ERROR: 错误状态
 */
typedef enum {
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

/**
 * 信号质量结构体
 * - rssi: 信号强度指示(0-31, 99=未知)
 * - ber: 误码率(0-7, 99=未知)
 */
typedef struct {
    int rssi;   // 0-31, 99=unknown
    int ber;    // 0-7, 99=unknown
} signal_quality_t;

/*============================================================================
 ML307R 4G模块基础接口
============================================================================*/

/**
 * ML307R模块初始化
 * 执行AT通信测试、关闭回显、SIM卡检查、网络注册、PDP拨号
 * @return 0=成功, -1=失败
 */
int ml307r_init(void);

/**
 * 获取ML307R模块当前状态
 * @return ml307r_state_t 当前状态
 */
ml307r_state_t ml307r_get_state(void);

/**
 * 获取ML307R模块信号质量
 * 通过AT+CSQ命令查询RSSI和BER
 * @param sq 信号质量结构体指针，用于输出
 * @return 0=成功, -1=失败
 */
int ml307r_get_signal_quality(signal_quality_t *sq);

/**
 * 判断ML307R模块是否欠费
 * 条件：RSSI在10-31范围内(信号正常)，但状态不是已注册/拨号/连接中
 * @return true=欠费, false=正常
 */
bool ml307r_is_arrears(void);

/**
 * ML307R模块重新连接
 * 关闭射频、开启射频、等待网络注册、重新激活PDP
 * @return 0=成功, -1=失败
 */
int ml307r_reconnect(void);

/*============================================================================
 MQTT客户端接口
============================================================================*/

/**
 * MQTT客户端连接
 * 获取设备凭据、构造Topic、配置MQTT、建立连接、订阅下行Topic
 * @return 0=成功, -1=失败
 */
int ml307r_mqtt_connect(void);

/**
 * MQTT客户端断开连接
 * @return 0=成功, -1=失败
 */
int ml307r_mqtt_disconnect(void);

/**
 * MQTT发布消息
 * @param topic 主题
 * @param payload 消息载荷
 * @param qos 服务质量等级(0/1)
 * @return 0=成功, -1=失败
 */
int ml307r_mqtt_publish(const char *topic, const char *payload, int qos);

/**
 * 获取MQTT客户端当前状态
 * @return mqtt_state_t 当前状态
 */
mqtt_state_t ml307r_mqtt_get_state(void);

/*============================================================================
 主任务接口
============================================================================*/

/**
 * ML307R主任务函数
 * 在主循环中调用，包含设备注册、网络初始化、MQTT连接、主循环上报
 */
void ml307r_task(void);

#endif // ML307R_H
