#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 AT解析器对外接口
 MQTT相关AT命令封装
============================================================================*/

/**
 * MQTT消息回调函数类型
 * @param topic 消息主题
 * @param payload 消息负载
 * @param payload_len 负载长度
 */
typedef void (*mqtt_msg_callback_t)(const char *topic, const char *payload, int payload_len);

/**
 * MQTT连接URC结果（由 mqtt_urc_handler 在收到 +MQTTURC: "conn" 时设置）
 * -1 = 待定（尚未收到URC）
 *  0 = 连接成功
 * >0 = 失败码：1=超时, 2=客户端断开, 3=服务器拒绝, 4=服务器断开, 5=PING超时, 6=网络错误
 * 使用前须先置为 -1，连接结束后再读取
 */
extern volatile int g_mqtt_conn_result;

/*============================================================================
 MQTT AT命令封装
============================================================================*/

/**
 * 配置MQTT连接参数
 * @param host MQTT服务器地址
 * @param port MQTT服务器端口
 * @param client_id 客户端ID
 * @param username 用户名
 * @param password 密码
 * @return 0=成功, -1=失败
 */
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password);

/**
 * 建立MQTT连接
 * @return 0=成功, -1=失败
 */
int at_mqtt_connect(void);

/**
 * 断开MQTT连接
 * @return 0=成功, -1=失败
 */
int at_mqtt_disconnect(void);

/**
 * 订阅MQTT主题
 * @param topic 主题
 * @param qos 消息服务质量(0/1)
 * @return 0=成功, -1=失败
 */
int at_mqtt_subscribe(const char *topic, int qos);

/**
 * 发布MQTT消息
 * @param topic 主题
 * @param qos 消息服务质量(0/1)
 * @param data 数据
 * @param data_len 数据长度
 * @return 0=成功, -1=失败
 */
int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len);

/**
 * 注册MQTT下行消息回调
 * 收到平台下发的消息时自动调用
 * @param callback 回调函数
 */
void at_mqtt_register_callback(mqtt_msg_callback_t callback);

#endif // AT_PARSER_H
