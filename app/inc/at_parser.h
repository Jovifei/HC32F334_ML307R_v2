#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 AT命令解析器接口
 MQTT相关AT命令封装
============================================================================*/

/**
 * MQTT消息回调函数类型
 * @param topic 消息主题
 * @param payload 消息载荷
 * @param payload_len 载荷长度
 */
typedef void (*mqtt_msg_callback_t)(const char *topic, const char *payload, int payload_len);

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
 * @param qos 服务质量等级(0/1)
 * @return 0=成功, -1=失败
 */
int at_mqtt_subscribe(const char *topic, int qos);

/**
 * 发布MQTT消息
 * @param topic 主题
 * @param qos 服务质量等级(0/1)
 * @param data 数据
 * @param data_len 数据长度
 * @return 0=成功, -1=失败
 */
int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len);

/**
 * 注册MQTT下行消息回调
 * 当收到订阅主题的消息时自动调用
 * @param callback 回调函数
 */
void at_mqtt_register_callback(mqtt_msg_callback_t callback);

#endif // AT_PARSER_H
