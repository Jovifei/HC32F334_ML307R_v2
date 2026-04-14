#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 AT����������ӿ�
 MQTT���AT�����װ
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
 MQTT AT�����װ
============================================================================*/

/**
 * ����MQTT���Ӳ���
 * @param host MQTT��������ַ
 * @param port MQTT�������˿�
 * @param client_id �ͻ���ID
 * @param username �û���
 * @param password ����
 * @return 0=�ɹ�, -1=ʧ��
 */
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password);

/**
 * ����MQTT����
 * @return 0=�ɹ�, -1=ʧ��
 */
int at_mqtt_connect(void);

/**
 * �Ͽ�MQTT����
 * @return 0=�ɹ�, -1=ʧ��
 */
int at_mqtt_disconnect(void);

/**
 * ����MQTT����
 * @param topic ����
 * @param qos ���������ȼ�(0/1)
 * @return 0=�ɹ�, -1=ʧ��
 */
int at_mqtt_subscribe(const char *topic, int qos);

/**
 * ����MQTT��Ϣ
 * @param topic ����
 * @param qos ���������ȼ�(0/1)
 * @param data ����
 * @param data_len ���ݳ���
 * @return 0=�ɹ�, -1=ʧ��
 */
int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len);

/**
 * ע��MQTT������Ϣ�ص�
 * ���յ������������Ϣʱ�Զ�����
 * @param callback �ص�����
 */
void at_mqtt_register_callback(mqtt_msg_callback_t callback);

#endif // AT_PARSER_H
