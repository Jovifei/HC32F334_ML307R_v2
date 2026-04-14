#include "at_parser.h"
#include "config.h"
#include "uart_at.h"
#include <stdio.h>
#include <string.h>

#define MQTT_CONN_ID 0

// MQTT连接URC结果: -1=待定, 0=成功, 其他=失败码(服务器拒绝等)
volatile int g_mqtt_conn_result = -1;

// MQTT 回调表
typedef struct {
  mqtt_msg_callback_t cb;
  bool used;
} mqtt_cb_entry_t;
static mqtt_cb_entry_t s_mqtt_cbs[MQTT_CB_MAX];
static bool s_urc_registered = false;

/*---------------------------------------------------------------------------
 Name        : mqtt_urc_handler
 Input       : line - URC�����ı���(��\0��β���ַ���)
 Output      : ��
 Description :
 MQTT URC(�������ϱ�)������ں�����
 �ú����� `at_register_urc("+MQTTURC:", ...)`
ע��󴥷�������ʶ�𲢴�������URC��
 - ���ӳɹ���ʾ��`+MQTTURC: "conn",0,0`
 - ����ȷ����ʾ��`+MQTTURC: "suback"...`
 - ������Ϣ��`+MQTTURC: "message","<topic>",<qos>,<len>,"<data>"`

 ��ʶ��������Ϣʱ������� topic/qos/len/payload�������� `s_mqtt_cbs`
����ע��Ļص��� ������� `mqtt_msg_callback_t(topic, payload, payload_len)`
�����ϲ�ַ��� ע�⣺
 - payload Ϊ˫���Ű������ַ�����ʽ����ʵ�ְ� `%[^\"]`
��������֧�ְ���δת�����ŵĸ������ݡ�
 - payload_len ʹ��URC�е� len �ֶΣ�������󳤶Ȳü��Ա��⻺���������
---------------------------------------------------------------------------*/
static void mqtt_urc_handler(const char *line) {
  if (line == NULL)
    return;

  // +MQTTURC: "conn",<conn_id>,<result> -> 连接结果
  // result: 0=成功, 1=超时, 2=客户端断开, 3=服务器拒绝, 4=服务器断开, 5=PING超时, 6=网络错误
  if (strstr(line, "+MQTTURC: \"conn\"") != NULL) {
    int conn_id = -1, result_code = -1;
    if (sscanf(line, "+MQTTURC: \"conn\",%d,%d", &conn_id, &result_code) == 2) {
      if (conn_id == MQTT_CONN_ID) {
        g_mqtt_conn_result = result_code;
      }
    }
    return;
  }

  // +MQTTURC: "suback" -> ����ȷ��
  if (strstr(line, "+MQTTURC: \"suback\"") != NULL) {
    return;
  }

  // +MQTTURC: "message","<topic>",<qos>,<len>,"<data>"
  const char *prefix = "+MQTTURC: \"message\"";
  if (strncmp(line, prefix, strlen(prefix)) == 0) {
    char topic[128] = {0};
    char payload[512] = {0};
    int qos = 0, len = 0;
    if (sscanf(line, "+MQTTURC: \"message\",\"%127[^\"]\",%d,%d,\"%511[^\"]\"",
               topic, &qos, &len, payload) >= 4) {
      int actual_len = len < 512 ? len : 511;
      for (int i = 0; i < MQTT_CB_MAX; i++) {
        if (s_mqtt_cbs[i].used && s_mqtt_cbs[i].cb != NULL) {
          s_mqtt_cbs[i].cb(topic, payload, actual_len);
        }
      }
    }
  }
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_config
 Input       : host - MQTT����������/IP
               port - MQTT�������˿�
               client_id - �ͻ���ID(ͨ��Ϊ�豸Ψһ��ʶ)
               username - �û���(��ƽ̨Ҫ����д)
               password - ����/��Կ(��ƽ̨Ҫ����д)
 Output      : 0=�ɹ�, -1=��ʱ, -2=����(͸���� at_send_command)
 Description :
 ����MQTT���Ӳ���(����һ��������������)��
 ͨ��ƴ�Ӳ�����ģ��AT���� `AT+MQTTCONN=...` д���������ã�������
`at_mqtt_connect()` ʹ�á� ����ģ�飺`uart_at.c` �ṩ�� `at_send_command()`
��ɴ��ڷ�������Ӧ�ȴ���
---------------------------------------------------------------------------*/
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password) {
  char cmd[512], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
           MQTT_CONN_ID, host, port, client_id, username, password);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_connect
 Input       : ��
 Output      : 0=�ɹ�, -1=��ʱ, -2=����(͸���� at_send_command)
 Description :
 ����MQTT���ӡ�
 ���� `AT+MQTTCONN=<conn_id>`
����ģ�����ʵ�����Ӷ��������ӳɹ���ģ������ϱ�URC�� ������Ϣ��
`mqtt_urc_handler()` ������ͨ����ע��ص����͡�
---------------------------------------------------------------------------*/
int at_mqtt_connect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_disconnect
 Input       : ��
 Output      : 0=�ɹ�, -1=��ʱ, -2=����(͸���� at_send_command)
 Description :
 �Ͽ�MQTT���ӡ�
 ���� `AT+MQTTDISC=<conn_id>`������ģ�������Ͽ�������������ӡ�
---------------------------------------------------------------------------*/
int at_mqtt_disconnect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_subscribe
 Input       : topic - ��������
               qos - ����QoS(0/1��ȡ����ģ��/ƽ̨֧��)
 Output      : 0=�ɹ�, -1=��ʱ, -2=����(͸���� at_send_command)
 Description :
 ����ָ��topic��������Ϣ��
 ����
`AT+MQTTSUB=<conn_id>,"<topic>",<qos>`������ȷ��ͨ����ͨ��URC�ϱ�(suback)��
---------------------------------------------------------------------------*/
int at_mqtt_subscribe(const char *topic, int qos) {
  char cmd[256], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic,
           qos);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_publish
 Input       : topic - ��������
               qos - ����QoS(0/1)
               data - ��������(�ַ���)
               data_len - ���ݳ���(�ֽ�)
 Output      : 0=�ɹ�, -1=��ʱ, -2=����/�����Ƿ�
 Description :
 ����MQTT��Ϣ��ָ��topic��
 ��ǰʵ�ֽ� payload ��Ϊ�ַ���ƴ��AT���� `AT+MQTTPUB=...,"<data>"` ���͡�
 ע�⣺
 - �� `data` ΪNULL�� `data_len<=0`��ֱ�ӷ��� -2��
 -
payload�������˫���Ż������ַ���������Ҫ�ϲ�����ת��/���룬����ģ�������ʧ�ܡ�
---------------------------------------------------------------------------*/
int at_mqtt_publish(const char *topic, int qos, const char *data,
                    int data_len) {
  if (data == NULL || data_len <= 0)
    return -2;
  char cmd[512], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=%d,\"%s\",%d,0,0,%d,\"%s\"",
           MQTT_CONN_ID, topic, qos, data_len, data);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_register_callback
 Input       : callback - MQTT������Ϣ�ص�����
 Output      : ��
 Description :
 ע��MQTT������Ϣ�ص���
 - ���ص�д�� `s_mqtt_cbs` �Ŀ��в�λ��֧��ע�����ص�(��� MQTT_CB_MAX ��)��
 - �״�ע��ʱ������� `at_register_urc("+MQTTURC:", mqtt_urc_handler)`
��URC���������� ȷ��ģ���ϱ���MQTT�¼�/��Ϣ�ܹ����������ַ����ص���
---------------------------------------------------------------------------*/
void at_mqtt_register_callback(mqtt_msg_callback_t callback) {
  if (callback == NULL)
    return;

  // ���ҿ��в�λ
  for (int i = 0; i < MQTT_CB_MAX; i++) {
    if (!s_mqtt_cbs[i].used) {
      s_mqtt_cbs[i].cb = callback;
      s_mqtt_cbs[i].used = true;
      break;
    }
  }

  // URC �ص�ֻ��ע��һ��
  if (!s_urc_registered) {
    at_register_urc("+MQTTURC:", mqtt_urc_handler);
    s_urc_registered = true;
  }
}
