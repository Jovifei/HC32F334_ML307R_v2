#include "at_parser.h"
#include "config.h"
#include "uart_at.h"
#include <stdio.h>
#include <string.h>

#define MQTT_CONN_ID 0

// MQTT连接URC结果: -1=待定, 0=成功, 其他=失败码(服务器拒绝等)
volatile int g_mqtt_conn_result = -1;

// MQTT断开URC结果: -1=未断开, >=0=断开原因码
volatile int g_mqtt_disc_code = -1;

// MQTT 回调表
typedef struct {
  mqtt_msg_callback_t cb;
  bool used;
} mqtt_cb_entry_t;
static mqtt_cb_entry_t s_mqtt_cbs[MQTT_CB_MAX];
static bool s_urc_registered = false;

/*---------------------------------------------------------------------------
 Name        : static void mqtt_urc_handler(const char *urc_str)
 Input       : urc_str - URC字符串
 Output      : 无
 Description : MQTT URC（未请求结果码）处理器。
               处理模块主动上报的MQTT相关事件，如连接、断开、消息等。
               该函数在中断上下文或后台任务中被调用，需要快速处理。
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

  // +MQTTURC: "disc",<conn_id>,<code> -> 连接断开通知
  if (strstr(line, "+MQTTURC: \"disc\"") != NULL) {
    int conn_id = -1, disc_code = 0;
    if (sscanf(line, "+MQTTURC: \"disc\",%d,%d", &conn_id, &disc_code) >= 1) {
      if (conn_id == MQTT_CONN_ID) {
        g_mqtt_disc_code = disc_code;
      }
    } else {
      g_mqtt_disc_code = 0;
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
 Name        : void at_mqtt_config(const char *host, uint16_t port, const char *client_id)
 Input       : host - MQTT服务器地址
               port - MQTT服务器端口
               client_id - 客户端ID
 Output      : 无
 Description : MQTT配置。
               配置MQTT连接参数，包括服务器地址、端口、客户端标识等。
---------------------------------------------------------------------------*/
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password) {
  char cmd[512], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
           MQTT_CONN_ID, host, port, client_id, username, password);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : void at_mqtt_connect(void)
 Input       : 无
 Output      : 无
 Description : MQTT连接。发起MQTT连接请求到已配置的服务器。
---------------------------------------------------------------------------*/
int at_mqtt_connect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : void at_mqtt_disconnect(void)
 Input       : 无
 Output      : 无
 Description : MQTT断开连接。
               断开与MQTT服务器的连接。
---------------------------------------------------------------------------*/
int at_mqtt_disconnect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : void at_mqtt_subscribe(const char *topic)
 Input       : topic - MQTT主题
 Output      : 无
 Description : MQTT订阅。订阅指定的MQTT主题以接收消息。
---------------------------------------------------------------------------*/
int at_mqtt_subscribe(const char *topic, int qos) {
  char cmd[256], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic,
           qos);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : void at_mqtt_publish(const char *topic, const char *payload)
 Input       : topic - MQTT主题
               payload - 消息载荷
 Output      : 无
 Description : MQTT发布。
               向指定主题发布MQTT消息。
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
 Name        : void at_mqtt_register_callback(mqtt_callback_t callback)
 Input       : callback - 回调函数指针
 Output      : 无
 Description : 注册MQTT回调函数。
               注册处理MQTT事件的回调函数。
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
