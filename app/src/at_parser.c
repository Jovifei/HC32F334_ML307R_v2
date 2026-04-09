#include "at_parser.h"
#include "config.h"
#include "uart_at.h"
#include <stdio.h>
#include <string.h>

#define MQTT_CONN_ID 0

// MQTT 回调表
typedef struct {
  mqtt_msg_callback_t cb;
  bool used;
} mqtt_cb_entry_t;
static mqtt_cb_entry_t s_mqtt_cbs[MQTT_CB_MAX];
static bool s_urc_registered = false;

/*---------------------------------------------------------------------------
 Name        : mqtt_urc_handler
 Input       : line - URC完整文本行(以\0结尾的字符串)
 Output      : 无
 Description :
 MQTT URC(非请求上报)解析入口函数。
 该函数由 `at_register_urc("+MQTTURC:", ...)`
注册后触发，负责识别并处理以下URC：
 - 连接成功提示：`+MQTTURC: "conn",0,0`
 - 订阅确认提示：`+MQTTURC: "suback"...`
 - 下行消息：`+MQTTURC: "message","<topic>",<qos>,<len>,"<data>"`

 当识别到下行消息时，会解析 topic/qos/len/payload，并遍历 `s_mqtt_cbs`
中已注册的回调， 逐个调用 `mqtt_msg_callback_t(topic, payload, payload_len)`
进行上层分发。 注意：
 - payload 为双引号包裹的字符串形式，本实现按 `%[^\"]`
解析，不支持包含未转义引号的复杂数据。
 - payload_len 使用URC中的 len 字段，并做最大长度裁剪以避免缓冲区溢出。
---------------------------------------------------------------------------*/
static void mqtt_urc_handler(const char *line) {
  if (line == NULL)
    return;

  // +MQTTURC: "conn",0,0 -> 连接成功
  if (strstr(line, "+MQTTURC: \"conn\",0,0") != NULL) {
    return;
  }

  // +MQTTURC: "suback" -> 订阅确认
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
 Input       : host - MQTT服务器域名/IP
               port - MQTT服务器端口
               client_id - 客户端ID(通常为设备唯一标识)
               username - 用户名(按平台要求填写)
               password - 密码/密钥(按平台要求填写)
 Output      : 0=成功, -1=超时, -2=错误(透传自 at_send_command)
 Description :
 配置MQTT连接参数(但不一定立即建立连接)。
 通过拼接并发送模块AT命令 `AT+MQTTCONN=...` 写入连接配置，供后续
`at_mqtt_connect()` 使用。 依赖模块：`uart_at.c` 提供的 `at_send_command()`
完成串口发送与响应等待。
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
 Input       : 无
 Output      : 0=成功, -1=超时, -2=错误(透传自 at_send_command)
 Description :
 建立MQTT连接。
 发送 `AT+MQTTCONN=<conn_id>`
触发模块进行实际连接动作；连接成功后模块可能上报URC， 下行消息由
`mqtt_urc_handler()` 解析并通过已注册回调上送。
---------------------------------------------------------------------------*/
int at_mqtt_connect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_disconnect
 Input       : 无
 Output      : 0=成功, -1=超时, -2=错误(透传自 at_send_command)
 Description :
 断开MQTT连接。
 发送 `AT+MQTTDISC=<conn_id>`，请求模块主动断开与服务器的连接。
---------------------------------------------------------------------------*/
int at_mqtt_disconnect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_subscribe
 Input       : topic - 订阅主题
               qos - 订阅QoS(0/1，取决于模块/平台支持)
 Output      : 0=成功, -1=超时, -2=错误(透传自 at_send_command)
 Description :
 订阅指定topic的下行消息。
 发送
`AT+MQTTSUB=<conn_id>,"<topic>",<qos>`，订阅确认通常会通过URC上报(suback)。
---------------------------------------------------------------------------*/
int at_mqtt_subscribe(const char *topic, int qos) {
  char cmd[256], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic,
           qos);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_publish
 Input       : topic - 发布主题
               qos - 发布QoS(0/1)
               data - 发布数据(字符串)
               data_len - 数据长度(字节)
 Output      : 0=成功, -1=超时, -2=错误/参数非法
 Description :
 发布MQTT消息到指定topic。
 当前实现将 payload 作为字符串拼入AT命令 `AT+MQTTPUB=...,"<data>"` 发送。
 注意：
 - 若 `data` 为NULL或 `data_len<=0`，直接返回 -2。
 -
payload中如包含双引号或特殊字符，可能需要上层先做转义/编码，否则模块解析会失败。
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
 Input       : callback - MQTT下行消息回调函数
 Output      : 无
 Description :
 注册MQTT下行消息回调。
 - 将回调写入 `s_mqtt_cbs` 的空闲槽位，支持注册多个回调(最多 MQTT_CB_MAX 个)。
 - 首次注册时，会调用 `at_register_urc("+MQTTURC:", mqtt_urc_handler)`
绑定URC处理函数， 确保模块上报的MQTT事件/消息能够被解析并分发到回调。
---------------------------------------------------------------------------*/
void at_mqtt_register_callback(mqtt_msg_callback_t callback) {
  if (callback == NULL)
    return;

  // 查找空闲槽位
  for (int i = 0; i < MQTT_CB_MAX; i++) {
    if (!s_mqtt_cbs[i].used) {
      s_mqtt_cbs[i].cb = callback;
      s_mqtt_cbs[i].used = true;
      break;
    }
  }

  // URC 回调只需注册一次
  if (!s_urc_registered) {
    at_register_urc("+MQTTURC:", mqtt_urc_handler);
    s_urc_registered = true;
  }
}
