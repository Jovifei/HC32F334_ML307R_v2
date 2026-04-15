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
 Name        : static void mqtt_urc_handler(const char *line)
 Input       : line - URC原始文本行（以\0结尾的字符串）
 Output      : 无
 Description : MQTT URC（主动上报）内部处理函数。
               由 at_register_urc("+MQTTURC:", ...)
               注册后触发，用于识别并处理MQTT URC：
               - 连接成功提示：+MQTTURC: "conn",0,0
               - 订阅确认提示：+MQTTURC: "suback"...
               - 下行消息：+MQTTURC: "message","<topic>",<qos>,<len>,"<data>"

               当识别到下行消息时，提取 topic/qos/len/payload，遍历 s_mqtt_cbs
               中已注册的回调函数，调用 mqtt_msg_callback_t(topic, payload, payload_len)。
               注意：
               - payload 为双引号包裹的字符串格式，实现使用 %[^\"] 格式化解析，
                 不支持包含未转义引号的复杂数据。
               - payload_len 使用URC中的 len 字段，并截断至最大长度以防止缓冲区溢出。
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
 Name        : int at_mqtt_config(const char *host, int port, const char *client_id,
               const char *username, const char *password)
 Input       : host - MQTT服务器地址/IP
               port - MQTT服务器端口
               client_id - 客户端ID（通常为设备唯一标识）
               username - 用户名（视平台要求填写）
               password - 密码/密钥（视平台要求填写）
 Output      : 0=成功, -1=超时, -2=错误（透传自 at_send_command）
 Description : 配置MQTT连接参数（只发一条带全参数的命令）。
               通过拼接参数生成模组AT命令 AT+MQTTCONN=... 写入连接配置，供
               at_mqtt_connect() 使用。依赖模组：uart_at.c 提供的 at_send_command()
               完成串口发送和响应等待。
---------------------------------------------------------------------------*/
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password) {
  char cmd[512], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
           MQTT_CONN_ID, host, port, client_id, username, password);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : int at_mqtt_connect(void)
 Input       : 无
 Output      : 0=成功, -1=超时, -2=错误（透传自 at_send_command）
 Description : 建立MQTT连接。
               发送 AT+MQTTCONN=<conn_id>，模组完成实际连接；连接成功后模组会上报URC。
               连接消息由 mqtt_urc_handler() 处理，并通过已注册回调分发。
---------------------------------------------------------------------------*/
int at_mqtt_connect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : int at_mqtt_disconnect(void)
 Input       : 无
 Output      : 0=成功, -1=超时, -2=错误（透传自 at_send_command）
 Description : 断开MQTT连接。
               发送 AT+MQTTDISC=<conn_id>，命令模组主动断开与服务器的连接。
---------------------------------------------------------------------------*/
int at_mqtt_disconnect(void) {
  char cmd[64], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : int at_mqtt_subscribe(const char *topic, int qos)
 Input       : topic - 订阅主题
               qos - 订阅QoS(0/1，取决于模组/平台支持)
 Output      : 0=成功, -1=超时, -2=错误（透传自 at_send_command）
 Description : 订阅指定topic的下行消息。
               发送 AT+MQTTSUB=<conn_id>,"<topic>",<qos>，订阅确认通过异步URC上报(suback)。
---------------------------------------------------------------------------*/
int at_mqtt_subscribe(const char *topic, int qos) {
  char cmd[256], resp[256];
  snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic,
           qos);
  return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len)
 Input       : topic - 消息主题
               qos - 消息QoS(0/1)
               data - 消息数据（字符串）
               data_len - 数据长度（字节）
 Output      : 0=成功, -1=超时, -2=错误/参数非法
 Description : 发布MQTT消息到指定topic。
               当前实现将 payload 作为字符串拼入AT命令 AT+MQTTPUB=...,"<data>" 发送。
               注意：
               - 若 data 为NULL 或 data_len<=0，直接返回 -2。
               - payload中若含双引号等特殊字符，需要上层提前转义/编码，否则模组可能解析失败。
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
 Name        : void at_mqtt_register_callback(mqtt_msg_callback_t callback)
 Input       : callback - MQTT下行消息回调函数
 Output      : 无
 Description : 注册MQTT下行消息回调。
               - 将回调写入 s_mqtt_cbs 的空闲插槽（支持注册多个回调，最多 MQTT_CB_MAX 个）。
               - 首次注册时，自动调用 at_register_urc("+MQTTURC:", mqtt_urc_handler)
                 使能URC监听，确保模组上报的MQTT事件/消息能够被正确分发到回调函数。
---------------------------------------------------------------------------*/
void at_mqtt_register_callback(mqtt_msg_callback_t callback) {
  if (callback == NULL)
    return;

  // 查找空闲插槽
  for (int i = 0; i < MQTT_CB_MAX; i++) {
    if (!s_mqtt_cbs[i].used) {
      s_mqtt_cbs[i].cb = callback;
      s_mqtt_cbs[i].used = true;
      break;
    }
  }

  // URC 回调只注册一次
  if (!s_urc_registered) {
    at_register_urc("+MQTTURC:", mqtt_urc_handler);
    s_urc_registered = true;
  }
}
