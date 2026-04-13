#include "ml307r.h"
#include "at_parser.h"
#include "config.h"
#include "device_register.h"
#include "main.h"
#include "uart_at.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==================== ML307R 状态机 ====================

static ml307r_state_t s_ml_state = ML307R_STATE_INIT;

/*---------------------------------------------------------------------------
 Name        : ml307r_init
 Input       : 无
 Output      : 0=成功, -1=失败
 Description :
 ML307R 4G模块初始化函数。执行以下步骤：
 1. AT通信测试
 2. 关闭回显(ATE0)
 3. 检查SIM卡状态(AT+CPIN?)
 4. 等待网络注册(AT+CEREG?)，最多30秒
 5. 激活PDP拨号(AT+MIPCALL=1,1)
---------------------------------------------------------------------------*/
int ml307r_init(void)
{
    int ret;
    char resp[128];

    s_ml_state = ML307R_STATE_INIT;
    DEBUG_4G_PRINTF(" >>> ml307r_init start\r\n");

    // ML307R 启动需要时间，先等待一下
    DEBUG_4G_PRINTF(" >>> Waiting for ML307R to be ready...\r\n");
    delay_ms(2000);  // 等待2秒让ML307R完全启动

    // 1. AT 通信测试
    DEBUG_4G_PRINTF(" >>> AT send: AT\r\n");
    ret = at_send_command("AT", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT resp: %s, ret=%d\r\n", resp, ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF(" !!! AT test failed !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }
    DEBUG_4G_PRINTF(" OK - AT test passed\r\n");

    // 2. 关闭回显 ATE0
    DEBUG_4G_PRINTF(" >>> AT send: ATE0\r\n");
    ret = at_send_command("ATE0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< ATE0 resp: %s, ret=%d\r\n", resp, ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF(" !!! ATE0 failed !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }
    DEBUG_4G_PRINTF(" OK - Echo disabled\r\n");

    // 3. 检查 SIM 卡 AT+CPIN?
    s_ml_state = ML307R_STATE_SIM_CHECK;
    DEBUG_4G_PRINTF(" >>> AT send: AT+CPIN?\r\n");
    ret = at_send_command("AT+CPIN?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CPIN? resp: %s, ret=%d\r\n", resp, ret);
    if (ret != 0 || strstr(resp, "READY") == NULL)
    {
      DEBUG_4G_PRINTF(" !!! SIM card not ready !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }
    DEBUG_4G_PRINTF(" OK - SIM card ready\r\n");

    // 4. 查询信号质量 AT+CSQ（建议信号 > 18）
    DEBUG_4G_PRINTF(" >>> AT send: AT+CSQ\r\n");
    ret = at_send_command("AT+CSQ", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CSQ resp: %s, ret=%d\r\n", resp, ret);
    if (ret == 0)
    {
      int rssi = 99;
      (void)sscanf(resp, "+CSQ: %d", &rssi);
      if (rssi < 10 || rssi > 31)
      {
        DEBUG_4G_PRINTF(" !!! Signal quality poor (RSSI=%d) !!!\r\n", rssi);
      }
      else
      {
        DEBUG_4G_PRINTF(" OK - Signal quality OK (RSSI=%d)\r\n", rssi);
      }
    }

    // 5. 查询网络附着状态 AT+CGATT?
    DEBUG_4G_PRINTF(" >>> AT send: AT+CGATT?\r\n");
    ret = at_send_command("AT+CGATT?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CGATT? resp: %s, ret=%d\r\n", resp, ret);
    if (ret == 0 && strstr(resp, "+CGATT: 1") != NULL)
    {
      DEBUG_4G_PRINTF(" OK - Network attached\r\n");
    }

    // 7. 等待网络注册 AT+CEREG?（最多 30 次，每次 1s）
    s_ml_state = ML307R_STATE_REGISTERED;
    DEBUG_4G_PRINTF(" >>> Waiting for network registration (AT+CEREG?)\r\n");
    int retry = 0;
    while (retry < 30)
    {
      DEBUG_4G_PRINTF(" >>> AT+CEREG? attempt %d/30\r\n", retry + 1);
      ret = at_send_command("AT+CEREG?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
      DEBUG_4G_PRINTF(" <<< AT+CEREG? resp: %s, ret=%d\r\n", resp, ret);
      if (ret == 0 && (strstr(resp, "+CEREG: 0,1") != NULL ||
                       strstr(resp, "+CEREG: 1,1") != NULL))
      {
        DEBUG_4G_PRINTF(" OK - Network registered after %d attempts\r\n", retry + 1);
        break;
      }
      retry++;
      delay_ms(1000);
    }
    if (retry >= 30)
    {
      DEBUG_4G_PRINTF(" !!! Network registration timeout !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }

    // 8. 激活 PDP 拨号 AT+MIPCALL=1,1
    s_ml_state = ML307R_STATE_DIAL;
    DEBUG_4G_PRINTF(" >>> AT send: AT+MIPCALL=1,1\r\n");
    ret = at_send_command("AT+MIPCALL=1,1", "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+MIPCALL=1,1 resp: %s, ret=%d\r\n", resp, ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF(" !!! PDP activation failed !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }

    s_ml_state = ML307R_STATE_CONNECTED;
    DEBUG_4G_PRINTF(" OK - ML307R init success, state=CONNECTED\r\n");
    return 0;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_get_state
 Input       : 无
 Output      : ml307r_state_t - 当前ML307R模块状态
 Description :
 获取ML307R 4G模块的当前状态，包括：
 - ML307R_STATE_INIT: 初始化状态
 - ML307R_STATE_SIM_CHECK: SIM卡检查状态
 - ML307R_STATE_REGISTERED: 网络已注册
 - ML307R_STATE_DIAL: PDP拨号状态
 - ML307R_STATE_CONNECTED: 连接成功
 - ML307R_STATE_ERROR: 错误状态
---------------------------------------------------------------------------*/
ml307r_state_t ml307r_get_state(void)
{
    DEBUG_4G_PRINTF(" >>> ml307r_get_state: %d\r\n", s_ml_state);
    return s_ml_state;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_get_signal_quality
 Input       : sq - signal_quality_t结构体指针，用于输出信号质量
 Output      : 0=成功, -1=失败
 Description :
 获取ML307R 4G模块的信号质量，通过AT+CSQ命令查询。
 返回RSSI(信号强度)和BER(误码率)两个参数。
---------------------------------------------------------------------------*/
int ml307r_get_signal_quality(signal_quality_t *sq)
{
    if (sq == NULL)
      return -1;
    char resp[128];
    int rssi = 99, ber = 99;
    DEBUG_4G_PRINTF(" >>> AT send: AT+CSQ\r\n");
    int ret = at_send_command("AT+CSQ", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CSQ resp: %s, ret=%d\r\n", resp, ret);
    if (ret == 0)
    {
      (void)sscanf(resp, "+CSQ: %d,%d", &rssi, &ber);
    }
    sq->rssi = rssi;
    sq->ber = ber;
    DEBUG_4G_PRINTF(" Signal quality: RSSI=%d, BER=%d\r\n", rssi, ber);
    return (ret == 0) ? 0 : -1;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_is_arrears
 Input       : 无
 Output      : true=欠费(信号正常但未注册), false=正常
 Description :
 判断ML307R模块是否处于欠费状态。
 条件：RSSI在10-31范围内(信号正常)，但状态不是已注册/拨号/连接中。
---------------------------------------------------------------------------*/
bool ml307r_is_arrears(void)
{
    signal_quality_t sq;
    ml307r_get_signal_quality(&sq);
    ml307r_state_t st = ml307r_get_state();
    bool arrears =
        (sq.rssi >= 10 && sq.rssi <= 31 && st != ML307R_STATE_REGISTERED &&
         st != ML307R_STATE_DIAL && st != ML307R_STATE_CONNECTED);
    DEBUG_4G_PRINTF(" ml307r_is_arrears: %s\r\n", arrears ? "YES" : "NO");
    return arrears;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_reconnect
 Input       : 无
 Output      : 0=成功, -1=失败
 Description :
 ML307R 4G模块重新连接函数。
 执行以下步骤：
 1. 关闭射频功能(AT+CFUN=0)
 2. 开启射频功能(AT+CFUN=1)
 3. 等待网络重新注册(AT+CEREG?)，最多30秒
 4. 重新激活PDP拨号(AT+MIPCALL=1,1)
---------------------------------------------------------------------------*/
int ml307r_reconnect(void)
{
    char resp[128];
    DEBUG_4G_PRINTF(" >>> ml307r_reconnect start\r\n");

    DEBUG_4G_PRINTF(" >>> AT send: AT+CFUN=0\r\n");
    at_send_command("AT+CFUN=0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CFUN=0 resp: %s\r\n", resp);
    delay_ms(500);

    DEBUG_4G_PRINTF(" >>> AT send: AT+CFUN=1\r\n");
    at_send_command("AT+CFUN=1", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CFUN=1 resp: %s\r\n", resp);
    delay_ms(2000);

    int retry = 0;
    while (retry < 30)
    {
      DEBUG_4G_PRINTF(" >>> AT+CEREG? reconnect attempt %d/30\r\n", retry + 1);
      int ret = at_send_command("AT+CEREG?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
      DEBUG_4G_PRINTF(" <<< AT+CEREG? resp: %s, ret=%d\r\n", resp, ret);
      if (ret == 0 && (strstr(resp, "+CEREG: 0,1") != NULL ||
                       strstr(resp, "+CEREG: 1,1") != NULL))
      {
        DEBUG_4G_PRINTF(" OK - Network re-registered after %d attempts\r\n", retry + 1);
        break;
      }
      retry++;
      delay_ms(1000);
    }
    if (retry >= 30)
    {
      DEBUG_4G_PRINTF(" !!! Network re-registration timeout !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }

    s_ml_state = ML307R_STATE_REGISTERED;
    DEBUG_4G_PRINTF(" >>> AT send: AT+MIPCALL=1,1\r\n");
    int ret = at_send_command("AT+MIPCALL=1,1", "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+MIPCALL=1,1 resp: %s, ret=%d\r\n", resp, ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF(" !!! PDP re-activation failed !!!\r\n");
      s_ml_state = ML307R_STATE_ERROR;
      return -1;
    }

    s_ml_state = ML307R_STATE_CONNECTED;
    DEBUG_4G_PRINTF(" OK - ml307r_reconnect success\r\n");
    return 0;
}

// ==================== MQTT 客户端状态机 ====================

static mqtt_state_t s_mqtt_state = MQTT_STATE_DISCONNECTED;
static char s_topic_up[64] = {0};
static char s_topic_down[64] = {0};

/*---------------------------------------------------------------------------
 Name        : ml307r_mqtt_connect
 Input       : 无
 Output      : 0=成功, -1=失败
 Description :
 MQTT客户端连接函数
---------------------------------------------------------------------------*/
int ml307r_mqtt_connect(void)
{
    int ret;
    s_mqtt_state = MQTT_STATE_CONNECTING;
    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_connect start\r\n");

    // 获取设备凭据
    const device_credentials_t *cred = device_register_get_credentials();
    if (!cred->registered)
    {
      DEBUG_4G_PRINTF("[MQTT] !!! Device not registered !!!\r\n");
      s_mqtt_state = MQTT_STATE_ERROR;
      return -1;
    }

    // 构造动态 Topic
    snprintf(s_topic_up, sizeof(s_topic_up), "up/%s/%s", cred->product_id, cred->device_id);
    snprintf(s_topic_down, sizeof(s_topic_down), "down/%s/%s", cred->product_id, cred->device_id);
    DEBUG_4G_PRINTF("[MQTT] Topic up: %s\r\n", s_topic_up);
    DEBUG_4G_PRINTF("[MQTT] Topic down: %s\r\n", s_topic_down);

    // 使用动态凭据配置 MQTT
    DEBUG_4G_PRINTF("[MQTT] >>> at_mqtt_config\r\n");
    ret = at_mqtt_config(MQTT_SERVER, MQTT_PORT, cred->device_sn, cred->device_id, cred->device_key);
    DEBUG_4G_PRINTF("[MQTT] <<< at_mqtt_config ret=%d\r\n", ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF("[MQTT] !!! MQTT config failed !!!\r\n");
      s_mqtt_state = MQTT_STATE_ERROR;
      return -1;
    }
    DEBUG_4G_PRINTF("[MQTT] OK - MQTT config done\r\n");

    DEBUG_4G_PRINTF("[MQTT] >>> at_mqtt_connect\r\n");
    ret = at_mqtt_connect();
    DEBUG_4G_PRINTF("[MQTT] <<< at_mqtt_connect ret=%d\r\n", ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF("[MQTT] !!! MQTT connect failed !!!\r\n");
      s_mqtt_state = MQTT_STATE_ERROR;
      return -1;
    }
    DEBUG_4G_PRINTF("[MQTT] OK - MQTT connected\r\n");

    DEBUG_4G_PRINTF("[MQTT] >>> at_mqtt_subscribe: %s\r\n", s_topic_down);
    ret = at_mqtt_subscribe(s_topic_down, 1);
    DEBUG_4G_PRINTF("[MQTT] <<< at_mqtt_subscribe ret=%d\r\n", ret);
    if (ret != 0)
    {
      DEBUG_4G_PRINTF("[MQTT] !!! MQTT subscribe failed !!!\r\n");
      s_mqtt_state = MQTT_STATE_ERROR;
      return -1;
    }
    DEBUG_4G_PRINTF("[MQTT] OK - MQTT subscribed to %s\r\n", s_topic_down);

    s_mqtt_state = MQTT_STATE_CONNECTED;
    DEBUG_4G_PRINTF("[MQTT] OK - ml307r_mqtt_connect success, state=CONNECTED\r\n");
    return 0;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_mqtt_disconnect
 Input       : 无
 Output      : 0=成功, -1=失败
 Description :
 MQTT客户端断开连接函数
---------------------------------------------------------------------------*/
int ml307r_mqtt_disconnect(void)
{
    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_disconnect\r\n");
    int ret = at_mqtt_disconnect();
    DEBUG_4G_PRINTF("[MQTT] <<< at_mqtt_disconnect ret=%d\r\n", ret);
    s_mqtt_state = MQTT_STATE_DISCONNECTED;
    return (ret == 0) ? 0 : -1;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_mqtt_publish
 Input       : topic - 主题
                 payload - 消息载荷
                 qos - 服务质量等级(0/1)
 Output      : at_mqtt_publish的返回值
 Description :
 MQTT发布消息函数，将数据发布到指定主题
---------------------------------------------------------------------------*/
int ml307r_mqtt_publish(const char *topic, const char *payload, int qos)
{
    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_publish: topic=%s, qos=%d, payload=%s\r\n", topic, qos, payload);
    int ret = at_mqtt_publish(topic, qos, payload, (int)strlen(payload));
    DEBUG_4G_PRINTF("[MQTT] <<< ml307r_mqtt_publish ret=%d\r\n", ret);
    return ret;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_mqtt_get_state
 Input       : 无
 Output      : mqtt_state_t - 当前MQTT连接状态
 Description :
 获取MQTT客户端的当前状态
---------------------------------------------------------------------------*/
mqtt_state_t ml307r_mqtt_get_state(void)
{
    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_get_state: %d\r\n", s_mqtt_state);
    return s_mqtt_state;
}

// ==================== 云端消息解析 ====================

static void on_mqtt_message(const char *topic, const char *payload, int len);

// MQTT ID 计数器
static int s_mqtt_msg_id = 0;

/*---------------------------------------------------------------------------
 Name        : publish_device_info
 Input       : 无
 Output      : 无
 Description :
 上报设备信息到云端。
 包含设备SN、产品型号、MCU版本和RSSI信号强度。
---------------------------------------------------------------------------*/
static void publish_device_info(void)
{
    const device_credentials_t *cred = device_register_get_credentials();
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"method\":\"information\",\"params\":{"
             "\"sn\":\"%s\",\"product_model\":\"%s\","
             "\"mcu_version\":\"%s\",\"rssi\":-63}}",
             s_mqtt_msg_id++, cred->device_sn, cred->product_model, SW_VERSION);
    DEBUG_4G_PRINTF("[MQTT] >>> publish_device_info: %s\r\n", payload);
    ml307r_mqtt_publish(s_topic_up, payload, 0);
}

/*---------------------------------------------------------------------------
 Name        : publish_ct_power
 Input       : 无
 Output      : 无
 Description :
 上报CT功率数据到云端。
 包含第一路CT有功功率和当日累计发电量。
---------------------------------------------------------------------------*/
static void publish_ct_power(void)
{
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"method\":\"properties_changed\",\"params\":["
             "{\"siid\":4,\"piid\":7,\"value\":%.1f},"
             "{\"siid\":4,\"piid\":8,\"value\":%.1f}]}",
             s_mqtt_msg_id++, sys_param.ct1.power.avg_power,
             sys_param.ct_today_energy);
    DEBUG_4G_PRINTF("[MQTT] >>> publish_ct_power: %s\r\n", payload);
    ml307r_mqtt_publish(s_topic_up, payload, 0);
}

// ==================== JSON 属性解析辅助 ====================

typedef struct
{
    int siid;
    int piid;
    char value_str[32];
    int has_value;
} prop_item_t;

/*---------------------------------------------------------------------------
 Name        : parse_params
 Input       : json  - JSON字符串(包含 "params":[{...},...] 的报文)
                 items - 输出数组，用于保存解析到的(siids/piid/value)项
                 max_n - items数组最大容量
 Output      : 实际解析到的项数(0表示未解析到或格式不匹配)
 Description :
 从云端下行JSON报文中解析 "params" 数组字段，提取每一项的：
 - siid / piid
 - value(可选)，统一保存为字符串 `value_str`，并置位 `has_value`
 用途：
 - `handle_get_properties()`：解析 get_properties 请求参数(siids/piid)
 - `handle_set_properties()`：解析 set_properties 请求参数(siids/piid/value)
 说明与限制：
 - 本实现为轻量级字符串扫描/截取，不是通用JSON解析器
 - 仅支持 value
为不含逗号/右括号等分隔符的简单值(数字/布尔/不含引号的片段)，复杂JSON需上层另行处理
---------------------------------------------------------------------------*/
static int parse_params(const char *json, prop_item_t *items, int max_n)
{
    int n = 0;
    const char *p = strstr(json, "\"params\"");
    if (!p)
      return 0;
    p = strchr(p, '[');
    if (!p)
      return 0;

    while (n < max_n)
    {
      p = strchr(p, '{');
      if (!p)
        break;
      const char *end = strchr(p, '}');
      if (!end)
        break;

      const char *s = strstr(p, "\"siid\":");
      if (!s || s > end)
        break;
      items[n].siid = atoi(s + 7);

      s = strstr(p, "\"piid\":");
      if (!s || s > end)
        break;
      items[n].piid = atoi(s + 7);

      items[n].has_value = 0;
      s = strstr(p, "\"value\":");
      if (s && s < end)
      {
        s += 8;
        while (*s == ' ')
          s++;
        int i = 0;
        while (*s && *s != ',' && *s != '}' && *s != ']' && i < 31)
          items[n].value_str[i++] = *s++;
        items[n].value_str[i] = '\0';
        items[n].has_value = 1;
      }

      n++;
      p = end + 1;
    }
    return n;
}

/*---------------------------------------------------------------------------
 Name        : parse_msg_id
 Input       : json - JSON字符串(包含 "id":xxx 字段)
 Output      : id数值(解析失败返回0)
 Description :
 从云端下行JSON报文中提取 "id" 字段，用于生成对应的 result 响应报文。
---------------------------------------------------------------------------*/
static int parse_msg_id(const char *json)
{
    const char *p = strstr(json, "\"id\":");
    if (!p)
      return 0;
    return atoi(p + 5);
}

// ==================== 属性读取（get_properties） ====================

/*---------------------------------------------------------------------------
 Name        : handle_get_properties
 Input       : buf    - 完整JSON报文字符串
                 msg_id - 报文ID(用于应答匹配)
 Output      : 无
 Description :
 处理云端 "get_properties" 请求。
 行为：
 - 调用 `parse_params()` 解析 params 数组，获取请求的 siid/piid 列表
 - 根据本工程定义的SIID/PIID映射，读取对应的运行数据并组装 result 响应：
     - SIID=1：设备信息(型号、SN、软件版本等)
     - SIID=2：CT/电网数据(功率、能量、电压、频率、相序等)
 - 不支持或不存在的属性返回 code=-4004
 - 通过 `ml307r_mqtt_publish()` 将响应发布到上行Topic
---------------------------------------------------------------------------*/
static void handle_get_properties(const char *buf, int msg_id)
{
    DEBUG_4G_PRINTF("[MQTT] >>> handle_get_properties: msg_id=%d, buf=%s\r\n", msg_id, buf);
    prop_item_t items[16];
    int n = parse_params(buf, items, 16);
    if (n == 0)
      return;

    char resp[256];
    int pos = snprintf(resp, sizeof(resp), "{\"id\":%d,\"method\":\"result\",\"params\":[", msg_id);

    for (int i = 0; i < n && pos < (int)sizeof(resp) - 64; i++)
    {
      int siid = items[i].siid;
      int piid = items[i].piid;
      int code = 0;
      char val[32] = "0";

      if (siid == 1)
      {
        switch (piid)
        {
        case 1:
          snprintf(val, sizeof(val), "\"%s\"", PRODUCT_MODEL);
          break;
        case 2:
          snprintf(val, sizeof(val), "\"%s\"", PRODUCT_SN);
          break;
        case 3:
          snprintf(val, sizeof(val), "\"%s\"", SW_VERSION);
          break;
        default:
          code = -4004;
          break;
        }
      }
      else if (siid == 2)
      {
        switch (piid)
        {
        case 1:
          snprintf(val, sizeof(val), "%.1f", sys_param.ct1.power.fix_dir_power);
          break;
        case 2:
          snprintf(val, sizeof(val), "%.1f", sys_param.ct2.power.fix_dir_power);
          break;
        case 3:
          snprintf(val, sizeof(val), "%.1f", sys_param.ct3.power.fix_dir_power);
          break;
        case 4:
          snprintf(val, sizeof(val), "%.1f",
                   sys_param.ct1.power.fix_dir_power +
                       sys_param.ct2.power.fix_dir_power +
                       sys_param.ct3.power.fix_dir_power);
          break;
        case 5:
          snprintf(val, sizeof(val), "%.1f", sys_param.ct_today_energy);
          break;
        case 6:
          snprintf(val, sizeof(val), "%.2f", sys_param.grid.ua_vol_rms);
          break;
        case 7:
          snprintf(val, sizeof(val), "%.2f", sys_param.grid.grid_frequency);
          break;
        case 8:
          snprintf(val, sizeof(val), "%d", sys_param.grid.phase_id.sequence_k);
          break;
        default:
          code = -4004;
          break;
        }
      }
      else
      {
        code = -4004;
      }

      if (i > 0)
        resp[pos++] = ',';
      if (code == 0)
      {
        pos += snprintf(resp + pos, sizeof(resp) - pos,
                        "{\"siid\":%d,\"piid\":%d,\"code\":0,\"value\":%s}", siid, piid, val);
      }
      else
      {
        pos += snprintf(resp + pos, sizeof(resp) - pos,
                        "{\"siid\":%d,\"piid\":%d,\"code\":%d}", siid, piid, code);
      }
    }

    snprintf(resp + pos, sizeof(resp) - pos, "]}");
    DEBUG_4G_PRINTF("[MQTT] <<< handle_get_properties resp: %s\r\n", resp);
    ml307r_mqtt_publish(s_topic_up, resp, 0);
}

// ==================== 属性写入（set_properties） ====================

/*---------------------------------------------------------------------------
 Name        : handle_set_properties
 Input       : buf    - 完整JSON报文字符串
                 msg_id - 报文ID(用于应答匹配)
 Output      : 无
 Description :
 处理云端 "set_properties" 请求。
 当前实现：
 - 解析 params 数组，提取 siid/piid/value
 - 仅保留可扩展框架，默认所有写入项返回 code=-4004(属性不存在/不可写)
 - 通过 `ml307r_mqtt_publish()` 发布 result 应答
 后续扩展建议：
 -
在此处按SIID/PIID实现可写属性，例如开关量、校准参数、运行模式等，并做好范围/权限校验
---------------------------------------------------------------------------*/
static void handle_set_properties(const char *buf, int msg_id)
{
    DEBUG_4G_PRINTF("[MQTT] >>> handle_set_properties: msg_id=%d, buf=%s\r\n", msg_id, buf);
    prop_item_t items[16];
    int n = parse_params(buf, items, 16);
    if (n == 0)
      return;

    char resp[256];
    int pos = snprintf(resp, sizeof(resp), "{\"id\":%d,\"method\":\"result\",\"params\":[", msg_id);

    for (int i = 0; i < n && pos < (int)sizeof(resp) - 48; i++)
    {
      int siid = items[i].siid;
      int piid = items[i].piid;
      int code = -4004;

      if (siid == 2 && items[i].has_value)
      {
        (void)piid;
        code = -4004;
      }

      if (i > 0)
        resp[pos++] = ',';
      pos += snprintf(resp + pos, sizeof(resp) - pos,
                      "{\"siid\":%d,\"piid\":%d,\"code\":%d}", siid, piid, code);
    }

    snprintf(resp + pos, sizeof(resp) - pos, "]}");
    DEBUG_4G_PRINTF("[MQTT] <<< handle_set_properties resp: %s\r\n", resp);
    ml307r_mqtt_publish(s_topic_up, resp, 0);
}

// ==================== 时间同步 ====================

/*---------------------------------------------------------------------------
 Name        : timestamp_to_datetime
 Input       : ts       - Unix时间戳(秒)
                 tz_hours - 时区偏移(小时，例如东八区=8.0)
                 out      - 输出字符串缓冲区
                 out_len  - 输出缓冲区长度
 Output      : 无
 Description :
 将Unix时间戳转换为日期时间字符串 "YYYY-MM-DD HH:MM:SS"。
 实现特点：
 - 不依赖 time.h，使用简单的闰年/月份天数推算
 - 会先将时间戳按时区偏移修正后再转换
---------------------------------------------------------------------------*/
static void timestamp_to_datetime(long ts, float tz_hours, char *out, int out_len)
{
    ts += (long)(tz_hours * 3600.0f);

    long days = ts / 86400;
    long secs = ts % 86400;
    int hh = (int)(secs / 3600);
    int mm = (int)((secs % 3600) / 60);
    int ss = (int)(secs % 60);

    int year = 1970;
    while (1)
    {
      int days_in_year = ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) ? 366 : 365;
      if (days < days_in_year)
        break;
      days -= days_in_year;
      year++;
    }
    static const int mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int month = 1;
    for (int m = 0; m < 12; m++)
    {
      int md = mdays[m];
      if (m == 1 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0))
        md = 29;
      if (days < md)
      {
        month = m + 1;
        break;
      }
      days -= md;
    }
    int day = (int)days + 1;

    snprintf(out, out_len, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hh, mm, ss);
}

/*---------------------------------------------------------------------------
 Name        : handle_time_sync
 Input       : buf - 完整JSON报文字符串(包含 timestamp/timezone 字段)
 Output      : 无
 Description :
 处理云端 "time" 同步消息。
 行为：
 - 解析 "timestamp" 与可选的 "timezone"
 - 调用 `timestamp_to_datetime()` 得到本地时间字符串
 - 更新 `sys_param.time` 中的 date_time/date/time/today_date 等字段
---------------------------------------------------------------------------*/
static void handle_time_sync(const char *buf)
{
    DEBUG_4G_PRINTF("[MQTT] >>> handle_time_sync: buf=%s\r\n", buf);
    const char *p = strstr(buf, "\"timestamp\":");
    if (!p)
      return;
    long ts = atol(p + 12);
    if (ts < 1000000000L)
      return;

    float tz = 8.0f;
    const char *tz_p = strstr(buf, "\"timezone\":");
    if (tz_p)
      tz = (float)atof(tz_p + 11);

    char datetime[20];
    timestamp_to_datetime(ts, tz, datetime, sizeof(datetime));

    strncpy(sys_param.time.date_time, datetime, sizeof(sys_param.time.date_time) - 1);
    sys_param.time.date_time[19] = '\0';
    strncpy(sys_param.time.date, datetime, 10);
    sys_param.time.date[10] = '\0';
    strncpy(sys_param.time.time, datetime + 11, 8);
    sys_param.time.time[8] = '\0';
    sys_param.time.today_date = (uint8_t)atoi(datetime + 8);
    DEBUG_4G_PRINTF("[MQTT] OK - Time synced: %s\r\n", datetime);
}

// ==================== 主消息分发 ====================

/*---------------------------------------------------------------------------
 Name        : on_mqtt_message
 Input       : topic   - 下行topic
                 payload - 下行payload字符串
                 len     - payload长度
 Output      : 无
 Description :
 MQTT下行消息统一入口回调(由 `at_mqtt_register_callback()` 注册)。
 行为：
 - 拷贝payload到本地buf并确保\0结尾
 - 解析 method 与 id 字段
 - 按 method 分发到对应处理函数：
     - get_properties -> `handle_get_properties()`
     - set_properties -> `handle_set_properties()`
     - time -> `handle_time_sync()`
     - ota_start -> 预留(当前未实现)
---------------------------------------------------------------------------*/
static void on_mqtt_message(const char *topic, const char *payload, int len)
{
    (void)topic;
    char buf[512];
    if (len < 0)
      len = 0;
    if (len > (int)(sizeof(buf) - 1))
      len = (int)(sizeof(buf) - 1);
    strncpy(buf, payload, (size_t)len);
    buf[len] = '\0';

    DEBUG_4G_PRINTF("[MQTT] >>> on_mqtt_message: topic=%s, len=%d, payload=%s\r\n", topic, len, buf);

    char method[32] = {0};
    const char *m = strstr(buf, "\"method\":\"");
    if (m)
    {
      m += 10;
      const char *end = strchr(m, '"');
      if (end && (size_t)(end - m) < sizeof(method))
      {
        strncpy(method, m, (size_t)(end - m));
      }
    }

    int msg_id = parse_msg_id(buf);
    DEBUG_4G_PRINTF("[MQTT] method=%s, msg_id=%d\r\n", method, msg_id);

    if (strcmp(method, "get_properties") == 0)
    {
      handle_get_properties(buf, msg_id);
    }
    else if (strcmp(method, "set_properties") == 0)
    {
      handle_set_properties(buf, msg_id);
    }
    else if (strcmp(method, "time") == 0)
    {
      handle_time_sync(buf);
    }
    else if (strcmp(method, "ota_start") == 0)
    {
      DEBUG_4G_PRINTF("[MQTT] !!! ota_start not implemented !!!\r\n");
    }
    else
    {
      DEBUG_4G_PRINTF("[MQTT] !!! Unknown method: %s !!!\r\n", method);
    }
}

// ==================== 主任务 ====================

/*---------------------------------------------------------------------------
 Name        : ml307r_task
 Input       : 无
 Output      : 无
 Description :
 ML307R 4G模块主任务函数，在主循环中调用
---------------------------------------------------------------------------*/
void ml307r_task(void)
{
    static uint32_t last_publish_ms = 0;
    static uint8_t init_done = 0;
    static uint8_t reg_done = 0;

    // 阶段 0: 设备注册（首次运行）
    if (!reg_done)
    {
      DEBUG_4G_PRINTF(" >>> Stage 0: Device registration\r\n");
      if (!device_register_load_from_flash())
      {
        DEBUG_4G_PRINTF(" Device not in flash, requesting registration...\r\n");
        device_register_set_info(PRODUCT_ID, PRODUCT_SECRET, PRODUCT_MODEL, PRODUCT_SN);
        if (device_register_request("") == 0)
        {
          DEBUG_4G_PRINTF(" OK - Device registered successfully\r\n");
          reg_done = 1;
        }
        else
        {
          DEBUG_4G_PRINTF(" !!! Device registration failed, retry in 10s !!!\r\n");
          delay_ms(10000);
          return;
        }
      }
      else
      {
        DEBUG_4G_PRINTF(" OK - Device credentials loaded from flash\r\n");
        reg_done = 1;
      }
    }

    // 阶段 1: ML307R 网络初始化
    if (!init_done)
    {
      DEBUG_4G_PRINTF(" >>> Stage 1: ML307R network init\r\n");
      if (ml307r_init() != 0)
      {
        DEBUG_4G_PRINTF(" !!! ml307r_init failed, retry in 5s !!!\r\n");
        delay_ms(5000);
        return;
      }

      // 阶段 2: MQTT 连接
      DEBUG_4G_PRINTF(" >>> Stage 2: MQTT connection\r\n");
      if (ml307r_mqtt_connect() != 0)
      {
        DEBUG_4G_PRINTF(" !!! ml307r_mqtt_connect failed, retry in 3s !!!\r\n");
        delay_ms(3000);
        return;
      }

      // 注册下行消息回调
      DEBUG_4G_PRINTF(" Registering MQTT callback...\r\n");
      at_mqtt_register_callback(on_mqtt_message);

      // 上报设备信息
      DEBUG_4G_PRINTF(" Publishing device info...\r\n");
      publish_device_info();
      init_done = 1;
      DEBUG_4G_PRINTF(" OK - All init stages complete!\r\n");
    }

    // 阶段 3: 主循环
    uint32_t now = sys_param.timer.timer_1ms_count;

    // 每 5 分钟上报一次 CT 功率
    if ((uint32_t)(now - last_publish_ms) >= 300000U || now < last_publish_ms)
    {
      DEBUG_4G_PRINTF(" >>> Periodic CT power publish (5min interval)\r\n");
      publish_ct_power();
      last_publish_ms = now;
    }

    // 定时发送在线状态（每 30 秒）
    static uint32_t last_online_ms = 0;
    if ((uint32_t)(now - last_online_ms) >= 30000U || now < last_online_ms)
    {
      DEBUG_4G_PRINTF(" >>> Periodic online status publish (30s interval)\r\n");
      ml307r_mqtt_publish(s_topic_up, "{\"status\":\"online\"}", 0);
      last_online_ms = now;
    }
}
