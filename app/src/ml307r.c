#include "ml307r.h"
#include "at_parser.h"
#include "config.h"
#include "crypto.h"
#include "device_register.h"
#include "main.h"
#include "uart_at.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==================== 证书数据定义 ====================

// CA证书数据（PEM格式，每行LF结尾，数据末尾加\n）
#define CA_CERT_DATA \
"-----BEGIN CERTIFICATE-----\n\
MIIDSTCCAjGgAwIBAgIUGDm+nCjNDkxdFsPoXpEhEZcp29YwDQYJKoZIhvcNAQEL\n\
BQAwNDELMAkGA1UEBhMCQ04xCzAJBgNVBAgMAlpKMQswCQYDVQQHDAJIWjELMAkG\n\
A1UECgwCRE0wHhcNMjAwODE5MTAzNjI0WhcNMzAwODE3MTAzNjI0WjA0MQswCQYD\n\
VQQGEwJDTjELMAkGA1UECAwCWkoxCzAJBgNVBAcMAkhaMQswCQYDVQQKDAJETTCC\n\
ASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANsGzJ8WLtQo2XCjA+t4NrFx\n\
gZ9+PCK5Bcvq63ZgRz7uBiS0fDw+9qpOL4SvPmYIAjTj4dlmrk7g4UWk4gHXHxpE\n\
Ia0FX1UorCpDOmcW4PFIqWqgak586ZAW2WkJUJSpzn2qRc7NFKONB9i1Q3F8Fztl\n\
x2n9m2KMkOtCe5pKoiFl34aGdcCphC7nRvaj+EHM85ZbvTEyaLW/J2TFleofUbS0\n\
jNzvPNnkSrol1MI/4RmabY61wI5IT/fr3ADRnwh8GTSm1COe9Cio3Rhes06GWjns\n\
Mrivsf/9KMPdl5YLnnRl8pDB6qYF1Fg7cnj7UmAyfsn44I+GyOGnHisT2OykrYsC\n\
AwEAAaNTMFEwHQYDVR0OBBYEFJ4w7L1q1AjdnMcPsJLZIF0ily4JMB8GA1UdIwQY\n\
MBaAFJ4w7L1q1AjdnMcPsJLZIF0ily4JMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZI\n\
hvcNAQELBQADggEBACWwWG0RkCOlTjzUbZ9yoWBdG/Pq0OrvtNjPc3klSrWsMXr9\n\
5QL9Gd8ZAj2DKEpOcWA99KFxwHj0e5az0/9WL5If1tIt+itHKbzmTKHIZlvTmJXj\n\
U9dj4Ni0JxBW+Y+I2HxfQJQHbBRwIbdn0xIIe2wngzcaGhDgrXpUBMAix/ZI9C6i\n\
pxsEqnv8Y+380bszVVspNmePMbdjMdgDVgAYYm6M1t7AsDuC0Yc5H7CWW2c9Dxj/\n\
x5+/EfAfRODf06WIloS4cVdPaJSVtVnSRo4vZIwNA0Ejbq8QJv/Z0grBwmjbu+La\n\
1CJZrUyESMdK9axtXhhk7k0GCEtptBc0qaEvLys=\n\
-----END CERTIFICATE-----\n"

// 静态数组：用 sizeof(array)-1 在编译期计算字节数
static const char ca_cert_data[] = CA_CERT_DATA;

// 客户端私钥数据（PEM格式，每行LF结尾，数据末尾加\n）
#define CLIENT_KEY_DATA \
"-----BEGIN RSA PRIVATE KEY-----\n\
MIIEowIBAAKCAQEAqlSQxf2Je/pK96IjD9zVSNSmVlFvcFKjLfCvEWBoaYpQnZ6m\n\
cYcMEFubJtaGO/3nh4HdwF0R4hCHgMeGhzsYdeUWrUQchf5xFlEudYOnh6yeEjlu\n\
2XaBvdRYcuaF0Q+aNDFhjrQPx1+tg8QnA9ZM6AdoUhHj010VhCw8f56/IeQX7NIG\n\
FcygQGk0iGTRsKnnkqyzGhuh5TzQo5ulGUyuv7WxxL8VzlKO3of1Ze0f2jatddRa\n\
0PgCD0+cexFR7MSul0A0YW6b8VQ3JcmTU+/czjZMlZaSz2HOCeI1ML1qzEazQDgx\n\
pwzkfXjyhzbJEh+H2GZb8+wqvqYp32eS1YglOQIDAQABAoIBAQCE7Z5Whfln4fvj\n\
qouGc6eYQSzXLJK1rChhT/awrvaNdz5W5FutOeG2WmeJNd5or0yjujwfRgMQ1CmL\n\
3SsmGn6Kc5DxA7jm8Z279vs1BtwVzCdIvb2+xPeX+EJW2YURlQWZOfiS7/9ob0jx\n\
DoIMQpCefDlz4zZT289Q6V0FlBRCE4PW4Kt97GsuK6uletsrZYad5tA7FyyV73bB\n\
rDgzd7pfWRZblZIqiuUpxgxlhKf/TKxxfmE1oxtWlIeQZ3YUqhzoTMtbZb5LfnQf\n\
GactFlP5qDVUTHb25Inp48J5IrTByIP0tWgS9egdl6Y2Y7W2EnV0beAlo/e6NbOG\n\
2um4uy+JAoGBAN9EYG29M7YKReDGPXn/oVZ4XWAQlGjCa7f/OlpBIQ9gIl0mwPaW\n\
PTzL9RlQpYJeHfC9CG4swwd4p/9JmBD31EP12VVZDB7gft/9Ab+AK95garvVip3o\n\
3YApbxF1sbhWuB0WRDvPARairuSBRjCnjZjAtvRBs7hyaR4m6QMMiYC/AoGBAMNN\n\
YAjkThxIBcl08JJP5cUhaQLDVlAvTLkivQotgsFYAp6P0W0SE/DL7uA6y0bGBXAp\n\
B+Tkp0EnjFg0sSFXwyvDGU+mi8ib92NnibNW2KkMhYvxTLf4KvxooAFMw3OSB9iE\n\
iqzlKPgxGfrkkKTHyZrjp1VrWF0BJGGO9VgUAmAHAoGAIu+hIyra/55F5vE+R2vP\n\
xCcbfV+6yVW641TFvb+5O52wYuEgirVhqbTEioyLCYZOqw/5VLweXHnt182dRPJy\n\
vZunwWoosmAwmj4N2vhkZOyzZub4RNNebaKOJa5D+/Nd5fdJAA7ZPcCBG3J7GDvv\n\
tTpapoftZmxItJYkGf0JOyMCgYBzjDXDTQABr4Ls++jRR8ATb37niZZpfnKQHkNl\n\
4CM6LM3v7frj4ww1LRK5S0+1IbdZ8oyfGcDyxWk9Tn910KzCGX3CKDEPkWc+QqWv\n\
mzQ4YBYrA6p2NbeI+oWf/0CsAbvumAxL2pDwtmw+ijghpapbn1EDMq1m6SQTLDdv\n\
0vbMxQKBgE7rOfCD6FZA+VJqWNc6ph/CBwKdt+cMn9QfWWEN8dY5h/WXBOO7Bd4d\n\
EECihd2y2KxYHy4voBuRyBmRCA0tzWvCHro+xnp1A61kpAAHQhOmI2eeRoZYeM5b\n\
mh5Nt/mYrpPOwhbgZIPl5YGiRuVzBFQPWEnFgUR7tLs3XsyLThMW\n\
-----END RSA PRIVATE KEY-----\n"

static const char client_key_data[] = CLIENT_KEY_DATA;

// 客户端证书数据（PEM格式，每行LF结尾，数据末尾加\n）
#define CLIENT_CERT_DATA \
"-----BEGIN CERTIFICATE-----\n\
MIIC+zCCAeMCAQEwDQYJKoZIhvcNAQELBQAwNDELMAkGA1UEBhMCQ04xCzAJBgNV\n\
BAgMAlpKMQswCQYDVQQHDAJIWjELMAkGA1UECgwCRE0wHhcNMjAwODE5MTA1NDEy\n\
WhcNMzAwODE3MTA1NDEyWjBTMQswCQYDVQQGEwJDTjELMAkGA1UECAwCWkoxCzAJ\n\
BgNVBAcMAkhaMQswCQYDVQQKDAJETTEMMAoGA1UECwwDREVWMQ8wDQYDVQQDDAZk\n\
ZXZpY2UwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCqVJDF/Yl7+kr3\n\
oiMP3NVI1KZWUW9wUqMt8K8RYGhpilCdnqZxhwwQW5sm1oY7/eeHgd3AXRHiEIeA\n\
x4aHOxh15RatRByF/nEWUS51g6eHrJ4SOW7ZdoG91Fhy5oXRD5o0MWGOtA/HX62D\n\
xCcD1kzoB2hSEePTXRWELDx/nr8h5Bfs0gYVzKBAaTSIZNGwqeeSrLMaG6HlPNCj\n\
m6UZTK6/tbHEvxXOUo7eh/Vl7R/aNq111FrQ+AIPT5x7EVHsxK6XQDRhbpvxVDcl\n\
yZNT79zONkyVlpLPYc4J4jUwvWrMRrNAODGnDOR9ePKHNskSH4fYZlvz7Cq+pinf\n\
Z5LViCU5AgMBAAEwDQYJKoZIhvcNAQELBQADggEBALZFVhHN7ev8eesGcGPZGzdY\n\
SG0NzEcvB5BRpBhj9UYkvuJ9r45WQB2v+bnHOS6050h3wfz/OFTUzi2gaswZBCLt\n\
heV0OsW30J1ct3tM8bkH1haLZwQKwSJ2Mto9nDyxyPHp5UKMrmJrdRopAGEtibde\n\
ai/tk2e/IKXa46njFZIe2GzuiqYICk4fknOBvFc7VeFNSurJst+KsTcj+d4ll+TD\n\
ofQuGs5K6EQ1r2OQ166Qp74C8S8oXy2izDl57rDNzSpJxGi91zpIFPwHbLhtBqqX\n\
A2tnu2PUsyhBa/GAkHk/WRMF30suq1mwb/josIbIW7hqgfs8hFQ+wYNyvgtc4ms=\n\
-----END CERTIFICATE-----\n"

static const char client_cert_data[] = CLIENT_CERT_DATA;

// ==================== ML307R 状态机 ====================

static ml307r_state_t s_ml_state = ML307R_STATE_INIT;

/*---------------------------------------------------------------------------
 Name        : int ml307r_init(void)
 Input       : 无
 Output      : 0=成功, -1=失败
 Description : ML307R 4G模组初始化。依次执行以下步骤：
               1. AT通信测试
               2. 关闭回显(ATE0)
               3. 检查SIM卡状态(AT+CPIN?)
               4. 等待网络注册(AT+CEREG?)，超时30秒
               5. 激活PDP上下文(AT+MIPCALL=1,1)
---------------------------------------------------------------------------*/
int ml307r_init(void)
{
    int ret;
    char resp[128];

    s_ml_state = ML307R_STATE_INIT;
    DEBUG_4G_PRINTF(" >>> ml307r_init start\r\n");

    // ML307R 启动需要时间，先等待一下
    DEBUG_4G_PRINTF(" >>> Waiting for ML307R to be ready...\r\n");
    delay_ms(2000);  // 等待2秒让ML307R完全就绪

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

    // 4. 查询信号质量 AT+CSQ，建议信号 > 18
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

    // 7. 等待网络注册 AT+CEREG?，最多 30 次，每次 1s
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

    // 8. 激活 PDP 上下文 AT+MIPCALL=1,1
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
 Name        : ml307r_state_t ml307r_get_state(void)
 Input       : 无
 Output      : ml307r_state_t - 当前ML307R模组状态
 Description : 获取ML307R 4G模组的当前状态，包含以下状态：
               - ML307R_STATE_INIT: 初始化状态
               - ML307R_STATE_SIM_CHECK: SIM卡检查状态
               - ML307R_STATE_REGISTERED: 已完成注册
               - ML307R_STATE_DIAL: PDP激活状态
               - ML307R_STATE_CONNECTED: 连接成功
               - ML307R_STATE_ERROR: 错误状态
---------------------------------------------------------------------------*/
ml307r_state_t ml307r_get_state(void)
{
    DEBUG_4G_PRINTF(" >>> ml307r_get_state: %d\r\n", s_ml_state);
    return s_ml_state;
}

/*---------------------------------------------------------------------------
 Name        : int ml307r_get_signal_quality(signal_quality_t *sq)
 Input       : sq - signal_quality_t结构体指针，用于接收信号质量
 Output      : 0=成功, -1=失败
 Description : 获取ML307R 4G模组的信号质量，通过AT+CSQ命令查询。
               返回RSSI（信号强度）和BER（误码率）两个指标。
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
 Name        : bool ml307r_is_arrears(void)
 Input       : 无
 Output      : true=欠费（信号正常但未注册）, false=正常
 Description : 判断ML307R模组是否处于欠费状态。
               条件：RSSI在10-31范围内（信号良好），但状态不是已注册/激活/连接中。
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
 Name        : int ml307r_reconnect(void)
 Input       : 无
 Output      : 0=成功, -1=失败
 Description : ML307R 4G模组重新连接。依次执行以下步骤：
               1. 关闭无线功能(AT+CFUN=0)
               2. 开启无线功能(AT+CFUN=1)
               3. 等待重新网络注册(AT+CEREG?)，超时30秒
               4. 重新激活PDP上下文(AT+MIPCALL=1,1)
---------------------------------------------------------------------------*/
int ml307r_reconnect(void)
{
    char resp[128];
    DEBUG_4G_PRINTF(" >>> ml307r_reconnect start\r\n");

    DEBUG_4G_PRINTF(" >>> AT send: AT+CFUN=0\r\n");
    int ret0 = at_send_command("AT+CFUN=0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CFUN=0 resp: %s, ret=%d, err=%d(%s)\r\n",
                    resp, ret0, at_get_last_error_code(), at_get_last_error_line());
    if (ret0 != 0)
    {
      DEBUG_4G_PRINTF(" !!! CFUN=0 failed !!!\r\n");
    }
    delay_ms(500);

    DEBUG_4G_PRINTF(" >>> AT send: AT+CFUN=1\r\n");
    int ret1 = at_send_command("AT+CFUN=1", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    DEBUG_4G_PRINTF(" <<< AT+CFUN=1 resp: %s, ret=%d, err=%d(%s)\r\n",
                    resp, ret1, at_get_last_error_code(), at_get_last_error_line());
    if (ret1 != 0)
    {
      DEBUG_4G_PRINTF(" !!! CFUN=1 failed !!!\r\n");
    }
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
 Name        : int ml307r_mqtt_connect(void)
 Input       : 无
 Output      : 0=成功, -1=失败
 Description : MQTT客户端连接。
---------------------------------------------------------------------------*/
int ml307r_mqtt_connect(void)
{
    int ret;
    s_mqtt_state = MQTT_STATE_CONNECTING;
    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_connect start\r\n");

    // 获取设备凭证
    const device_credentials_t *cred = &g_device_cred;
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

    // 使用动态凭证连接 MQTT
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
 Name        : int ml307r_mqtt_disconnect(void)
 Input       : 无
 Output      : 0=成功, -1=失败
 Description : MQTT客户端断开连接。
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
 Name        : int ml307r_mqtt_publish(const char *topic, const char *payload, int qos)
 Input       : topic - 主题
               payload - 消息载荷
               qos - 消息服务质量(0/1)
 Output      : at_mqtt_publish的返回值
 Description : MQTT发布消息，将消息数据发送到指定的主题。
---------------------------------------------------------------------------*/
int ml307r_mqtt_publish(const char *topic, const char *payload, int qos)
{
    // 短名映射："up" → s_topic_up, "down" → s_topic_down
    const char *real_topic = topic;
    if (topic != NULL && strcmp(topic, "up") == 0 && s_topic_up[0] != '\0')
        real_topic = s_topic_up;
    else if (topic != NULL && strcmp(topic, "down") == 0 && s_topic_down[0] != '\0')
        real_topic = s_topic_down;

    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_publish: topic=%s, qos=%d\r\n", real_topic, qos);
    int ret = at_mqtt_publish(real_topic, qos, payload, (int)strlen(payload));
    DEBUG_4G_PRINTF("[MQTT] <<< ml307r_mqtt_publish ret=%d\r\n", ret);
    return ret;
}

/*---------------------------------------------------------------------------
 Name        : mqtt_state_t ml307r_mqtt_get_state(void)
 Input       : 无
 Output      : mqtt_state_t - 当前MQTT连接状态
 Description : 获取MQTT客户端的当前状态。
---------------------------------------------------------------------------*/
mqtt_state_t ml307r_mqtt_get_state(void)
{
    DEBUG_4G_PRINTF("[MQTT] >>> ml307r_mqtt_get_state: %d\r\n", s_mqtt_state);
    return s_mqtt_state;
}

/*---------------------------------------------------------------------------
 Name        : bool ml307r_mqtt_is_connected(void)
 Input       : 无
 Output      : true=已连接且订阅完成, false=未连接
 Description : 供外部模块（iot.c等）查询MQTT是否处于可发布状态。
---------------------------------------------------------------------------*/
bool ml307r_mqtt_is_connected(void)
{
    return (ml307r_mqtt_get_state() == MQTT_STATE_CONNECTED);
}

// ==================== 上行消息发布 ====================

static void on_mqtt_message(const char *topic, const char *payload, int len);

// MQTT ID 计数器
static int s_mqtt_msg_id = 0;

/*---------------------------------------------------------------------------
 Name        : publish_device_info(void)
 Input       : 无
 Output      : 无
 Description : 上报设备信息到平台。
               包含设备SN、产品型号、MCU版本、RSSI信号强度。
---------------------------------------------------------------------------*/
static void publish_device_info(void)
{
    const device_credentials_t *cred = &g_device_cred;
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
 Name        : publish_ct_power(void)
 Input       : 无
 Output      : 无
 Description : 上报CT功率数据到平台。
               包含三路CT中功率和当天累计发电量数据。
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

// ==================== JSON 属性解析工具 ====================

typedef struct
{
    int siid;
    int piid;
    char value_str[32];
    int has_value;
} prop_item_t;

/*---------------------------------------------------------------------------
 Name        : parse_params(const char *json, prop_item_t *items, int max_n)
 Input       : json  - JSON字符串（包含 "params":[{...},...] 的消息体）
               items - 输出数组，用于保存解析的属性项(siids/piid/value)
               max_n - items数组的最大容量
 Output      : 实际解析到的属性项数（0表示未解析到或格式不匹配）
 Description : 从平台下行JSON消息中解析 "params" 属性字段，提取每一项：
               - siid / piid
               - value（可选），统一保存为字符串 value_str，设置位 has_value
               用途：
               - handle_get_properties() 用于 get_properties 请求（siids/piid）
               - handle_set_properties() 用于 set_properties 请求（siids/piid/value）
               说明：实现方式为轻量级字符串扫描/提取，不通过JSON库解析
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
 Name        : parse_msg_id(const char *json)
 Input       : json - JSON字符串（包含 "id":xxx 字段）
 Output      : id的值（解析失败返回0）
 Description : 从平台下行JSON消息中提取 "id" 字段，用于生成对应的 result 响应。
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
 Name        : handle_get_properties(const char *buf, int msg_id)
 Input       : buf    - 下行JSON消息字符串
               msg_id - 消息ID（用于响应匹配）
 Output      : 无
 Description : 处理平台下发的 "get_properties" 请求。
               处理流程：
               - 调用 parse_params() 解析 params 数组，获取请求的 siid/piid 列表
               - 按照固定的SIID/PIID映射，获取对应的属性数据并填装 result 响应：
                   - SIID=1：设备信息（型号、SN、软件版本号）
                   - SIID=2：CT/电网数据（功率、电量、电压、频率、相序）
               - 不支持或不存在的属性返回 code=-4004
               - 通过 ml307r_mqtt_publish() 发送响应到上行Topic
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
 Name        : handle_set_properties(const char *buf, int msg_id)
 Input       : buf    - 下行JSON消息字符串
               msg_id - 消息ID（用于响应匹配）
 Output      : 无
 Description : 处理平台下发的 "set_properties" 请求。
               当前实现：
               - 解析 params 数组，获取 siid/piid/value
               - 暂无可写属性扩展，默认返回 code=-4004（不可写/不支持写）
               - 通过 ml307r_mqtt_publish() 发送 result 响应
               未来扩展：
               - 在此处按SIID/PIID实现可写属性，如开关校准、工作模式等，需做范围/权限校验
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
 Name        : timestamp_to_datetime(long ts, float tz_hours, char *out, int out_len)
 Input       : ts       - Unix时间戳（秒）
               tz_hours - 时区偏移（小时，如东八区=8.0）
               out      - 输出字符串缓冲区
               out_len  - 输出缓冲区长度
 Output      : 无
 Description : 将Unix时间戳转换为格式化时间字符串 "YYYY-MM-DD HH:MM:SS"。
               实现要点：
               - 不依赖 time.h，使用简单的年/月算术运算
               - 先将时间戳加上时区偏移再进行转换
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
 Name        : handle_time_sync(const char *buf)
 Input       : buf - 下行JSON消息字符串（包含 timestamp/timezone 字段）
 Output      : 无
 Description : 处理平台下发的 "time" 同步消息。
               处理流程：
               - 解析 "timestamp" 和可选的 "timezone"
               - 调用 timestamp_to_datetime() 得到格式化时间字符串
               - 更新 sys_param.time 中的 date_time/date/time/today_date 各字段
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

// ==================== 消息分发 ====================

/*---------------------------------------------------------------------------
 Name        : on_mqtt_message(const char *topic, const char *payload, int len)
 Input       : topic   - 消息主题
               payload - 消息载荷字符串
               len     - payload长度
 Output      : 无
 Description : MQTT下行消息统一入口回调（由 at_mqtt_register_callback() 注册）。
               处理流程：
               - 将payload复制到buf，确保\0结尾
               - 解析 method 和 id 字段
               - 按 method 字符串分发到对应处理函数：
                   - get_properties -> handle_get_properties()
                   - set_properties -> handle_set_properties()
                   - time -> handle_time_sync()
                   - ota_start -> 预留（当前未实现）
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

// ML307R 初始化子状态
typedef enum {
    ML_SUB_AT,           // 阶段1: AT通信测试
    ML_SUB_ATE0,         // 阶段1: 关闭回显
    ML_SUB_CPIN,         // 阶段1: 检查SIM卡
    ML_SUB_CSQ,          // 阶段1: 查询信号强度
    ML_SUB_CEREG,        // 阶段1: 等待网络注册
    ML_SUB_CEREG_WAIT,    // 阶段1: 循环检测等待
    ML_SUB_CEREG_DELAY,  // 阶段1: 重试等待延时(3s)
    ML_SUB_MIPCALL,       // 阶段1: 激活PDP
    ML_SUB_MIPCALL_DEACT, // 阶段1: 去激活PDP（CME ERROR 50后重试）
    ML_SUB_CERT_CA,       // 阶段2: 写入CA证书
    ML_SUB_CERT_KEY,      // 阶段3: 写入客户端私钥
    ML_SUB_CERT_DEV,      // 阶段3: 写入客户端证书
    ML_SUB_HTTPS_CHECK,    // 阶段4: 检查是否已有凭证
    ML_SUB_HTTPS_SSL_CFG,  // 阶段4: SSL单向认证配置
    ML_SUB_HTTPS_CREATE,   // 阶段4: 创建HTTP实例
    ML_SUB_HTTPS_HEADER,   // 阶段4: 配置HTTP头
    ML_SUB_HTTPS_CONTENT,  // 阶段4: 发送JSON body（第一步：发header）
    ML_SUB_HTTPS_CONTENT2, // 阶段4: 发送JSON body（第二步：发MHTTPCONTENT命令）
    ML_SUB_HTTPS_CONTENT3, // 阶段4: 发送JSON body（第三步：等待>并发送body）
    ML_SUB_HTTPS_REQUEST,  // 阶段4: 发送HTTP POST请求
    ML_SUB_HTTPS_WAIT_URC, // 阶段4: 等待URC响应(解析device_id/key)
    ML_SUB_HTTPS_CLEANUP,  // 阶段4: 删除HTTP实例
    ML_SUB_SSL_AUTH,      // 阶段5: SSL双向认证
    ML_SUB_SSL_AUTH_WAIT,  // 阶段5: 等待SSL证书绑定完成
    ML_SUB_SSL_AUTH_WAIT2, // 阶段5: 等待双向认证设置完成
    ML_SUB_MQTT_CONN,          // 阶段6: MQTT连接（发SSL配置）
    ML_SUB_MQTT_CONN_WAIT1,    // 阶段6: 等待keepalive设置完成
    ML_SUB_MQTT_CONN_WAIT2,    // 阶段6: 等待clean设置完成，发AT+MQTTCONN
    ML_SUB_MQTT_SUB,           // 阶段6: 等待AT+MQTTCONN模组接受(OK)
    ML_SUB_MQTT_CONN_URC_WAIT, // 阶段6: 等待+MQTTURC:"conn"确认实际连接
    ML_SUB_MQTT_SUB_WAIT,      // 阶段7: 已订阅，等待AT+MQTTSUB完成
    ML_SUB_DONE,               // 全部完成
} ml_sub_state_t;

// 证书写入步骤（每张证书写入独立状态机）
typedef enum {
    CERT_STEP_CMD,        // 发命令头部，等待>
    CERT_STEP_DATA,       // 等待>后写入证书数据
    CERT_STEP_WAIT_OK,    // 等待写入完成（OK）
} cert_step_t;

// 静态变量
static ml_sub_state_t s_ml_sub_state = ML_SUB_AT;
static uint32_t s_wait_until = 0;
static uint8_t s_init_done = 0;
static uint8_t s_cereg_retry = 0;
static uint32_t s_cereg_delay_start = 0;   // ML_SUB_CEREG_DELAY 独立计时起点
static int s_cereg_stat = -1;              // +CEREG: <n>,<stat> 中的 stat 字段（1=home, 5=roaming）
static cert_step_t s_cert_step = CERT_STEP_CMD;  // 证书写入步骤
static uint32_t s_cert_wait_start = 0;            // 证书等待计时起点

// HTTPS注册状态机用静态变量
static uint8_t s_https_reg_done = 0;              // HTTPS注册完成标志(device_id/key已解析)
static uint8_t s_urc_registered = 0;              // URC回调已注册标志
static uint32_t s_https_body_len = 0;             // JSON body字节长度
static char s_reg_device_id[32];                  // 解析出的device_id
static char s_reg_device_key[64];                 // 解析出的device_key
static char s_json_body[256];                     // JSON body缓冲区（用于HTTPS注册）

// MQTT重连状态
static uint8_t s_mqtt_retry = 0;                  // MQTT连接重试次数（超限后全量重启）
static uint32_t s_mqtt_urc_wait_start = 0;        // 等待MQTT conn URC的计时起点

// 等待一段时间（毫秒，非阻塞）
// 返回1=等待结束, 0=还在等待
static int ml_wait_ms(uint32_t ms)
{
    if (s_wait_until == 0)
    {
        s_wait_until = SysTick_GetTick() + ms;
        return 0;
    }
    if ((SysTick_GetTick() - s_wait_until) >= ms)
    {
        s_wait_until = 0;
        return 1; // 等待结束
    }
    return 0;
}

// ==================== URC回调：解析 device_id / device_key ====================

// URC回调：解析 +MHTTPURC "content" 响应中的 device_id 和 device_key
static void https_urc_callback(const char *line)
{
    if (s_https_reg_done)
        return; // 已解析过，跳过

    // 调试：打印收到的 URC 行
    DEBUG_4G_PRINTF(" DBG URC: [%s]\r\n", line);

    // JSON 格式：{"code":0,"msg":"OK","data":{"device_key":"xxx","device_id":"yyy"}}
    // device_key 和 device_id 的顺序不固定，各自从 line 头部独立搜索

    // 提取 device_id（"device_id":"<value>"）
    // "device_id":"  共13个字符
    const char *p = strstr(line, "\"device_id\":\"");
    if (p == NULL) {
        DEBUG_4G_PRINTF(" DBG URC: device_id pattern not found\r\n");
        return;
    }
    p += 13; // 跳过 "device_id":" (13个字符，使 p 指向值的第一个字符)
    const char *end = strchr(p, '"');
    if (end == NULL || end <= p) {
        DEBUG_4G_PRINTF(" DBG URC: device_id end quote not found\r\n");
        return;
    }
    int len = (int)(end - p);
    if (len >= (int)sizeof(s_reg_device_id)) {
        len = sizeof(s_reg_device_id) - 1;
    }
    memcpy(s_reg_device_id, p, len);
    s_reg_device_id[len] = '\0';

    // 提取 device_key（"device_key":"<value>"）从 line 开头搜索（顺序无关）
    // "device_key":"  共14个字符
    p = strstr(line, "\"device_key\":\"");
    if (p == NULL) {
        DEBUG_4G_PRINTF(" DBG URC: device_key pattern not found\r\n");
        return;
    }
    p += 14; // 跳过 "device_key":" (14个字符，使 p 指向值的第一个字符)
    end = strchr(p, '"');
    if (end == NULL || end <= p) {
        DEBUG_4G_PRINTF(" DBG URC: device_key end quote not found\r\n");
        return;
    }
    len = (int)(end - p);
    if (len >= (int)sizeof(s_reg_device_key)) {
        len = sizeof(s_reg_device_key) - 1;
    }
    memcpy(s_reg_device_key, p, len);
    s_reg_device_key[len] = '\0';

    // 保存到 device_register 模块
    device_register_set_credentials(s_reg_device_id, s_reg_device_key);
    device_register_save_to_flash();

    s_https_reg_done = 1;
    DEBUG_4G_PRINTF(" OK - Device registered: ID=%s, KEY=%s\r\n", s_reg_device_id, s_reg_device_key);
}

// 发送证书内容（不带命令头，用于第二步）
static void send_cert_data(const char *cert_data)
{
    uint16_t len = (uint16_t)strlen(cert_data);
    for (uint16_t i = 0; i < len; i++)
    {
        while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET)
            ;
        USART_WriteData(CM_USART2, (uint16_t)cert_data[i]);
    }
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_CPLT) == RESET)
        ;
}

// ==================== 错误码描述 ====================

// 返回 ML307R 模组错误码的人类可读描述
static const char *ml307r_err_str(int code)
{
    switch (code) {
    /* SSL */
    case ML307R_SSL_ERR_PARAM:                   return "SSL: invalid param";
    case ML307R_SSL_ERR_UNKNOWN:                 return "SSL: unknown error";
    case ML307R_SSL_ERR_SERVER_CERT_VERIFY_FAIL: return "SSL: server cert verify fail";
    case ML307R_SSL_ERR_NEGOTIATE_TIMEOUT:       return "SSL: negotiate timeout";
    case ML307R_SSL_ERR_NEGOTIATE_FAIL:          return "SSL: negotiate fail";
    case ML307R_SSL_ERR_CERTKEY_INVALID:         return "SSL: cert/key invalid";
    case ML307R_SSL_ERR_CERTKEY_NOT_EXIST:       return "SSL: cert/key not exist";
    case ML307R_SSL_ERR_CERTKEY_WRITE_FAIL:      return "SSL: cert/key write fail";
    case ML307R_SSL_ERR_CERTKEY_TOO_LARGE:       return "SSL: cert/key too large";
    /* MQTT */
    case ML307R_MQTT_ERR_UNKNOWN:                return "MQTT: unknown error";
    case ML307R_MQTT_ERR_INVALID_PARAM:          return "MQTT: invalid param";
    case ML307R_MQTT_ERR_NOT_CONNECTED_OR_CONN_FAIL: return "MQTT: not connected or conn fail";
    case ML307R_MQTT_ERR_CONNECTING:             return "MQTT: connecting";
    case ML307R_MQTT_ERR_ALREADY_CONNECTED:      return "MQTT: already connected";
    case ML307R_MQTT_ERR_NETWORK:                return "MQTT: network error";
    case ML307R_MQTT_ERR_STORAGE:                return "MQTT: storage error";
    case ML307R_MQTT_ERR_STATE:                  return "MQTT: state error (not ready)";
    case ML307R_MQTT_ERR_DNS:                    return "MQTT: DNS error";
    /* HTTP */
    case ML307R_HTTP_ERR_NO_FREE_CLIENT:         return "HTTP: no free client";
    case ML307R_HTTP_ERR_CLIENT_BUSY:            return "HTTP: client busy";
    case ML307R_HTTP_ERR_URL_PARSE_FAIL:         return "HTTP: URL parse fail";
    case ML307R_HTTP_ERR_SSL_NOT_ENABLED:        return "HTTP: SSL not enabled";
    case ML307R_HTTP_ERR_CONNECT_FAIL:           return "HTTP: connect fail";
    case ML307R_HTTP_ERR_SEND_FAIL:              return "HTTP: send fail";
    /* TCP/IP */
    case ML307R_TCPIP_ERR_PDP_NOT_ACTIVATED:     return "TCPIP: PDP not activated";
    case ML307R_TCPIP_ERR_DNS_PARSE_FAIL_OR_BAD_IP: return "TCPIP: DNS parse fail";
    default:
        if (code >= 750 && code <= 769) return "SSL: cert/key operation error";
        if (code >= 600 && code <= 608) return "MQTT error";
        if (code >= 650 && code <= 658) return "HTTP error";
        if (code >= 550 && code <= 582) return "TCPIP error";
        return "unknown error";
    }
}

// MQTT conn URC 结果说明
static const char *mqtt_conn_result_str(int code)
{
    switch (code) {
    case 0: return "connected";
    case 1: return "timeout";
    case 2: return "client disconnect";
    case 3: return "server refused";
    case 4: return "server disconnect";
    case 5: return "PING timeout";
    case 6: return "network error";
    default: return "unknown";
    }
}

// URC回调：捕获 +CEREG: <n>,<stat> 中的网络注册状态
// stat: 1=home registered, 5=roaming registered, 0/2/3/4=not registered
static void cereg_urc_callback(const char *line)
{
    if (line == NULL)
        return;
    int n = -1, stat = -1;
    if (sscanf(line, "+CEREG: %d,%d", &n, &stat) == 2) {
        s_cereg_stat = stat;
    } else if (sscanf(line, "+CEREG: %d", &stat) == 1) {
        // URC 格式（无n）: +CEREG: <stat>
        s_cereg_stat = stat;
    }
}

/*---------------------------------------------------------------------------
 Name        : ml307r_task
 Input       : 无
 Output      : 无
 Description :
 ML307R 4G模组主任务，在主循环中调用，实现状态机驱动
---------------------------------------------------------------------------*/
void ml307r_task(void)
{
    int ret;
    char resp[128];

    // ========== 初始化状态机 ==========
    if (!s_init_done)
    {
        // 注册 URC 回调（仅注册一次）
        if (!s_urc_registered) {
            at_register_urc("+MHTTPURC", https_urc_callback);  // HTTPS 内容解析
            at_register_urc("+CEREG:", cereg_urc_callback);     // 网络注册状态捕获
            at_mqtt_register_callback(on_mqtt_message);          // MQTT URC + 消息回调
            s_urc_registered = 1;
        }

        switch (s_ml_sub_state)
        {
        case ML_SUB_AT:
            DEBUG_4G_PRINTF(" >>> [1] AT test\r\n");
            at_command_start("AT", 3000);
            s_ml_sub_state = ML_SUB_ATE0;
            break;

        case ML_SUB_ATE0:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - AT passed\r\n");
                DEBUG_4G_PRINTF(" >>> [2] Disable echo\r\n");
                at_command_start("ATE0", 3000);
                s_ml_sub_state = ML_SUB_CPIN;
            } else if (ret == AT_NB_ERR) {
                DEBUG_4G_PRINTF(" !!! AT failed, retry...\r\n");
                at_command_start("AT", 3000); // 重试
            }
            break;

        case ML_SUB_CPIN:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - Echo disabled\r\n");
                DEBUG_4G_PRINTF(" >>> [3] Check SIM\r\n");
                at_command_start("AT+CPIN?", 3000);
                s_ml_sub_state = ML_SUB_CSQ;
            } else if (ret == AT_NB_ERR) {
                DEBUG_4G_PRINTF(" !!! ATE0 failed, retry...\r\n");
                s_ml_sub_state = ML_SUB_AT;
            }
            break;

        case ML_SUB_CSQ:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - SIM ready\r\n");
                DEBUG_4G_PRINTF(" >>> [4] Check signal\r\n");
                at_command_start("AT+CSQ", 3000);
                s_ml_sub_state = ML_SUB_CEREG;
            } else if (ret == AT_NB_ERR) {
                DEBUG_4G_PRINTF(" !!! CPIN failed, retry...\r\n");
                s_ml_sub_state = ML_SUB_AT;
            }
            break;

        case ML_SUB_CEREG:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - Signal checked\r\n");
                DEBUG_4G_PRINTF(" >>> [5] Wait network registration\r\n");
                s_cereg_retry = 0;
                at_command_start("AT+CEREG?", 5000);
                s_ml_sub_state = ML_SUB_CEREG_WAIT;
            } else if (ret == AT_NB_ERR) {
                DEBUG_4G_PRINTF(" !!! CSQ failed, retry...\r\n");
                s_ml_sub_state = ML_SUB_AT;
            }
            break;

        case ML_SUB_CEREG_WAIT:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                // s_cereg_stat 由 cereg_urc_callback 在收到 +CEREG: 行时更新
                // stat=1 (home) 或 stat=5 (roaming) 代表已注册
                if (s_cereg_stat == 1 || s_cereg_stat == 5) {
                    DEBUG_4G_PRINTF(" OK - Network registered (stat=%d)\r\n", s_cereg_stat);
                    DEBUG_4G_PRINTF(" >>> [6] Activate PDP\r\n");
                    at_command_start("AT+MIPCALL=1,1", 10000);
                    s_ml_sub_state = ML_SUB_MIPCALL;
                } else {
                    // 已注册命令OK但stat不合法，等待重试
                    s_cereg_retry++;
                    DEBUG_4G_PRINTF(" >>> CEREG stat=%d not registered, retry %d/30\r\n",
                                    s_cereg_stat, s_cereg_retry);
                    if (s_cereg_retry >= 30) {
                        DEBUG_4G_PRINTF(" !!! CEREG timeout, restart...\r\n");
                        s_ml_sub_state = ML_SUB_AT;
                    } else {
                        s_cereg_delay_start = SysTick_GetTick();
                        s_ml_sub_state = ML_SUB_CEREG_DELAY;
                    }
                }
            } else if (ret == AT_NB_ERR) {
                s_cereg_retry++;
                DEBUG_4G_PRINTF(" >>> CEREG retry %d/30\r\n", s_cereg_retry);
                if (s_cereg_retry >= 30) {
                    DEBUG_4G_PRINTF(" !!! CEREG timeout, restart...\r\n");
                    s_ml_sub_state = ML_SUB_AT;
                } else {
                    s_cereg_delay_start = SysTick_GetTick();
                    s_ml_sub_state = ML_SUB_CEREG_DELAY;
                }
            }
            break;

        case ML_SUB_CEREG_DELAY:
            at_command_check();
            if ((SysTick_GetTick() - s_cereg_delay_start) >= 3000) {
                DEBUG_4G_PRINTF(" >>> Retrying CEREG query...\r\n");
                at_command_start("AT+CEREG?", 5000);
                s_ml_sub_state = ML_SUB_CEREG_WAIT;
            }
            break;

        case ML_SUB_MIPCALL:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - PDP activated\r\n");
                s_cert_step = CERT_STEP_CMD;
                s_ml_sub_state = ML_SUB_CERT_CA;
            } else if (ret == AT_NB_ERR) {
                int err_code = at_get_last_error_code();
                DEBUG_4G_PRINTF(" !!! MIPCALL failed (err=%d)\r\n", err_code);
                if (err_code == 50) {
                    // CME ERROR 50: PDP context already active, deactivate first
                    DEBUG_4G_PRINTF(" >>> PDP already active, deactivating...\r\n");
                    at_command_start("AT+MIPCALL=0,1", 5000);
                    s_ml_sub_state = ML_SUB_MIPCALL_DEACT;
                } else {
                    // Other errors: retry from CEREG (not full restart)
                    DEBUG_4G_PRINTF(" >>> Retry from network check\r\n");
                    s_cereg_retry = 0;
                    s_ml_sub_state = ML_SUB_CEREG;
                }
            }
            break;

        case ML_SUB_MIPCALL_DEACT:
            ret = at_command_check();
            if (ret == AT_NB_OK || ret == AT_NB_ERR) {
                DEBUG_4G_PRINTF(" OK - PDP deactivated, retrying activation\r\n");
                at_command_start("AT+MIPCALL=1,1", 10000);
                s_ml_sub_state = ML_SUB_MIPCALL;
            }
            break;

        case ML_SUB_CERT_CA:
            if (s_cert_step == CERT_STEP_CMD)
            {
                // 步骤1：发送命令头部，等待>提示符
                char cert_cmd[64];
                snprintf(cert_cmd, sizeof(cert_cmd),
                         "AT+MSSLCERTWR=\"ca.cer\",0,%d\r\n",
                         (int)(sizeof(ca_cert_data) - 1));
                DEBUG_4G_PRINTF(" >>> [7] Write CA cert step1: %s", cert_cmd);
                at_flush_rx();
                at_send_raw((const uint8_t *)cert_cmd, (uint16_t)strlen(cert_cmd));
                s_cert_wait_start = SysTick_GetTick();
                s_cert_step = CERT_STEP_DATA;
            }
            else if (s_cert_step == CERT_STEP_DATA)
            {
                // 步骤2：等待>提示符，通过 at_got_prompt flag，避免占用 ring buffer
                if (at_got_prompt())
                {
                    DEBUG_4G_PRINTF(" OK - Got > for CA cert, sending data...\r\n");
                    send_cert_data(ca_cert_data);
                    s_cert_wait_start = SysTick_GetTick();
                    s_cert_step = CERT_STEP_WAIT_OK;
                }
                else if ((SysTick_GetTick() - s_cert_wait_start) > 8000)
                {
                    // 打印原始 RX 数据，观察模组实际返回了什么
                    char dbg_buf[64];
                    int n = at_read_response(dbg_buf, sizeof(dbg_buf));
                    DEBUG_4G_PRINTF(" !!! CA cert: no > in 8s, RX(%d)=[%s]\r\n", n, dbg_buf);
                    s_cert_step = CERT_STEP_CMD;
                }
            }
            else if (s_cert_step == CERT_STEP_WAIT_OK)
            {
                // 步骤3：等待OK，通过 at_check_last_result flag
                int r = at_check_last_result();
                if (r == 1)
                {
                    DEBUG_4G_PRINTF(" OK - CA cert written!\r\n");
                    s_cert_step = CERT_STEP_CMD;
                    s_ml_sub_state = ML_SUB_CERT_KEY;
                }
                else if (r == -1)
                {
                    DEBUG_4G_PRINTF(" !!! CA cert write ERROR, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
                else if ((SysTick_GetTick() - s_cert_wait_start) > 30000)
                {
                    DEBUG_4G_PRINTF(" !!! CA cert: no OK in 30s, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
            }
            break;

        case ML_SUB_CERT_KEY:
            if (s_cert_step == CERT_STEP_CMD)
            {
                // 步骤1：发送命令头部，等待>提示符
                char key_cmd[64];
                snprintf(key_cmd, sizeof(key_cmd),
                         "AT+MSSLKEYWR=\"client.key\",0,%d\r\n",
                         (int)(sizeof(client_key_data) - 1));
                DEBUG_4G_PRINTF(" >>> [8] Write client key step1: %s", key_cmd);
                at_flush_rx();
                at_send_raw((const uint8_t *)key_cmd, (uint16_t)strlen(key_cmd));
                s_cert_wait_start = SysTick_GetTick();
                s_cert_step = CERT_STEP_DATA;
            }
            else if (s_cert_step == CERT_STEP_DATA)
            {
                if (at_got_prompt())
                {
                    DEBUG_4G_PRINTF(" OK - Got > for client key, sending data...\r\n");
                    send_cert_data(client_key_data);
                    s_cert_wait_start = SysTick_GetTick();
                    s_cert_step = CERT_STEP_WAIT_OK;
                }
                else if ((SysTick_GetTick() - s_cert_wait_start) > 8000)
                {
                    DEBUG_4G_PRINTF(" !!! Client key: no > in 8s, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
            }
            else if (s_cert_step == CERT_STEP_WAIT_OK)
            {
                int r = at_check_last_result();
                if (r == 1)
                {
                    DEBUG_4G_PRINTF(" OK - Client key written!\r\n");
                    s_cert_step = CERT_STEP_CMD;
                    s_ml_sub_state = ML_SUB_CERT_DEV;
                }
                else if (r == -1)
                {
                    DEBUG_4G_PRINTF(" !!! Client key write ERROR, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
                else if ((SysTick_GetTick() - s_cert_wait_start) > 30000)
                {
                    DEBUG_4G_PRINTF(" !!! Client key: no OK in 30s, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
            }
            break;

        case ML_SUB_CERT_DEV:
            if (s_cert_step == CERT_STEP_CMD)
            {
                // 步骤1：发送命令头部，等待>提示符
                char dev_cmd[64];
                snprintf(dev_cmd, sizeof(dev_cmd),
                         "AT+MSSLCERTWR=\"client.cer\",0,%d\r\n",
                         (int)(sizeof(client_cert_data) - 1));
                DEBUG_4G_PRINTF(" >>> [9] Write client cert step1: %s", dev_cmd);
                at_flush_rx();
                at_send_raw((const uint8_t *)dev_cmd, (uint16_t)strlen(dev_cmd));
                s_cert_wait_start = SysTick_GetTick();
                s_cert_step = CERT_STEP_DATA;
            }
            else if (s_cert_step == CERT_STEP_DATA)
            {
                if (at_got_prompt())
                {
                    DEBUG_4G_PRINTF(" OK - Got > for client cert, sending data...\r\n");
                    send_cert_data(client_cert_data);
                    s_cert_wait_start = SysTick_GetTick();
                    s_cert_step = CERT_STEP_WAIT_OK;
                }
                else if ((SysTick_GetTick() - s_cert_wait_start) > 8000)
                {
                    DEBUG_4G_PRINTF(" !!! Client cert: no > in 8s, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
            }
            else if (s_cert_step == CERT_STEP_WAIT_OK)
            {
                int r = at_check_last_result();
                if (r == 1)
                {
                    DEBUG_4G_PRINTF(" OK - Client cert written!\r\n");
                    s_cert_step = CERT_STEP_CMD;
                    // 进入 HTTPS 凭证检查
                    s_ml_sub_state = ML_SUB_HTTPS_CHECK;
                }
                else if (r == -1)
                {
                    DEBUG_4G_PRINTF(" !!! Client cert write ERROR, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
                else if ((SysTick_GetTick() - s_cert_wait_start) > 30000)
                {
                    DEBUG_4G_PRINTF(" !!! Client cert: no OK in 30s, retry\r\n");
                    s_cert_step = CERT_STEP_CMD;
                }
            }
            break;

        // ==================== 阶段4: HTTPS预配置（获取 device_id / device_key） ====================

        case ML_SUB_HTTPS_CHECK:
        {
            // 检查 EEPROM 中是否已有有效凭证（已在启动时加载）
            if (g_device_reg_state == DEVICE_REG_SUCCESS) {
                const device_credentials_t *cred = &g_device_cred;
                DEBUG_4G_PRINTF(" OK - Already registered, skip HTTPS\r\n");
                DEBUG_4G_PRINTF("     device_id=%s\r\n", cred->device_id);
                s_ml_sub_state = ML_SUB_SSL_AUTH;
            } else {
                DEBUG_4G_PRINTF(" >>> [10] Need registration, start HTTPS...\r\n");
                s_https_reg_done = 0;
                s_cert_step = CERT_STEP_CMD;
                s_ml_sub_state = ML_SUB_HTTPS_SSL_CFG;
            }
            break;
        }

        case ML_SUB_HTTPS_SSL_CFG:
            // AT+MSSLCFG="cert",0,"ca.cer" - 绑定CA证书（单向认证用于HTTPS注册）
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break; // 还在等待，不做任何事
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" >>> SSL cert config\r\n");
            at_command_start("AT+MSSLCFG=\"cert\",0,\"ca.cer\"", 3000);
            s_ml_sub_state = ML_SUB_HTTPS_CREATE;
            break;

        case ML_SUB_HTTPS_CREATE:
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK)
                break;
            DEBUG_4G_PRINTF(" OK - SSL cert OK\r\n");
            DEBUG_4G_PRINTF(" >>> Create HTTP instance\r\n");
            at_command_start("AT+MHTTPCREATE=\"https://api.cn.dream-maker.com:8443\"", 5000);
            s_ml_sub_state = ML_SUB_HTTPS_HEADER;
            break;

        case ML_SUB_HTTPS_HEADER:
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK)
                break;
            DEBUG_4G_PRINTF(" OK - HTTP instance created\r\n");
            DEBUG_4G_PRINTF(" >>> Config SSL and header\r\n");
            // 绑定SSL到HTTP实例
            at_command_start("AT+MHTTPCFG=\"ssl\",0,1,0", 3000);
            s_ml_sub_state = ML_SUB_HTTPS_CONTENT;
            break;

        case ML_SUB_HTTPS_CONTENT:
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            // 设置请求头
            DEBUG_4G_PRINTF(" OK - SSL bound\r\n");
            DEBUG_4G_PRINTF(" >>> Set header\r\n");
            at_command_start("AT+MHTTPCFG=\"header\",0,\"Content-Type: application/json\"", 3000);
            s_ml_sub_state = ML_SUB_HTTPS_CONTENT2;
            break;

        case ML_SUB_HTTPS_CONTENT2:
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            // 构造 JSON body 并发送
            DEBUG_4G_PRINTF(" OK - Header set\r\n");
            char prov_code[17];
            md5_encrypt_code(PRODUCT_SECRET, PRODUCT_SN, prov_code);
            snprintf(s_json_body, sizeof(s_json_body),
                     "{\"product_id\":\"%s\",\"sn\":\"%s\",\"prov_code\":\"%s\",\"mark\":\"++++++\"}",
                     PRODUCT_ID, PRODUCT_SN, prov_code);
            s_https_body_len = strlen(s_json_body);
            DEBUG_4G_PRINTF(" >>> Send JSON body (%u bytes)\r\n", (unsigned int)s_https_body_len);
            // 先发命令头，等待 >
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "AT+MHTTPCONTENT=0,0,%u", (unsigned int)s_https_body_len);
            at_flush_rx();
            at_send_raw((const uint8_t *)cmd, (uint16_t)strlen(cmd));
            at_send_raw((const uint8_t *)"\r\n", 2);
            s_cert_wait_start = SysTick_GetTick();
            s_ml_sub_state = ML_SUB_HTTPS_CONTENT3;
            break;

        case ML_SUB_HTTPS_CONTENT3:
            // 等待 > 提示符，同时处理 RX 数据
            at_command_check(); // 处理 RX 以触发 URC 回调
            if (at_got_prompt()) {
                DEBUG_4G_PRINTF(" OK - Got >, sending JSON...\r\n");
                // 直接发送 JSON body（不加 \r\n）
                at_send_raw((const uint8_t *)s_json_body, (uint16_t)s_https_body_len);
                s_ml_sub_state = ML_SUB_HTTPS_REQUEST;
            } else if ((SysTick_GetTick() - s_cert_wait_start) > 5000) {
                DEBUG_4G_PRINTF(" !!! MHTTPCONTENT: no > in 5s\r\n");
                s_ml_sub_state = ML_SUB_HTTPS_CLEANUP;
            }
            break;

        case ML_SUB_HTTPS_REQUEST:
        {
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - JSON sent, sending HTTP POST...\r\n");
            at_command_start("AT+MHTTPREQUEST=0,2,0,\"/APIServerV2/tool/mqtt/preset\"", 10000);
            s_cert_wait_start = SysTick_GetTick();
            s_ml_sub_state = ML_SUB_HTTPS_WAIT_URC;
            break;
        }

        case ML_SUB_HTTPS_WAIT_URC:
        {
            // 持续处理 RX 数据以触发 URC 回调
            at_command_check();
            if (s_https_reg_done) {
                // URC 回调已解析成功
                DEBUG_4G_PRINTF(" OK - Device registered via HTTPS!\r\n");
                at_command_start("AT+MHTTPDEL=0", 3000);
                s_ml_sub_state = ML_SUB_HTTPS_CLEANUP;
            } else if ((SysTick_GetTick() - s_cert_wait_start) > 15000) {
                DEBUG_4G_PRINTF(" !!! HTTPS: no URC in 15s, cleanup and retry\r\n");
                at_command_start("AT+MHTTPDEL=0", 3000);
                s_ml_sub_state = ML_SUB_HTTPS_CLEANUP;
            }
            break;
        }

        case ML_SUB_HTTPS_CLEANUP:
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break; // 还在等待，不做任何事
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - HTTP instance deleted\r\n");
            if (!s_https_reg_done) {
                // 注册尚未完成，重试 HTTPS 注册流程
                DEBUG_4G_PRINTF(" >>> HTTPS reg not done, retrying...\r\n");
                s_ml_sub_state = ML_SUB_HTTPS_SSL_CFG;
            } else {
                // 注册已完成，继续 SSL 双向认证
                s_ml_sub_state = ML_SUB_SSL_AUTH;
            }
            break;

        // ==================== 阶段5: SSL双向认证 ====================

        case ML_SUB_SSL_AUTH:
        {
            // 等待上一条命令完成（可能是 HTTPS_CLEANUP 的 MHTTPDEL）
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break; // 还在等待，不做任何事
            if (ret != AT_NB_OK && ret != AT_NB_ERR) {
                // AT_NB_WAITING 或其他，不做任何事
                break;
            }
            // AT命令完成了，发送 SSL 双向认证命令
            DEBUG_4G_PRINTF(" >>> [11] SSL auth (MSSLCFG)\r\n");
            at_command_start("AT+MSSLCFG=\"cert\",0,\"ca.cer\",\"client.cer\",\"client.key\"", 3000);
            s_ml_sub_state = ML_SUB_SSL_AUTH_WAIT;
            break;
        }

        case ML_SUB_SSL_AUTH_WAIT:
            // 等待 SSL 证书绑定命令完成
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - Certs bound\r\n");
            at_command_start("AT+MSSLCFG=\"auth\",0,2", 3000);
            s_ml_sub_state = ML_SUB_SSL_AUTH_WAIT2;
            break;

        case ML_SUB_SSL_AUTH_WAIT2:
            // 等待双向认证设置完成
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - Auth mode set\r\n");
            DEBUG_4G_PRINTF(" >>> MQTT SSL enable\r\n");
            at_command_start("AT+MQTTCFG=\"ssl\",0,1,0", 3000);
            s_ml_sub_state = ML_SUB_MQTT_CONN;
            break;

        // ==================== 阶段6: MQTT连接 ====================

        case ML_SUB_MQTT_CONN:
        {
            const device_credentials_t *cred = &g_device_cred;
            // 等待 MQTTSSL 命令完成
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - MQTT SSL enabled\r\n");
            DEBUG_4G_PRINTF(" >>> MQTT keepalive=120\r\n");
            at_command_start("AT+MQTTCFG=\"keepalive\",0,120", 3000);
            s_ml_sub_state = ML_SUB_MQTT_CONN_WAIT1;
            break;
        }

        case ML_SUB_MQTT_CONN_WAIT1:
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - Keepalive set\r\n");
            DEBUG_4G_PRINTF(" >>> MQTT clean=1\r\n");
            at_command_start("AT+MQTTCFG=\"clean\",0,1", 3000);
            s_ml_sub_state = ML_SUB_MQTT_CONN_WAIT2;
            break;

        case ML_SUB_MQTT_CONN_WAIT2:
        {
            const device_credentials_t *cred = &g_device_cred;
            ret = at_command_check();
            if (ret == AT_NB_IDLE)
                break;
            if (ret != AT_NB_OK && ret != AT_NB_ERR)
                break;
            DEBUG_4G_PRINTF(" OK - Clean session set\r\n");
            char mqtt_cmd[256];
            snprintf(mqtt_cmd, sizeof(mqtt_cmd),
                "AT+MQTTCONN=0,\"mqtt.cn.dream-maker.com\",8883,\"%s\",\"%s\",\"%s\"",
                PRODUCT_SN, cred->device_id, cred->device_key);
            DEBUG_4G_PRINTF(" >>> [12] MQTT connect\r\n");
            g_mqtt_conn_result = -1;  // 重置，等待新的连接URC
            at_command_start(mqtt_cmd, 10000);
            s_ml_sub_state = ML_SUB_MQTT_SUB;
            break;
        }

        // ==================== 阶段6b: 等待AT+MQTTCONN命令被模组接受 ====================

        case ML_SUB_MQTT_SUB:
            // 等待模组接受 AT+MQTTCONN 命令（返回OK，不代表TCP连接已建立）
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - MQTTCONN cmd accepted, waiting conn URC...\r\n");
                s_mqtt_urc_wait_start = SysTick_GetTick();
                s_ml_sub_state = ML_SUB_MQTT_CONN_URC_WAIT;
            } else if (ret == AT_NB_ERR) {
                int err_code = at_get_last_error_code();
                DEBUG_4G_PRINTF(" !!! MQTT connect cmd failed (err=%d: %s)\r\n",
                    err_code, ml307r_err_str(err_code));
                s_mqtt_retry++;
                if (s_mqtt_retry >= 3) {
                    DEBUG_4G_PRINTF(" !!! MQTT retry limit, full restart\r\n");
                    s_mqtt_retry = 0;
                    s_ml_sub_state = ML_SUB_AT;
                } else {
                    DEBUG_4G_PRINTF(" >>> MQTT retry %d/3\r\n", s_mqtt_retry);
                    s_ml_sub_state = ML_SUB_MQTT_CONN;
                }
            }
            break;

        // ==================== 阶段6c: 等待+MQTTURC:"conn"确认TCP连接结果 ====================

        case ML_SUB_MQTT_CONN_URC_WAIT:
        {
            // 持续处理RX数据以触发mqtt_urc_handler更新 g_mqtt_conn_result
            at_command_check();

            if (g_mqtt_conn_result == 0) {
                // 连接成功，构造topic并发起订阅
                const device_credentials_t *cred = &g_device_cred;
                snprintf(s_topic_up,   sizeof(s_topic_up),   "up/%s/%s",   PRODUCT_ID, cred->device_id);
                snprintf(s_topic_down, sizeof(s_topic_down), "down/%s/%s", PRODUCT_ID, cred->device_id);
                s_mqtt_state = MQTT_STATE_CONNECTED;
                char sub_cmd[128];
                snprintf(sub_cmd, sizeof(sub_cmd),
                    "AT+MQTTSUB=0,\"%s\",0", s_topic_down);
                DEBUG_4G_PRINTF(" >>> [13] MQTT connected, subscribe: %s\r\n", sub_cmd);
                s_mqtt_retry = 0;  // 连接成功，重置重试计数
                at_command_start(sub_cmd, 5000);
                s_ml_sub_state = ML_SUB_MQTT_SUB_WAIT;
            } else if (g_mqtt_conn_result > 0) {
                // 连接失败（服务器拒绝、网络错误等）
                DEBUG_4G_PRINTF(" !!! MQTT conn URC: result=%d (%s)\r\n",
                    g_mqtt_conn_result, mqtt_conn_result_str(g_mqtt_conn_result));
                s_mqtt_retry++;
                if (s_mqtt_retry >= 3) {
                    DEBUG_4G_PRINTF(" !!! MQTT retry limit, full restart\r\n");
                    s_mqtt_retry = 0;
                    s_ml_sub_state = ML_SUB_AT;
                } else {
                    DEBUG_4G_PRINTF(" >>> MQTT retry %d/3\r\n", s_mqtt_retry);
                    s_ml_sub_state = ML_SUB_MQTT_CONN;
                }
            } else if ((SysTick_GetTick() - s_mqtt_urc_wait_start) > 15000) {
                // 15s 内未收到连接URC
                DEBUG_4G_PRINTF(" !!! MQTT conn URC timeout\r\n");
                s_mqtt_retry++;
                if (s_mqtt_retry >= 3) {
                    DEBUG_4G_PRINTF(" !!! MQTT retry limit, full restart\r\n");
                    s_mqtt_retry = 0;
                    s_ml_sub_state = ML_SUB_AT;
                } else {
                    DEBUG_4G_PRINTF(" >>> MQTT retry %d/3\r\n", s_mqtt_retry);
                    s_ml_sub_state = ML_SUB_MQTT_CONN;
                }
            }
            break;
        }

        // ==================== 阶段7: MQTT订阅 ====================

        case ML_SUB_MQTT_SUB_WAIT:
            ret = at_command_check();
            if (ret == AT_NB_OK) {
                DEBUG_4G_PRINTF(" OK - MQTT subscribed\r\n");
                s_ml_sub_state = ML_SUB_DONE;
            } else if (ret == AT_NB_ERR) {
                int err_code = at_get_last_error_code();
                DEBUG_4G_PRINTF(" !!! MQTT subscribe failed (err=%d: %s)\r\n",
                    err_code, ml307r_err_str(err_code));
                s_mqtt_retry++;
                if (s_mqtt_retry >= 3) {
                    DEBUG_4G_PRINTF(" !!! MQTT retry limit, full restart\r\n");
                    s_mqtt_retry = 0;
                    s_ml_sub_state = ML_SUB_AT;
                } else {
                    DEBUG_4G_PRINTF(" >>> MQTT retry %d/3, reconnecting\r\n", s_mqtt_retry);
                    s_ml_sub_state = ML_SUB_MQTT_CONN;
                }
            }
            break;

        case ML_SUB_DONE:
            DEBUG_4G_PRINTF(" >>> ML307R ALL INIT COMPLETE!\r\n");
            s_init_done = 1;
            break;

        default:
            s_ml_sub_state = ML_SUB_AT;
            break;
        }
        return;
    }

    // ========== 正常运行中：检测断连并自动重连 ==========
    at_command_check(); // 持续处理RX，触发URC回调

    if (g_mqtt_disc_code >= 0) {
        // 收到断连URC
        DEBUG_4G_PRINTF("[ML307R] !!! MQTT disconnected (disc_code=%d), reconnecting...\r\n",
                        g_mqtt_disc_code);
        g_mqtt_disc_code = -1;      // 清除标志
        s_mqtt_state = MQTT_STATE_DISCONNECTED;
        s_mqtt_retry = 0;
        s_init_done = 0;
        s_ml_sub_state = ML_SUB_MQTT_CONN; // 直接重连，跳过证书/注册
    }
}
