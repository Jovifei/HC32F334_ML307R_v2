#include "board.h"
#include "main.h"
#include "device_register.h"
#include "crypto.h"
#include "at_parser.h"
#include "uart_at.h"
#include "config.h"
#include "eeprom.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ==================== 设备凭据 ====================

static device_credentials_t s_cred = {0};
static device_reg_state_t s_state = DEVICE_REG_IDLE;

// 凭据存储地址(SN区之后，EEPROM_ELEC_BASE_ADDR+16=0x1A0)
#define CRED_ADDR  0x1A0
#define CRED_LEN   192  // 12椤?

// EEPROM 凭据结构(192字节)
typedef struct __attribute__((packed)) {
    char product_id[32];
    char product_secret[32];
    char product_model[32];
    char device_sn[32];
    char device_id[32];
    char device_key[64];
    uint8_t valid;
    uint8_t crc8;
} cred_store_t;

/*---------------------------------------------------------------------------
 Name        : cred_crc8
 Input       : data - 输入数据指针
               len  - 输入数据长度(字节)
 Output      : CRC8校验值
 Description :
 计算凭据结构的CRC8校验值(多项式 0x07)。
 用于校验 `cred_store_t` 在EEPROM中的数据完整性。
 注意：算法实现需与 `eeprom.c` 中的CRC8保持一致，否则会导致校验失败。
---------------------------------------------------------------------------*/
static uint8_t cred_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

/*---------------------------------------------------------------------------
 Name        : cred_read
 Input       : addr - EEPROM起始地址(16位)
               data - 读取缓冲区
               len  - 读取长度(字节)
 Output      : BSP_I2C_Read 的返回值(LL_OK=成功，其它=失败)
 Description :
 从EEPROM读取任意长度数据。
 该函数封装I2C读流程：先写入2字节地址，再读取数据。
---------------------------------------------------------------------------*/
static int cred_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint8_t reg[2];
    reg[0] = (uint8_t)(addr >> 8);
    reg[1] = (uint8_t)(addr & 0xFF);
    return BSP_I2C_Read(CM_I2C, EEPROM_I2C_ADDR, reg, 2, data, len);
}

// ==================== 瀵瑰鎺ュ彛 ====================

/*---------------------------------------------------------------------------
 Name        : device_register_init
 Input       : 无
 Output      : 无
 Description :
 初始化设备注册模块的内部状态。
 清空 `s_cred`，并将注册状态置为 `DEVICE_REG_IDLE`。
---------------------------------------------------------------------------*/
void device_register_init(void)
{
    memset(&s_cred, 0, sizeof(s_cred));
    s_state = DEVICE_REG_IDLE;
}

/*---------------------------------------------------------------------------
 Name        : device_register_set_info
 Input       : product_id     - 产品ID字符串
               product_secret - 产品密钥字符串
               product_model  - 产品型号字符串
               device_sn      - 设备序列号字符串
 Output      : 无
 Description :
 设置设备注册所需的基础信息，写入到全局凭据 `s_cred`。
 说明：
 - 传入参数允许为NULL，为NULL则对应字段保持不变。
 - 每个字段内部使用 `strncpy(..., 31)`，确保不会写越界，但上层仍应保证字符串以\\0结尾。
---------------------------------------------------------------------------*/
void device_register_set_info(const char *product_id, const char *product_secret,
                               const char *product_model, const char *device_sn)
{
    if (product_id)     strncpy(s_cred.product_id,     product_id,     31);
    if (product_secret) strncpy(s_cred.product_secret, product_secret, 31);
    if (product_model)  strncpy(s_cred.product_model,  product_model,  31);
    if (device_sn)      strncpy(s_cred.device_sn,      device_sn,      31);
}

/*---------------------------------------------------------------------------
 Name        : device_register_request
 Input       : mark - 备注字段(可选)，用于服务端区分来源/批次；可为NULL
 Output      : 0=成功, -1=失败
 Description :
 发起设备注册(向云端申请 device_id / device_key)。
 流程概述：
 - 校验 `s_cred.product_id/product_secret/device_sn` 是否已设置
 - 计算 prov_code = MD5(product_secret + device_sn) (见 `crypto.c/md5_encrypt_code`)
 - 通过模块AT命令走HTTP(S)流程：
   1) `AT+HTTPINIT`
   2) `AT+HTTPPARA="URL",...`
   3) `AT+HTTPPARA="CONTENT","application/json"`
   4) `AT+HTTPDATA=<len>,10000` -> 等待 DOWNLOAD
   5) `at_send_raw()` 写入 JSON body
   6) `AT+HTTPACTION=1` 发起POST
   7) `AT+HTTPREAD` 读取响应并解析 device_id/device_key
   8) `AT+HTTPTERM` 清理HTTP上下文
 - 若解析成功，更新 `s_cred`、置 `s_state=DEVICE_REG_SUCCESS` 并写入EEPROM保存
 失败条件：
 - AT命令失败/超时
 - 服务端响应无法解析出 device_id/device_key
---------------------------------------------------------------------------*/
int device_register_request(const char *mark)
{
    char resp[256];
    char prov_code[17];
    char body[256];
    int ret;

    if (strlen(s_cred.product_id) == 0 ||
        strlen(s_cred.product_secret) == 0 ||
        strlen(s_cred.device_sn) == 0) {
        DEBUG_4G_PRINTF("Device info incomplete\n");
        return -1;
    }

    s_state = DEVICE_REG_REQUESTING;
    DEBUG_4G_PRINTF("Device registration start\n");
    DEBUG_4G_PRINTF("Product ID: %s\n", s_cred.product_id);
    DEBUG_4G_PRINTF("Device SN: %s\n", s_cred.device_sn);

    // prov_code = MD5(product_secret + device_sn)
    md5_encrypt_code(s_cred.product_secret, s_cred.device_sn, prov_code);
    DEBUG_4G_PRINTF("Prov code: %s\n", prov_code);

    snprintf(body, sizeof(body),
             "{\"product_id\":\"%s\",\"sn\":\"%s\",\"prov_code\":\"%s\",\"mark\":\"%s\"}",
             s_cred.product_id, s_cred.device_sn, prov_code, mark ? mark : "");

    // HTTPS POST
    char *id_start = NULL;
    char *key_start = NULL;

    ret = at_send_command("AT+HTTPINIT", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) {
        DEBUG_4G_PRINTF("HTTPINIT failed\n");
        s_state = DEVICE_REG_FAILED;
        return -1;
    }

    char url[200];
    snprintf(url, sizeof(url),
             "AT+HTTPPARA=\"URL\",\"https://api.dream-maker.com:8443/APIServerV2/tool/mqtt/preset\"");
    ret = at_send_command(url, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) goto cleanup;

    ret = at_send_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK",
                          AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) goto cleanup;

    char data_cmd[32];
    snprintf(data_cmd, sizeof(data_cmd), "AT+HTTPDATA=%d,10000", (int)strlen(body));
    ret = at_send_command(data_cmd, "DOWNLOAD", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) goto cleanup;

    at_send_raw((uint8_t*)body, strlen(body));
    delay_ms(500);

    ret = at_send_command("AT+HTTPACTION=1", "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    if (ret != 0) goto cleanup;

    delay_ms(3000);

    ret = at_send_command("AT+HTTPREAD", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) goto cleanup;

    DEBUG_4G_PRINTF("HTTP response: %s\n", resp);

    // 解析 {"code":0,"data":{"device_id":"xxx","device_key":"yyy"}}
    id_start = strstr(resp, "\"device_id\":\"");
    key_start = strstr(resp, "\"device_key\":\"");

    if (id_start && key_start) {
        id_start += 13;
        char *id_end = strchr(id_start, '"');
        if (id_end) {
            int len = id_end - id_start;
            strncpy(s_cred.device_id, id_start, (len < 32) ? len : 31);
        }

        key_start += 13;
        char *key_end = strchr(key_start, '"');
        if (key_end) {
            int len = key_end - key_start;
            strncpy(s_cred.device_key, key_start, (len < 64) ? len : 63);
        }

        s_cred.valid = 0x55;
        s_state = DEVICE_REG_SUCCESS;
        DEBUG_4G_PRINTF("Device registered successfully\n");
        DEBUG_4G_PRINTF("Device ID: %s\n", s_cred.device_id);
        DEBUG_4G_PRINTF("Device Key: %s\n", s_cred.device_key);
        device_register_save_to_flash();
    } else {
        DEBUG_4G_PRINTF("Failed to parse device credentials\n");
        s_state = DEVICE_REG_FAILED;
    }

cleanup:
    at_send_command("AT+HTTPTERM", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    return (s_state == DEVICE_REG_SUCCESS) ? 0 : -1;
}

/*---------------------------------------------------------------------------
 Name        : device_register_get_state
 Input       : 无
 Output      : device_reg_state_t - 当前注册状态
 Description :
 获取设备注册流程的当前状态，用于上层任务判断是否需要重试/是否已成功。
---------------------------------------------------------------------------*/
device_reg_state_t device_register_get_state(void)
{
    return s_state;
}

/*---------------------------------------------------------------------------
 Name        : device_register_get_credentials
 Input       : 无
 Output      : const device_credentials_t* - 凭据结构体指针
 Description :
 获取当前设备凭据(`s_cred`)的只读指针。
 注意：返回的是内部静态对象地址，调用方不要修改其内容。
---------------------------------------------------------------------------*/
const device_credentials_t* device_register_get_credentials(void)
{
    return &s_cred;
}

/*---------------------------------------------------------------------------
 Name        : device_register_load_from_flash
 Input       : 无
 Output      : true=加载成功, false=加载失败/数据无效
 Description :
 从EEPROM加载设备凭据到 `s_cred`。
 校验逻辑：
 - 读取 `cred_store_t` 数据结构
 - 计算CRC8并与存储的 `tmp.crc8` 比较
 - 校验 `tmp.valid == 0x55`
 成功后：
 - 拷贝各字段到 `s_cred`
 - 将 `s_state` 置为 `DEVICE_REG_SUCCESS`
---------------------------------------------------------------------------*/
bool device_register_load_from_flash(void)
{
    cred_store_t tmp;
    int ret = cred_read(CRED_ADDR, (uint8_t*)&tmp, sizeof(tmp));
    if (ret != LL_OK) {
        DEBUG_4G_PRINTF("EEPROM read failed\n");
        return false;
    }

    uint8_t crc = cred_crc8((uint8_t*)&tmp, sizeof(tmp) - 1);
    if (crc != tmp.crc8 || tmp.valid != 0x55) {
        DEBUG_4G_PRINTF("Credentials invalid (CRC or valid flag)\n");
        return false;
    }

    memcpy(s_cred.product_id,     tmp.product_id,     32);
    memcpy(s_cred.product_secret, tmp.product_secret, 32);
    memcpy(s_cred.product_model,  tmp.product_model,  32);
    memcpy(s_cred.device_sn,      tmp.device_sn,      32);
    memcpy(s_cred.device_id,      tmp.device_id,      32);
    memcpy(s_cred.device_key,     tmp.device_key,     64);
    s_cred.valid = tmp.valid;

    s_state = DEVICE_REG_SUCCESS;
    DEBUG_4G_PRINTF("Credentials loaded from EEPROM\n");
    DEBUG_4G_PRINTF("Device ID: %s\n", s_cred.device_id);
    return true;
}

/*---------------------------------------------------------------------------
 Name        : device_register_save_to_flash
 Input       : 无
 Output      : true=保存成功, false=保存失败
 Description :
 将当前凭据 `s_cred` 保存到EEPROM。
 行为：
 - 组装 `cred_store_t tmp`，写入valid标志与CRC8
 - 按16字节页写入(与EEPROM页大小一致)，逐页调用 `BSP_I2C_Write`
 注意：
 - 写入地址使用 `CRED_ADDR`，调用方应确保该区域未被其它数据占用。
 - 若中途写入失败返回false，上层可选择重试或标记异常。
---------------------------------------------------------------------------*/
bool device_register_save_to_flash(void)
{
    cred_store_t tmp = {0};
    strncpy(tmp.product_id,     s_cred.product_id,     31);
    strncpy(tmp.product_secret, s_cred.product_secret, 31);
    strncpy(tmp.product_model,  s_cred.product_model,  31);
    strncpy(tmp.device_sn,      s_cred.device_sn,      31);
    strncpy(tmp.device_id,      s_cred.device_id,      31);
    strncpy(tmp.device_key,     s_cred.device_key,    63);
    tmp.valid = 0x55;
    tmp.crc8 = cred_crc8((uint8_t*)&tmp, sizeof(tmp) - 1);

    // 按页写(16字节/页)
    int pages = (sizeof(tmp) + 15) / 16;
    for (int i = 0; i < pages; i++) {
        if (BSP_I2C_Write(CM_I2C, EEPROM_I2C_ADDR,
                          (uint8_t[2]){(uint8_t)(CRED_ADDR>>8), (uint8_t)CRED_ADDR},
                          2, (uint8_t*)&tmp + i*16, 16) != LL_OK) {
            return false;
        }
    }
    return true;
}
