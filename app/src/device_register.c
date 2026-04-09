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

// ==================== �豸ƾ�� ====================

static device_credentials_t s_cred = {0};
static device_reg_state_t s_state = DEVICE_REG_IDLE;

// ƾ�ݴ洢��ַ(SN��֮��EEPROM_ELEC_BASE_ADDR+16=0x1A0)
#define CRED_ADDR  0x1A0
#define CRED_LEN   192  // 12�?

// EEPROM ƾ�ݽṹ(192�ֽ�)
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
 Input       : data - ��������ָ��
               len  - �������ݳ���(�ֽ�)
 Output      : CRC8У��ֵ
 Description :
 ����ƾ�ݽṹ��CRC8У��ֵ(����ʽ 0x07)��
 ����У�� `cred_store_t` ��EEPROM�е����������ԡ�
 ע�⣺�㷨ʵ������ `eeprom.c` �е�CRC8����һ�£�����ᵼ��У��ʧ�ܡ�
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
 Input       : addr - EEPROM��ʼ��ַ(16λ)
               data - ��ȡ������
               len  - ��ȡ����(�ֽ�)
 Output      : BSP_I2C_Read �ķ���ֵ(LL_OK=�ɹ�������=ʧ��)
 Description :
 ��EEPROM��ȡ���ⳤ�����ݡ�
 �ú�����װI2C�����̣���д��2�ֽڵ�ַ���ٶ�ȡ���ݡ�
---------------------------------------------------------------------------*/
static int cred_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint8_t reg[2];
    reg[0] = (uint8_t)(addr >> 8);
    reg[1] = (uint8_t)(addr & 0xFF);
    return BSP_I2C_Read(CM_I2C, EEPROM_I2C_ADDR, reg, 2, data, len);
}

// ==================== 对外接口 ====================

/*---------------------------------------------------------------------------
 Name        : device_register_init
 Input       : ��
 Output      : ��
 Description :
 ��ʼ���豸ע��ģ����ڲ�״̬��
 ��� `s_cred`������ע��״̬��Ϊ `DEVICE_REG_IDLE`��
---------------------------------------------------------------------------*/
void device_register_init(void)
{
    memset(&s_cred, 0, sizeof(s_cred));
    s_state = DEVICE_REG_IDLE;
}

/*---------------------------------------------------------------------------
 Name        : device_register_set_info
 Input       : product_id     - ��ƷID�ַ���
               product_secret - ��Ʒ��Կ�ַ���
               product_model  - ��Ʒ�ͺ��ַ���
               device_sn      - �豸���к��ַ���
 Output      : ��
 Description :
 �����豸ע������Ļ�����Ϣ��д�뵽ȫ��ƾ�� `s_cred`��
 ˵����
 - �����������ΪNULL��ΪNULL���Ӧ�ֶα��ֲ��䡣
 - ÿ���ֶ��ڲ�ʹ�� `strncpy(..., 31)`��ȷ������дԽ�磬���ϲ���Ӧ��֤�ַ�����\\0��β��
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
 Input       : mark - ��ע�ֶ�(��ѡ)�����ڷ����������Դ/���Σ���ΪNULL
 Output      : 0=�ɹ�, -1=ʧ��
 Description :
 �����豸ע��(���ƶ����� device_id / device_key)��
 ���̸�����
 - У�� `s_cred.product_id/product_secret/device_sn` �Ƿ�������
 - ���� prov_code = MD5(product_secret + device_sn) (�� `crypto.c/md5_encrypt_code`)
 - ͨ��ģ��AT������HTTP(S)���̣�
   1) `AT+HTTPINIT`
   2) `AT+HTTPPARA="URL",...`
   3) `AT+HTTPPARA="CONTENT","application/json"`
   4) `AT+HTTPDATA=<len>,10000` -> �ȴ� DOWNLOAD
   5) `at_send_raw()` д�� JSON body
   6) `AT+HTTPACTION=1` ����POST
   7) `AT+HTTPREAD` ��ȡ��Ӧ������ device_id/device_key
   8) `AT+HTTPTERM` ����HTTP������
 - �������ɹ������� `s_cred`���� `s_state=DEVICE_REG_SUCCESS` ��д��EEPROM����
 ʧ��������
 - AT����ʧ��/��ʱ
 - �������Ӧ�޷������� device_id/device_key
---------------------------------------------------------------------------*/
int device_register_request(const char *mark)
{
    char resp[128];
    char prov_code[17];
    char body[128];
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

    // ���� {"code":0,"data":{"device_id":"xxx","device_key":"yyy"}}
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
 Input       : ��
 Output      : device_reg_state_t - ��ǰע��״̬
 Description :
 ��ȡ�豸ע�����̵ĵ�ǰ״̬�������ϲ������ж��Ƿ���Ҫ����/�Ƿ��ѳɹ���
---------------------------------------------------------------------------*/
device_reg_state_t device_register_get_state(void)
{
    return s_state;
}

/*---------------------------------------------------------------------------
 Name        : device_register_get_credentials
 Input       : ��
 Output      : const device_credentials_t* - ƾ�ݽṹ��ָ��
 Description :
 ��ȡ��ǰ�豸ƾ��(`s_cred`)��ֻ��ָ�롣
 ע�⣺���ص����ڲ���̬�����ַ�����÷���Ҫ�޸������ݡ�
---------------------------------------------------------------------------*/
const device_credentials_t* device_register_get_credentials(void)
{
    return &s_cred;
}

/*---------------------------------------------------------------------------
 Name        : device_register_load_from_flash
 Input       : ��
 Output      : true=���سɹ�, false=����ʧ��/������Ч
 Description :
 ��EEPROM�����豸ƾ�ݵ� `s_cred`��
 У���߼���
 - ��ȡ `cred_store_t` ���ݽṹ
 - ����CRC8����洢�� `tmp.crc8` �Ƚ�
 - У�� `tmp.valid == 0x55`
 �ɹ���
 - �������ֶε� `s_cred`
 - �� `s_state` ��Ϊ `DEVICE_REG_SUCCESS`
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
 Input       : ��
 Output      : true=����ɹ�, false=����ʧ��
 Description :
 ����ǰƾ�� `s_cred` ���浽EEPROM��
 ��Ϊ��
 - ��װ `cred_store_t tmp`��д��valid��־��CRC8
 - ��16�ֽ�ҳд��(��EEPROMҳ��Сһ��)����ҳ���� `BSP_I2C_Write`
 ע�⣺
 - д���ַʹ�� `CRED_ADDR`�����÷�Ӧȷ��������δ����������ռ�á�
 - ����;д��ʧ�ܷ���false���ϲ��ѡ�����Ի����쳣��
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

    // ��ҳд(16�ֽ�/ҳ)
    int pages = (sizeof(tmp) + 15) / 16;
    for (int i = 0; i < pages; i++) {
        uint16_t page_addr = CRED_ADDR + i * 16;
        if (BSP_I2C_Write(CM_I2C, EEPROM_I2C_ADDR,
                          (uint8_t[2]){(uint8_t)(page_addr >> 8), (uint8_t)page_addr},
                          2, (uint8_t *)&tmp + i * 16, 16) != LL_OK) {
            return false;
        }
    }
    return true;
}
