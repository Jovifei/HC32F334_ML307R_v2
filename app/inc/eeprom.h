/*lic*/
#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdint.h>
#include <stdbool.h>

// EEPROMҳд����ض���
#define E2P_DEVICE_BASE_ADDR 0 // �豸��Ϣ����ַ
#define E2P_PAGE_LEN 16        // EEPROMҳд�볤��
// EEPROM��ַ����
#define EEPROM_DEVICE_RECORD_SIZE 48          // ÿ����¼��С
#define EEPROM_MAX_DEVICES INV_DEVICE_MAX_NUM // ����豸��
#define BSP_I2C_BAUDRATE (100000UL)
#define BSP_I2C_TIMEOUT (0x40000U)

// �û�����б�EEPROM��ַ����
#define EEPROM_DEVICE_AREA_SIZE (EEPROM_DEVICE_RECORD_SIZE * EEPROM_MAX_DEVICES) // 384�ֽ� (8��48)
#define EEPROM_USER_PAIR_BASE_ADDR (EEPROM_DEVICE_AREA_SIZE)                     // 0x180 (384)���û������Ϣ����ַ��ǰ����������豸��Ϣ��
#define EEPROM_USER_PAIR_RECORD_SIZE 32                                          // ÿ���û���Լ�¼32�ֽ�
#define EEPROM_USER_PAIR_MAX_NUM USER_PAIR_LIST_MAX_NUM                          // ����û��������

// �õ���EEPROM��ַ���� (��8���豸���û�����б�֮��)
#define EEPROM_USER_PAIR_AREA_SIZE (EEPROM_USER_PAIR_RECORD_SIZE * EEPROM_USER_PAIR_MAX_NUM) // 256�ֽ� (8��32)
#define EEPROM_CONSUMPTION_BASE_ADDR (EEPROM_USER_PAIR_BASE_ADDR + EEPROM_USER_PAIR_AREA_SIZE + 16)

// CT��SN EEPROM��ַ���� (���õ���֮��)
#define EEPROM_SN_BASE_ADDR (EEPROM_CONSUMPTION_BASE_ADDR + 16) // SN�洢��ʼ��ַ (�õ�����16�ֽ�)
#define EEPROM_SN_SIZE (16)                                     // SN�洢�ռ��С(16�ֽ�)

// �õ��������洢EEPROM��ַ���� (��SN֮�󣬵���һҳ������ĥ��)
#define EEPROM_ELEC_BASE_ADDR (EEPROM_SN_BASE_ADDR + EEPROM_SN_SIZE) // SN��֮��16�ֽ�

// EEPROM I2C�豸��ַ
#define EEPROM_I2C_ADDR 0x50

// EEPROM�豸��¼��Ч�Ա�־
#define EEPROM_RECORD_VALID 0x55
#define EEPROM_RECORD_INVALID 0x00

// CRC8��ض���
#define CRC8_POLYNOMIAL 0x07 // CRC-8����ʽ
#define CRC8_INIT_VALUE 0x00 // CRC-8��ʼֵ

// EEPROM�豸��¼�ṹ�壨48�ֽڣ�
// �豸������б�
typedef struct
{
    char device_sn[SN_LENGTH + 1];                 // �豸���к� (16�ֽڣ�15+������)
    uint32_t sub1g_addr;                           // Sub1G��ַ (4�ֽ�)
    uint8_t siid;                                  // ����ʵ��ID 4-11 (1�ֽ�)
    uint8_t valid;                                 // ��¼��Ч��־ 0x55=��Ч, 0x00=��Ч (1�ֽ�)
    char product_model[PRODUCT_MODEL_MAX_LEN + 1]; // ��Ʒ�ͺ��ַ��� (11�ֽڣ���ǰPRODUCT_MODEL_MAX_LEN=10)
    uint8_t phase;                                 // ΢��������λ (1�ֽ�, 0=δʶ��, 1=A��, 2=B��, 3=C��)
    uint8_t reserved[13];                          // Ԥ���ֽ� (13�ֽڣ�Ϊ������չԤ��)
    uint8_t crc8;                                  // CRC8У�� (1�ֽ�)
} eeprom_device_record_t;                          // �ܹ�: 16+4+1+1+11+1+13+1 = 48�ֽ�

// �û�����б�
typedef struct __attribute__((packed))
{
    char device_sn[SN_LENGTH + 1]; // ���к��ַ������� '\0' ������������16�ֽ�
    uint8_t valid;                 // ��Ч��־��0x55=��Ч��0x00=��Ч��
    uint8_t reserved[14];          // Ԥ���ֽڣ��������
    uint8_t crc8;                  // CRC8 У��
} eeprom_user_pair_record_t;       // ��32�ֽ�

// �õ���EEPROM��¼�ṹ�壨16�ֽڣ�
typedef struct __attribute__((packed))
{
    uint32_t electricity_consumption; // �õ�����(Wh) (4�ֽ�)����ʹ��
    uint8_t valid;                    // ��Ч��־��0x55=��Ч��0x00=��Ч��(1�ֽ�)
    uint16_t to_grid_power_limit;     // �ϴ��������Ĺ������� (2�ֽ�)
    uint8_t power_work_mode;          // ���ʹ���ģʽ (1�ֽ�)
    uint8_t antiflow_enable;          // ������ʹ�� (1�ֽ�)
    uint8_t sequence_k;               // CT�������� (1�ֽ�, 0=δ����, 1-6=�������)
    uint8_t channel_index;            // sub1g�ŵ����� (1�ֽ�, 0-9=��Ч�ŵ�, ����=��Ч)
    int8_t ct1_dir;                   // CT1���ʷ��� (1�ֽ�, 1=����, -1=����)
    int8_t ct2_dir;                   // CT2���ʷ��� (1�ֽ�, 1=����, -1=����)
    int8_t ct3_dir;                   // CT3���ʷ��� (1�ֽ�, 1=����, -1=����)
    uint8_t sequence_k_tag;           // sequence_k����У���ǩ (ֵ=sequence_k*10+sequence_k, ����OTA��������)
    uint8_t crc8;                     // CRC8У�� (1�ֽ�)
} eeprom_consumption_record_t;        // ��16�ֽ�

// CT��SN��16�ֽڣ�
typedef struct __attribute__((packed))
{
    char device_sn[15]; // �豸���к� (15�ֽ�)
    uint8_t valid;      // ��Ч��־��0x55=��Ч��0x00=��Ч��(1�ֽ�)
} eeprom_sn_record_t;   // ��16�ֽڣ�����EEPROMҳд��

// �õ���������¼�ṹ�壨16�ֽڵ���һҳ�洢��Ƶ��д�����������������룩
typedef struct __attribute__((packed))
{
    uint32_t electricity_consumption; // �õ�����(Wh) (4�ֽ�)
    uint8_t valid;                    // ��Ч��־��0x55=��Ч��0x00=��Ч��(1�ֽ�)
    uint8_t reserved[10];             // Ԥ���ֽ� (10�ֽ�)
    uint8_t crc8;                     // CRC8У�� (1�ֽ�)
} eeprom_elec_record_t;               // ��16�ֽ�

uint8_t calculate_crc8(const uint8_t *data, uint16_t len);
bool verify_record_crc8(const eeprom_device_record_t *record);
int eeprom_init_and_load_devices(void);
int eeprom_remove_device_by_sub1g_addr(uint32_t sub1g_addr);
int eeprom_remove_device_by_siid(uint8_t siid);
int eeprom_clear_all_devices(void);
uint8_t eeprom_find_siid_by_addr(uint32_t sub1g_addr);
uint8_t eeprom_get_device_count(void);
uint8_t eeprom_add_device(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code);
uint8_t eeprom_add_device_by_sn_only(const char *device_sn);                                                 // ֻ����SNԤ�����豸
int eeprom_update_device_sub1g_addr(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code); // �����豸��sub1g��ַ
int8_t eeprom_find_inv_index_by_sn(const char *device_sn);                                                   // ����SN�����豸����
int eeprom_update_device_phase(uint32_t sub1g_addr, uint8_t phase);

// �û�����б�EEPROM��������

int eeprom_load_user_pair_list(void);
int eeprom_clear_user_pair_list(void);
int eeprom_write_user_pair_record(uint8_t index, const eeprom_user_pair_record_t *record);
int eeprom_read_user_pair_record(uint8_t index, eeprom_user_pair_record_t *record);
int eeprom_save_user_pair_device_by_sn(const char *device_sn);
int eeprom_clear_user_pair_list_device_by_sn(const char *device_sn); // ɾ�������豸

// �õ���EEPROM��������
int eeprom_load_set_param(void);           // ��EEPROM�м������ò���
int eeprom_save_set_param(void);           // �������ò�����EEPROM
int eeprom_save_elec_consumption(void);    // ���������õ���������ҳ
int eeprom_load_elec_consumption(void);    // ���������õ����Ӷ���ҳ
int eeprom_migrate_elec_consumption(void); // OTA�������ݣ��Ӿ�����Ǩ���õ�����������

// CT��SN��д����
int eeprom_write_sn(const char *sn); // д��SN��EEPROM
int eeprom_read_sn(char *sn);        // ��EEPROM��ȡSN

int32_t BSP_I2C_Write(CM_I2C_TypeDef *I2Cx, uint16_t u16DevAddr, const uint8_t *pu8Reg, uint8_t u8RegLen, const uint8_t *pu8Buf, uint32_t u32Len);
int32_t BSP_I2C_Read(CM_I2C_TypeDef *I2Cx, uint16_t u16DevAddr, const uint8_t *pu8Reg, uint8_t u8RegLen, uint8_t *pu8Buf, uint32_t u32Len);

// ͨ�þ���HT24LC08��ȷ��ַ��д/�ȡ
int32_t eeprom_write_block(uint16_t addr, const uint8_t *data, uint16_t len);
int32_t eeprom_read_block(uint16_t addr, uint8_t *data, uint16_t len);

// �����Լ���ӡ������豸ʹ��
void print_device_list(void);
// void run_eeprom_tests(void);

// ��Ʒ�ͺ�ת������
const char *product_model_code_to_string(uint8_t code);
uint8_t product_model_string_to_code(const char *model_str);

extern void update_ct_to_phase_mapping(uint8_t sequence_k);

#endif /* __EEPROM_H__ */

/*eof*/
