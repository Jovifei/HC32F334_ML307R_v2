/*lic*/
#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdint.h>
#include <stdbool.h>

// EEPROM页写入相关定义
#define E2P_DEVICE_BASE_ADDR 0 // 设备信息基地址
#define E2P_PAGE_LEN 16        // EEPROM页写入长度
// EEPROM地址定义
#define EEPROM_DEVICE_RECORD_SIZE 48          // 每条记录大小
#define EEPROM_MAX_DEVICES INV_DEVICE_MAX_NUM // 最大设备数
#define BSP_I2C_BAUDRATE (100000UL)
#define BSP_I2C_TIMEOUT (0x40000U)

// 用户配对列表EEPROM地址定义
#define EEPROM_DEVICE_AREA_SIZE (EEPROM_DEVICE_RECORD_SIZE * EEPROM_MAX_DEVICES) // 384字节 (8×48)
#define EEPROM_USER_PAIR_BASE_ADDR (EEPROM_DEVICE_AREA_SIZE)                     // 0x180 (384)，用户配对信息基地址，前面用来配对设备信息了
#define EEPROM_USER_PAIR_RECORD_SIZE 32                                          // 每条用户配对记录32字节
#define EEPROM_USER_PAIR_MAX_NUM USER_PAIR_LIST_MAX_NUM                          // 最大用户配对数量

// 用电量EEPROM地址定义 (在8个设备和用户配对列表之后)
#define EEPROM_USER_PAIR_AREA_SIZE (EEPROM_USER_PAIR_RECORD_SIZE * EEPROM_USER_PAIR_MAX_NUM) // 256字节 (8×32)
#define EEPROM_CONSUMPTION_BASE_ADDR (EEPROM_USER_PAIR_BASE_ADDR + EEPROM_USER_PAIR_AREA_SIZE + 16)

// CT板SN EEPROM地址定义 (在用电量之后)
#define EEPROM_SN_BASE_ADDR (EEPROM_CONSUMPTION_BASE_ADDR + 16) // SN存储起始地址 (用电量后16字节)
#define EEPROM_SN_SIZE (16)                                     // SN存储空间大小(16字节)

// 用电量独立存储EEPROM地址定义 (在SN之后，单独一页，减少磨损)
#define EEPROM_ELEC_BASE_ADDR (EEPROM_SN_BASE_ADDR + EEPROM_SN_SIZE) // SN区之后16字节

// EEPROM I2C设备地址
#define EEPROM_I2C_ADDR 0x50

// EEPROM设备记录有效性标志
#define EEPROM_RECORD_VALID 0x55
#define EEPROM_RECORD_INVALID 0x00

// CRC8相关定义
#define CRC8_POLYNOMIAL 0x07 // CRC-8多项式
#define CRC8_INIT_VALUE 0x00 // CRC-8初始值

// EEPROM设备记录结构体（48字节）
// 设备已配对列表
typedef struct
{
    char device_sn[SN_LENGTH + 1];                 // 设备序列号 (16字节，15+结束符)
    uint32_t sub1g_addr;                           // Sub1G地址 (4字节)
    uint8_t siid;                                  // 服务实例ID 4-11 (1字节)
    uint8_t valid;                                 // 记录有效标志 0x55=有效, 0x00=无效 (1字节)
    char product_model[PRODUCT_MODEL_MAX_LEN + 1]; // 产品型号字符串 (11字节，当前PRODUCT_MODEL_MAX_LEN=10)
    uint8_t phase;                                 // 微逆所在相位 (1字节, 0=未识别, 1=A相, 2=B相, 3=C相)
    uint8_t reserved[13];                          // 预留字节 (13字节，为将来扩展预留)
    uint8_t crc8;                                  // CRC8校验 (1字节)
} eeprom_device_record_t;                          // 总共: 16+4+1+1+11+1+13+1 = 48字节

// 用户配对列表
typedef struct __attribute__((packed))
{
    char device_sn[SN_LENGTH + 1]; // 序列号字符串（含 '\0' 结束符），共16字节
    uint8_t valid;                 // 有效标志（0x55=有效，0x00=无效）
    uint8_t reserved[14];          // 预留字节，用于填充
    uint8_t crc8;                  // CRC8 校验
} eeprom_user_pair_record_t;       // 共32字节

// 用电量EEPROM记录结构体（16字节）
typedef struct __attribute__((packed))
{
    uint32_t electricity_consumption; // 用电总量(Wh) (4字节)，不使用
    uint8_t valid;                    // 有效标志（0x55=有效，0x00=无效）(1字节)
    uint16_t to_grid_power_limit;     // 上传到电网的功率限制 (2字节)
    uint8_t power_work_mode;          // 功率工作模式 (1字节)
    uint8_t antiflow_enable;          // 防逆流使能 (1字节)
    uint8_t sequence_k;               // CT相序设置 (1字节, 0=未设置, 1-6=相序组合)
    uint8_t channel_index;            // sub1g信道索引 (1字节, 0-9=有效信道, 其他=无效)
    int8_t ct1_dir;                   // CT1功率方向 (1字节, 1=正向, -1=反向)
    int8_t ct2_dir;                   // CT2功率方向 (1字节, 1=正向, -1=反向)
    int8_t ct3_dir;                   // CT3功率方向 (1字节, 1=正向, -1=反向)
    uint8_t sequence_k_tag;           // sequence_k伴生校验标签 (值=sequence_k*10+sequence_k, 用于OTA升级兼容)
    uint8_t crc8;                     // CRC8校验 (1字节)
} eeprom_consumption_record_t;        // 共16字节

// CT板SN（16字节）
typedef struct __attribute__((packed))
{
    char device_sn[15]; // 设备序列号 (15字节)
    uint8_t valid;      // 有效标志（0x55=有效，0x00=无效）(1字节)
} eeprom_sn_record_t;   // 共16字节，对齐EEPROM页写入

// 用电量独立记录结构体（16字节单独一页存储，频繁写入区域与配置区隔离）
typedef struct __attribute__((packed))
{
    uint32_t electricity_consumption; // 用电总量(Wh) (4字节)
    uint8_t valid;                    // 有效标志（0x55=有效，0x00=无效）(1字节)
    uint8_t reserved[10];             // 预留字节 (10字节)
    uint8_t crc8;                     // CRC8校验 (1字节)
} eeprom_elec_record_t;               // 共16字节

uint8_t calculate_crc8(const uint8_t *data, uint16_t len);
bool verify_record_crc8(const eeprom_device_record_t *record);
int eeprom_init_and_load_devices(void);
int eeprom_remove_device_by_sub1g_addr(uint32_t sub1g_addr);
int eeprom_remove_device_by_siid(uint8_t siid);
int eeprom_clear_all_devices(void);
uint8_t eeprom_find_siid_by_addr(uint32_t sub1g_addr);
uint8_t eeprom_get_device_count(void);
uint8_t eeprom_add_device(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code);
uint8_t eeprom_add_device_by_sn_only(const char *device_sn);                                                 // 只根据SN预分配设备
int eeprom_update_device_sub1g_addr(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code); // 更新设备的sub1g地址
int8_t eeprom_find_inv_index_by_sn(const char *device_sn);                                                   // 根据SN查找设备索引
int eeprom_update_device_phase(uint32_t sub1g_addr, uint8_t phase);

// 用户配对列表EEPROM操作函数

int eeprom_load_user_pair_list(void);
int eeprom_clear_user_pair_list(void);
int eeprom_write_user_pair_record(uint8_t index, const eeprom_user_pair_record_t *record);
int eeprom_read_user_pair_record(uint8_t index, eeprom_user_pair_record_t *record);
int eeprom_save_user_pair_device_by_sn(const char *device_sn);
int eeprom_clear_user_pair_list_device_by_sn(const char *device_sn); // 删除单个设备

// 用电量EEPROM操作函数
int eeprom_load_set_param(void);           // 从EEPROM中加载配置参数
int eeprom_save_set_param(void);           // 保存配置参数到EEPROM
int eeprom_save_elec_consumption(void);    // 单独保存用电量到独立页
int eeprom_load_elec_consumption(void);    // 单独加载用电量从独立页
int eeprom_migrate_elec_consumption(void); // OTA升级兼容：从旧区域迁移用电量到新区域

// CT板SN读写函数
int eeprom_write_sn(const char *sn); // 写入SN到EEPROM
int eeprom_read_sn(char *sn);        // 从EEPROM读取SN

int32_t BSP_I2C_Write(CM_I2C_TypeDef *I2Cx, uint16_t u16DevAddr, const uint8_t *pu8Reg, uint8_t u8RegLen, const uint8_t *pu8Buf, uint32_t u32Len);
int32_t BSP_I2C_Read(CM_I2C_TypeDef *I2Cx, uint16_t u16DevAddr, const uint8_t *pu8Reg, uint8_t u8RegLen, uint8_t *pu8Buf, uint32_t u32Len);

// 测试以及打印已配对设备使用
void print_device_list(void);
// void run_eeprom_tests(void);

// 产品型号转换函数
const char *product_model_code_to_string(uint8_t code);
uint8_t product_model_string_to_code(const char *model_str);

extern void update_ct_to_phase_mapping(uint8_t sequence_k);

#endif /* __EEPROM_H__ */

/*eof*/
