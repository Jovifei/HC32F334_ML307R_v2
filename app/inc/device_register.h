#ifndef DEVICE_REGISTER_H
#define DEVICE_REGISTER_H

#include <stdint.h>
#include <stdbool.h>

// 设备注册状态
typedef enum {
    DEVICE_REG_IDLE = 0,        // 空闲
    DEVICE_REG_REQUESTING,      // 正在请求
    DEVICE_REG_SUCCESS,         // 成功
    DEVICE_REG_FAILED           // 失败
} device_reg_state_t;

// 设备凭据结构
typedef struct {
    char product_id[32];        // 产品ID
    char product_secret[32];    // 产品密钥
    char product_model[32];     // 产品型号
    char device_sn[32];         // 设备序列号
    char device_id[32];         // 设备ID（注册后获取）
    char device_key[64];        // 设备密钥（注册后获取）
    uint8_t valid;              // 有效标记（0x55=已注册）
    bool registered;            // 是否已注册
} device_credentials_t;

/**
 * @brief 设备注册模块全局状态（只读使用）
 */
extern device_credentials_t g_device_cred;
extern device_reg_state_t g_device_reg_state;

/**
 * @brief 启动阶段初始化并加载已保存的凭证
 * @param product_id 产品ID
 * @param product_secret 产品密钥
 * @param product_model 产品型号
 * @param device_sn 设备序列号
 *
 * 说明：该函数会清空内部状态、写入基础信息，并尝试从EEPROM加载 device_id/device_key。
 * @param product_id 产品ID
 * @param product_secret 产品密钥
 * @param product_model 产品型号
 * @param device_sn 设备序列号
 */
void device_register_bootstrap(const char *product_id, const char *product_secret,
                               const char *product_model, const char *device_sn);

/**
 * @brief 设置设备凭证（device_id/device_key）
 * @param device_id 设备ID
 * @param device_key 设备密钥
 */
void device_register_set_credentials(const char *device_id, const char *device_key);

/**
 * @brief 发起设备注册请求（通过 ML307R HTTPS）
 * @param mark 标记字段（可选，传 NULL 或 ""）
 * @return 0=成功发起, -1=失败
 */
int device_register_request(const char *mark);

/**
 * @brief 从 Flash 加载已保存的凭据
 * @return true=成功, false=失败或未注册
 */
bool device_register_load_from_flash(void);

/**
 * @brief 保存凭据到 Flash
 * @return true=成功, false=失败
 */
bool device_register_save_to_flash(void);

#endif // DEVICE_REGISTER_H
