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
 * @brief 初始化设备注册模块
 */
void device_register_init(void);

/**
 * @brief 设置设备信息（从配置或MCU获取）
 * @param product_id 产品ID
 * @param product_secret 产品密钥
 * @param product_model 产品型号
 * @param device_sn 设备序列号
 */
void device_register_set_info(const char *product_id, const char *product_secret,
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
 * @brief 获取注册状态
 * @return device_reg_state_t
 */
device_reg_state_t device_register_get_state(void);

/**
 * @brief 获取设备凭据
 * @return device_credentials_t 指针
 */
const device_credentials_t* device_register_get_credentials(void);

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
