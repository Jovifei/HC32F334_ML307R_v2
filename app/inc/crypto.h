#ifndef CRYPTO_H
#define CRYPTO_H

#include <stdint.h>
#include <stddef.h>

// MD5 上下文结构
typedef struct {
    uint32_t state[4];
    uint32_t count[2];
    uint8_t buffer[64];
} md5_context_t;

/**
 * @brief 初始化 MD5 上下文
 * @param ctx MD5 上下文指针
 */
void md5_init(md5_context_t *ctx);

/**
 * @brief 更新 MD5 数据
 * @param ctx MD5 上下文指针
 * @param input 输入数据
 * @param ilen 输入数据长度
 */
void md5_update(md5_context_t *ctx, const uint8_t *input, size_t ilen);

/**
 * @brief 完成 MD5 计算
 * @param ctx MD5 上下文指针
 * @param output 输出缓冲区（16字节）
 */
void md5_final(md5_context_t *ctx, uint8_t output[16]);

/**
 * @brief 计算字符串的 MD5 哈希
 * @param input 输入字符串
 * @param ilen 输入长度
 * @param output 输出缓冲区（16字节）
 */
void md5_hash(const uint8_t *input, size_t ilen, uint8_t output[16]);

/**
 * @brief 生成设备注册的加密码（参考 ESP32 api.c）
 * @param product_secret 产品密钥
 * @param device_sn 设备序列号
 * @param code_out 输出缓冲区（至少17字节，16字符+'\0'）
 * @note 计算 MD5(product_secret + device_sn)，取中间8字节转为16字符十六进制
 */
void md5_encrypt_code(const char *product_secret, const char *device_sn, char *code_out);

#endif // CRYPTO_H
