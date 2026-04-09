#include "crypto.h"
#include <string.h>
#include <stdio.h>

// MD5 常量
static const uint8_t S[64] = {
    7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,
    5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,
    4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,
    6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21
};

static const uint32_t K[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
    0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
    0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
    0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
    0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
    0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
    0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
    0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
    0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391
};

// 辅助宏
#define F(x, y, z) (((x) & (y)) | ((~(x)) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~(z))))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~(z))))

#define ROTLEFT(a, b) (((a) << (b)) | ((a) >> (32 - (b))))

#define FF(a, b, c, d, m, s, t) { \
    (a) += F((b), (c), (d)) + (m) + (t); \
    (a) = ROTLEFT((a), (s)); \
    (a) += (b); \
}

#define GG(a, b, c, d, m, s, t) { \
    (a) += G((b), (c), (d)) + (m) + (t); \
    (a) = ROTLEFT((a), (s)); \
    (a) += (b); \
}

#define HH(a, b, c, d, m, s, t) { \
    (a) += H((b), (c), (d)) + (m) + (t); \
    (a) = ROTLEFT((a), (s)); \
    (a) += (b); \
}

#define II(a, b, c, d, m, s, t) { \
    (a) += I((b), (c), (d)) + (m) + (t); \
    (a) = ROTLEFT((a), (s)); \
    (a) += (b); \
}

/*---------------------------------------------------------------------------
 Name        : md5_transform
 Input       : ctx  - MD5上下文指针(包含state/buffer/count等状态)
               data - 64字节输入块(单个MD5分组)
 Output      : 无
 Description :
 MD5核心压缩函数(Transform)。
 对输入的64字节分组进行4轮运算，更新 `ctx->state[4]`。
 说明：
 - 该函数只处理单个分组，不负责填充/长度编码。
 - 上层 `md5_update()` 负责按64字节分组调用本函数。
 - 使用小端序将输入字节转换为16个32位字 `m[16]`。
---------------------------------------------------------------------------*/
static void md5_transform(md5_context_t *ctx, const uint8_t data[64])
{
    uint32_t a, b, c, d, m[16], i;

    // 将输入数据转换为16个32位字(小端序)
    for (i = 0; i < 16; i++) {
        m[i] = ((uint32_t)data[i * 4]) |
               ((uint32_t)data[i * 4 + 1] << 8) |
               ((uint32_t)data[i * 4 + 2] << 16) |
               ((uint32_t)data[i * 4 + 3] << 24);
    }

    a = ctx->state[0];
    b = ctx->state[1];
    c = ctx->state[2];
    d = ctx->state[3];

    // Round 1
    FF(a, b, c, d, m[0],  S[0],  K[0]);
    FF(d, a, b, c, m[1],  S[1],  K[1]);
    FF(c, d, a, b, m[2],  S[2],  K[2]);
    FF(b, c, d, a, m[3],  S[3],  K[3]);
    FF(a, b, c, d, m[4],  S[4],  K[4]);
    FF(d, a, b, c, m[5],  S[5],  K[5]);
    FF(c, d, a, b, m[6],  S[6],  K[6]);
    FF(b, c, d, a, m[7],  S[7],  K[7]);
    FF(a, b, c, d, m[8],  S[8],  K[8]);
    FF(d, a, b, c, m[9],  S[9],  K[9]);
    FF(c, d, a, b, m[10], S[10], K[10]);
    FF(b, c, d, a, m[11], S[11], K[11]);
    FF(a, b, c, d, m[12], S[12], K[12]);
    FF(d, a, b, c, m[13], S[13], K[13]);
    FF(c, d, a, b, m[14], S[14], K[14]);
    FF(b, c, d, a, m[15], S[15], K[15]);

    // Round 2
    GG(a, b, c, d, m[1],  S[16], K[16]);
    GG(d, a, b, c, m[6],  S[17], K[17]);
    GG(c, d, a, b, m[11], S[18], K[18]);
    GG(b, c, d, a, m[0],  S[19], K[19]);
    GG(a, b, c, d, m[5],  S[20], K[20]);
    GG(d, a, b, c, m[10], S[21], K[21]);
    GG(c, d, a, b, m[15], S[22], K[22]);
    GG(b, c, d, a, m[4],  S[23], K[23]);
    GG(a, b, c, d, m[9],  S[24], K[24]);
    GG(d, a, b, c, m[14], S[25], K[25]);
    GG(c, d, a, b, m[3],  S[26], K[26]);
    GG(b, c, d, a, m[8],  S[27], K[27]);
    GG(a, b, c, d, m[13], S[28], K[28]);
    GG(d, a, b, c, m[2],  S[29], K[29]);
    GG(c, d, a, b, m[7],  S[30], K[30]);
    GG(b, c, d, a, m[12], S[31], K[31]);

    // Round 3
    HH(a, b, c, d, m[5],  S[32], K[32]);
    HH(d, a, b, c, m[8],  S[33], K[33]);
    HH(c, d, a, b, m[11], S[34], K[34]);
    HH(b, c, d, a, m[14], S[35], K[35]);
    HH(a, b, c, d, m[1],  S[36], K[36]);
    HH(d, a, b, c, m[4],  S[37], K[37]);
    HH(c, d, a, b, m[7],  S[38], K[38]);
    HH(b, c, d, a, m[10], S[39], K[39]);
    HH(a, b, c, d, m[13], S[40], K[40]);
    HH(d, a, b, c, m[0],  S[41], K[41]);
    HH(c, d, a, b, m[3],  S[42], K[42]);
    HH(b, c, d, a, m[6],  S[43], K[43]);
    HH(a, b, c, d, m[9],  S[44], K[44]);
    HH(d, a, b, c, m[12], S[45], K[45]);
    HH(c, d, a, b, m[15], S[46], K[46]);
    HH(b, c, d, a, m[2],  S[47], K[47]);

    // Round 4
    II(a, b, c, d, m[0],  S[48], K[48]);
    II(d, a, b, c, m[7],  S[49], K[49]);
    II(c, d, a, b, m[14], S[50], K[50]);
    II(b, c, d, a, m[5],  S[51], K[51]);
    II(a, b, c, d, m[12], S[52], K[52]);
    II(d, a, b, c, m[3],  S[53], K[53]);
    II(c, d, a, b, m[10], S[54], K[54]);
    II(b, c, d, a, m[1],  S[55], K[55]);
    II(a, b, c, d, m[8],  S[56], K[56]);
    II(d, a, b, c, m[15], S[57], K[57]);
    II(c, d, a, b, m[6],  S[58], K[58]);
    II(b, c, d, a, m[13], S[59], K[59]);
    II(a, b, c, d, m[4],  S[60], K[60]);
    II(d, a, b, c, m[11], S[61], K[61]);
    II(c, d, a, b, m[2],  S[62], K[62]);
    II(b, c, d, a, m[9],  S[63], K[63]);

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
}

/*---------------------------------------------------------------------------
 Name        : md5_init
 Input       : ctx - MD5上下文指针
 Output      : 无
 Description :
 初始化MD5上下文。
 将计数器清零，并设置初始链接变量(state)为MD5标准初值：
 - A=0x67452301
 - B=0xefcdab89
 - C=0x98badcfe
 - D=0x10325476
 后续需配合 `md5_update()`/`md5_final()` 完成完整计算。
---------------------------------------------------------------------------*/
void md5_init(md5_context_t *ctx)
{
    ctx->count[0] = 0;
    ctx->count[1] = 0;
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xefcdab89;
    ctx->state[2] = 0x98badcfe;
    ctx->state[3] = 0x10325476;
}

/*---------------------------------------------------------------------------
 Name        : md5_update
 Input       : ctx   - MD5上下文指针
               input - 输入数据指针
               ilen  - 输入数据长度(字节)
 Output      : 无
 Description :
 向MD5上下文追加数据并更新哈希状态。
 行为：
 - 维护 `ctx->count` 记录总输入长度
 - 将数据缓存到 `ctx->buffer`，凑满64字节后调用 `md5_transform()`
 - 支持任意长度输入，多次调用等价于一次性输入
---------------------------------------------------------------------------*/
void md5_update(md5_context_t *ctx, const uint8_t *input, size_t ilen)
{
    size_t fill;
    uint32_t left;

    if (ilen == 0) return;

    left = ctx->count[0] & 0x3F;
    fill = 64 - left;

    ctx->count[0] += (uint32_t)ilen;
    ctx->count[0] &= 0xFFFFFFFF;

    if (ctx->count[0] < (uint32_t)ilen)
        ctx->count[1]++;

    if (left && ilen >= fill) {
        memcpy((void *)(ctx->buffer + left), input, fill);
        md5_transform(ctx, ctx->buffer);
        input += fill;
        ilen -= fill;
        left = 0;
    }

    while (ilen >= 64) {
        md5_transform(ctx, input);
        input += 64;
        ilen -= 64;
    }

    if (ilen > 0) {
        memcpy((void *)(ctx->buffer + left), input, ilen);
    }
}

/*---------------------------------------------------------------------------
 Name        : md5_final
 Input       : ctx    - MD5上下文指针
               output - 输出缓冲区(16字节)
 Output      : 无
 Description :
 结束MD5计算并输出最终摘要(16字节)。
 行为：
 - 按MD5规范追加 0x80 + 0x00 padding，使消息长度满足 (mod 64) == 56
 - 在最后8字节写入原始消息长度(单位bit，小端序)
 - 调用 `md5_transform()` 处理最后一个/两个分组
 - 以小端序输出 `ctx->state` 到 output
---------------------------------------------------------------------------*/
void md5_final(md5_context_t *ctx, uint8_t output[16])
{
    uint32_t used, available;
    //uint8_t padding[64];

    used = ctx->count[0] & 0x3F;
    ctx->buffer[used++] = 0x80;

    available = 64 - used;

    if (available < 8) {
        memset(ctx->buffer + used, 0, available);
        md5_transform(ctx, ctx->buffer);
        used = 0;
        available = 64;
    }

    memset(ctx->buffer + used, 0, available - 8);

    // 添加长度(位数，小端序)
    uint32_t high = (ctx->count[0] >> 29) | (ctx->count[1] << 3);
    uint32_t low = (ctx->count[0] << 3);

    ctx->buffer[56] = (uint8_t)(low);
    ctx->buffer[57] = (uint8_t)(low >> 8);
    ctx->buffer[58] = (uint8_t)(low >> 16);
    ctx->buffer[59] = (uint8_t)(low >> 24);
    ctx->buffer[60] = (uint8_t)(high);
    ctx->buffer[61] = (uint8_t)(high >> 8);
    ctx->buffer[62] = (uint8_t)(high >> 16);
    ctx->buffer[63] = (uint8_t)(high >> 24);

    md5_transform(ctx, ctx->buffer);

    // 输出结果(小端序)
    for (int i = 0; i < 4; i++) {
        output[i * 4]     = (uint8_t)(ctx->state[i]);
        output[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 8);
        output[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 16);
        output[i * 4 + 3] = (uint8_t)(ctx->state[i] >> 24);
    }
}

/*---------------------------------------------------------------------------
 Name        : md5_hash
 Input       : input  - 输入数据指针
               ilen   - 输入数据长度(字节)
               output - 输出缓冲区(16字节)
 Output      : 无
 Description :
 一次性计算MD5摘要的便捷封装。
 内部依次调用 `md5_init()` / `md5_update()` / `md5_final()`。
---------------------------------------------------------------------------*/
void md5_hash(const uint8_t *input, size_t ilen, uint8_t output[16])
{
    md5_context_t ctx;
    md5_init(&ctx);
    md5_update(&ctx, input, ilen);
    md5_final(&ctx, output);
}

/*---------------------------------------------------------------------------
 Name        : md5_encrypt_code
 Input       : product_secret - 产品密钥字符串
               device_sn      - 设备序列号字符串
               code_out       - 输出缓冲区(至少17字节，含结尾\\0)
 Output      : 无
 Description :
 生成设备注册/鉴权用的“加密码”(16字节十六进制字符串)。
 算法：
 - 拼接 `product_secret + device_sn` 得到 combined
 - 对 combined 做MD5，得到16字节摘要
 - 取摘要的第4~11字节(共8字节)，转换为16个十六进制字符输出到 `code_out`
 约束：
 - `code_out` 必须可写，且至少容纳17字节(16字符 + '\\0')。
---------------------------------------------------------------------------*/
void md5_encrypt_code(const char *product_secret, const char *device_sn, char *code_out)
{
    uint8_t md5_result[16];
    char combined[128];

    // 拼接 product_secret + device_sn
    snprintf(combined, sizeof(combined), "%s%s", product_secret, device_sn);

    // 计算 MD5
    md5_hash((const uint8_t *)combined, strlen(combined), md5_result);

    // 取中间 8 字节(索引 4-11)，转为 16 字符十六进制
    for (int i = 0; i < 8; i++) {
        uint8_t byte = md5_result[i + 4];
        uint8_t high_nibble = (byte >> 4) & 0x0F;
        uint8_t low_nibble = byte & 0x0F;

        // 转为 ASCII 十六进制字符
        code_out[i * 2] = (high_nibble < 10) ? ('0' + high_nibble) : ('a' + high_nibble - 10);
        code_out[i * 2 + 1] = (low_nibble < 10) ? ('0' + low_nibble) : ('a' + low_nibble - 10);
    }

    code_out[16] = '\0';
}
