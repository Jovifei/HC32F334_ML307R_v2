#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ML307R UART 配置
#define ML307R_UART_BAUD      115200

// MQTT Broker 配置
#define MQTT_SERVER           "mqtt.dream-maker.com"
#define MQTT_PORT             8883

// 设备信息（新配置 - DM-MIS800）
// 注意：SW_VERSION 在 main.h 中定义，此处不再重复定义
#define PRODUCT_SN            "GTEST1000000011"
#define PRODUCT_ID            "669f128f59b7727830b3b5fc"
#define PRODUCT_MODEL         "DM-MIS800"
#define PRODUCT_SECRET        "WIr5vVBRHmURu8PB"

// 旧设备信息（兼容性保留）
#define DEVICE_SN             PRODUCT_SN
#define DEVICE_PRODUCT_MODEL  PRODUCT_MODEL
#define MCU_FIRMWARE_VERSION  SW_VERSION

// AT 命令超时 (ms)
#define AT_TIMEOUT_DEFAULT    3000
#define AT_TIMEOUT_LONG       10000

// UART RX 环形缓冲区大小
#define UART_AT_RX_BUF_SIZE   512

// MQTT 消息回调最大数量
#define MQTT_CB_MAX           4

#endif // CONFIG_H
