/*
 * wifi_info.h
 *
 * CT设备SN信息结构体定义
 * 原 wifi.h 已删除，仅保留 wifi_info_t 类型
 */

#ifndef APP_INC_WIFI_INFO_H_
#define APP_INC_WIFI_INFO_H_

#include <stdint.h>

typedef struct
{
    char sn[16];       // CT的SN (15+1字节)
    char version[12];
    char mac[12];
    char ssid[24];
    char passwd[24];
    uint16_t rssi;
    uint8_t net;
    uint16_t ble_on_enable;
    uint16_t debug_enable;
} wifi_info_t;

extern wifi_info_t wifi_info;

#endif /* APP_INC_WIFI_INFO_H_ */
