//
// Included Files
//
#include "board.h"
#include "main.h"
#include "wifi.h"
#include "eeprom.h"
#include "arm_math.h"
#include "sub1g.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "stdbool.h"
#include "flash.h"
#include "debug.h"

extern void wifi_set_phase_sequence(uint8_t sequence_k);

void bytes_to_wifi(char *buffer, uint16_t len);
void wifi_properties_para(char *string, wifi_msg_params_t *params_list);
void wifi_get_properties_reply(char *string, wifi_msg_params_t *params);
uint16_t wifi_set_properties_reply(char *string, wifi_msg_params_t *params, bool enable_thr, float low_thr, float high_thr);
void wifi_set_properties_not_found_reply(char *string, wifi_msg_params_t *params);
static void report_inverter_properties(uint8_t inv_idx);
static void report_inverter_properties_scheduled(uint8_t inv_idx);
void report_inverter_set_param(uint8_t inv_idx);

static void check_and_report_versions(void);
static void report_ct_siid3_properties(void);
static void report_ct_siid2_properties(void);
static void update_power_date_check(void);

#define GET_DOWN_MS_INTERVAL 200
#define PROP_CT_MS_INTERVAL 300000  // CT设备属性上报间隔(ms) - 5分钟
#define PROP_INV_MS_INTERVAL 180000 // 微逆定时上报间隔(ms)

#define GET_WIFI_STATE_MS_INTERVAL0 3000 // 1000
#define GET_WIFI_STATE_MS_INTERVAL1 60000
#define WAIT_COMMAND_TIMEOUT_MS 1000  // 等待回复1秒超时
#define WIFI_COMM_3MIN_TIMEOUT 180000 // 3分钟WiFi通信超时时间(ms)

char wifi_rx_buffer[COM_RX_BUFFER_SIZE];
char wifi_tx_buffer[COM_TX_BUFFER_SIZE];
uint16_t wifi_rx_index = 0;
static char report_bind_sn[SN_LENGTH + 1];
// uint16_t wifi_tx_index = 0;
// uint16_t wifi_tx_len = 0;

wifi_msg_params_t wifi_msg_params[24];
wifi_info_t wifi_info;

wifi_tx_flag_t tx_flag;
wifi_tx_type_t tx_type;
static bool msg_flag;

void wifi_task(void)
{
#if FUNCTION_WIFI_ENABLE
    if (msg_flag == true)
    {
        msg_flag = false;
        if (tx_type == OTA_READY)
        {
            if (strstr((char *)wifi_rx_buffer, "ok")) // 暂时不管接收到ERROR
            {
            }
            else
            {
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == PROP_EVENT) // 如果发送的消息为设置wifi、产品信息，上报属性，上报事件
        {
            if (strstr((char *)wifi_rx_buffer, "ok")) // 暂时不管接收到ERROR
            {
            }
            else
            {
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == SET_DEVICE) // 设置产品类型
        {
            if (strstr((char *)wifi_rx_buffer, "ok"))
            {
                tx_flag.device = false;
            }
            else
            {
                tx_flag.device = true;
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == SET_MCU_VERSION) // 设置MCU固件版本
        {
            if (strstr((char *)wifi_rx_buffer, "ok"))
            {
                tx_flag.mcu_version = false;
            }
            else
            {
                tx_flag.mcu_version = true;
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_SLAVE_VERSION) // 设置Slave设备版本
        {
            if (strstr((char *)wifi_rx_buffer, "ok"))
            {
                tx_flag.slave_version = false;
                sys_param.slave_version.slave_version_reported = true;
                tx_flag.slave_version_report_ms = 0; // 下次1小时后再上报
            }
            else
            {
                tx_flag.slave_version = true;
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_MAC) // 主动获取mac地址
        {
            memcpy((char *)wifi_info.mac, (char *)wifi_rx_buffer, 12);
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_TIME) // 主动获取时间
        {
            int year = 0;
            if (strlen(wifi_rx_buffer) > 0)
            {
                char year_str[5];
                strncpy(year_str, wifi_rx_buffer, 4);
                year_str[4] = '\0';
                year = atoi(year_str); // 将年份字符串转换为整数
            }
            if (year >= 2024)
            {
                memcpy(sys_param.time.date_time, wifi_rx_buffer, 19);
                sys_param.time.date_time[19] = '\0'; // 添加结束符

                // 提取日期部分 "YYYY-MM-DD"
                memcpy(sys_param.time.date, wifi_rx_buffer, 10);
                sys_param.time.date[10] = '\0';

                // 提取时间部分 "HH:MM:SS"
                memcpy(sys_param.time.time, wifi_rx_buffer + 11, 8);
                sys_param.time.time[8] = '\0';

                // 检查日期变更，如果确实变更了会自动清零当日发电量
                update_power_date_check();
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_VERSION)
        {
            memcpy((char *)wifi_info.version, (char *)wifi_rx_buffer, strlen((char *)wifi_rx_buffer) - 1);
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_RSSI)
        {
            sscanf((char *)wifi_rx_buffer, "%d", (int *)&wifi_info.rssi);
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == SET_BLE_ONOFF)
        {
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_NET) // 主动获取联网状态
        {
            if (strstr((char *)wifi_rx_buffer, "uap"))
            {
                wifi_info.net = UAP;
            }
            else if (strstr((char *)wifi_rx_buffer, "offline"))
            {
                wifi_info.net = OFFLINE;
            }
            else if (strstr((char *)wifi_rx_buffer, "local"))
            {
                wifi_info.net = LOCAL;
            }
            else if (strstr((char *)wifi_rx_buffer, "cloud"))
            {
                wifi_info.net = CLOUD;
                tx_flag.offline_count = 0;
            }
            else if (strstr((char *)wifi_rx_buffer, "unprov"))
            {
                wifi_info.net = OFFLINE;
                tx_flag.device = 1;
            }
            else if (strstr((char *)wifi_rx_buffer, "updating"))
            {
                // wifi_info.net =CLOUD;
                tx_flag.offline_count = 0;
            }
            tx_type = WIFI_TX_NULL;
        }
        else if (tx_type == GET_DOWN) // 主动下拉消息
        {
            if (strstr((char *)wifi_rx_buffer, "down none")) // 未收到下拉消息
            {
                tx_type = WIFI_TX_NULL;
            }
            else if (strstr((char *)wifi_rx_buffer, "DMIOT_net_change")) // wifi下发联网状态改变
            {
                if (strstr((char *)wifi_rx_buffer, "uap"))
                {
                    wifi_info.net = UAP;
                }
                else if (strstr((char *)wifi_rx_buffer, "offline"))
                {
                    wifi_info.net = OFFLINE;
                }
                else if (strstr((char *)wifi_rx_buffer, "local"))
                {
                    wifi_info.net = LOCAL;
                }
                else if (strstr((char *)wifi_rx_buffer, "cloud"))
                {
                    wifi_info.net = CLOUD;
                    tx_flag.offline_count = 0;
                }
                else if (strstr((char *)wifi_rx_buffer, "unprov"))
                {
                    wifi_info.net = OFFLINE;
                    tx_flag.device = 1;
                }
                else if (strstr((char *)wifi_rx_buffer, "updating"))
                {
                    tx_flag.offline_count = 0;
                }
                tx_type = WIFI_TX_NULL;
                // wifi_comm_count = 0;
            }
            else if (strstr((char *)wifi_rx_buffer, "down DMIOT_mcu_version_req"))
            {
                tx_flag.mcu_version = true;
                DEBUG_PRINTF("[WiFi] Req mcu_version\n");
            }
            else if (strstr((char *)wifi_rx_buffer, "down update_fw")) // wifi下发升级命令
            {
                char *st = 0;
                uint32_t fw_type = 0;
                uint32_t fw_len = 0;
                tx_flag.offline_count = 0;
                tx_type = WIFI_TX_NULL;
                st = strstr(wifi_rx_buffer, " ") + 1;
                st = strstr(st, " ") + 1;
                st = strstr(st, " ") + 1;
                sscanf(st, "%d", &fw_len);
                st = strstr(st, " ") + 1;
                sscanf(st, "%d", &fw_type);
                if (fw_len > 10000 && fw_type == 1) // MCU自身固件,立即重启进入bootloader下载固件
                {
                    int ret = LL_OK;
                    uint32_t u32Temp = APP_UPGRADE_FLAG;
                    ret = FLASH_EraseSector(APP_UPGRADE_FLAG_SECTOR, 4096); // 清除升级标志扇区
                    if (ret == LL_OK)
                        FLASH_WriteData(APP_UPGRADE_FLAG_ADDR, (uint8_t *)&u32Temp, 4U); // 写入升级标志
                    delay_ms(10);
                    if (ret == LL_OK)
                    {
                        NVIC_SystemReset(); // 重新进入bootloader,在bootloader下载固件
                    }
                }
            }
            else if (strstr((char *)wifi_rx_buffer, "down get_properties")) // wifi下发读取属性
            {
                uint8_t i = 0;
                memset(wifi_msg_params, 0, sizeof(wifi_msg_params_t) * 24);
                wifi_properties_para(wifi_rx_buffer, wifi_msg_params);
                wifi_tx_buffer[0] = 0;
                strcat((char *)wifi_tx_buffer, "result");
                for (i = 0; i < 24; i++)
                {
                    if (wifi_msg_params[i].flag == false)
                        break;

                    // 判断SIID范围
                    if (wifi_msg_params[i].siid >= SIID_MIN && wifi_msg_params[i].siid <= SIID_MAX)
                    {
                        // INV设备 (SIID 4-11)
                        uint8_t inv_index = wifi_msg_params[i].siid - SIID_MIN;

                        // 检查设备是否有效
                        if (!sys_param.paired_inv_info[inv_index].is_valid)
                        {
                            if (wifi_msg_params[i].piid == 1)
                            {
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = 0; // 没有配对的设备
                            }
                            else
                            {
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                            }
                        }
                        else
                        {
                            // 根据PIID读取对应属性
                            switch (wifi_msg_params[i].piid)
                            {
                            case INV_PIID_ONLINE_STATE:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].online_state;
                                break;
                            case INV_PIID_DEVICE_SN:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = sys_param.paired_inv_info[inv_index].device_sn;
                                break;
                            case INV_PIID_SW_VERSION:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = sys_param.paired_inv_info[inv_index].sw_version;
                                break;
                            case INV_PIID_SUB1G_VERSION:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = sys_param.paired_inv_info[inv_index].sub1g_version;
                                break;
                            case INV_PIID_PRODUCT_MODEL:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = sys_param.paired_inv_info[inv_index].product_model;
                                break;
                            case INV_PIID_WORK_STATE:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].work_state;
                                break;
                            case INV_PIID_GRID_POWER:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].grid_power;
                                break;
                            case INV_PIID_TODAY_ENERGY:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].today_energy;
                                break;
                            case INV_PIID_LIFETIME_ENERGY:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].lifetime_energy;
                                break;
                            case INV_PIID_ANTIFLOW_ENABLE:
                                if (sys_param.paired_inv_info[inv_index].antiflow_enable == 0)
                                {
                                    wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FALSE;
                                }
                                else
                                {
                                    wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_TRUE;
                                }
                                break;
                            case INV_PIID_INV_POWER_ENABLE:
                                if (sys_param.paired_inv_info[inv_index].power_enable == 0)
                                {
                                    wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FALSE;
                                }
                                else
                                {
                                    wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_TRUE;
                                }
                                break;
                            case INV_PIID_TODAY_POWER_TIME:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].today_power_time;
                                break;
                            case INV_PIID_FAULT_PARAM:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].fault_param;
                                break;
                            case INV_PIID_GRID_FREQUENCY:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].grid_frequency;
                                // wifi_msg_params[i].f_value = sys_param.grid.grid_frequency;
                                break;
                            case INV_PIID_GRID_VOLTAGE:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].grid_voltage;
                                // wifi_msg_params[i].f_value = sys_param.grid.ua_vol_rms;
                                break;
                            case INV_PIID_PV1_CURRENT:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].pv[0].current;
                                break;
                            case INV_PIID_PV1_VOLTAGE:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].pv[0].voltage;
                                break;
                            case INV_PIID_PV2_VOLTAGE:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].pv[1].voltage;
                                break;
                            case INV_PIID_PV2_CURRENT:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].pv[1].current;
                                break;
                            case INV_PIID_TEMPERATURE:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.paired_inv_info[inv_index].ambient_temperature;
                                break;
                            case INV_PIID_POWER_PHASE: // 微逆所在相位
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].phase;
                                break;

                            case INV_PIID_POWER_LIMIT:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].power_limit;
                                break;

                            case INV_PIID_CONNECTION_POINT:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].connection_point;
                                break;

                            case INV_PIID_PV1_POWER:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv[0].power;
                                break;

                            case INV_PIID_PV2_POWER:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv[1].power;
                                break;

                            case INV_PIID_PV3_POWER:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv[2].power;
                                break;

                            case INV_PIID_PV4_POWER:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv[3].power;
                                break;

                                // case INV_PIID_PV5_POWER:
                                //     wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                //     wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv[4].power;
                                //     break;

                                // case INV_PIID_PV6_POWER:
                                //     wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                //     wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv[5].power;
                                //     break;

                            case INV_PIID_PV_NUM:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].pv_num;
                                break;

                            case INV_PIID_SUBG_ADDR:
                            {
                                static char subg_addr_str[9]; //  "0x" + 6位十六进制 + '\0' = 9字节
                                snprintf(subg_addr_str, sizeof(subg_addr_str), "0x%06X", sys_param.paired_inv_info[inv_index].sub1g_addr);
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = subg_addr_str;
                                break;
                            }
                            case INV_PIID_CHANNEL_INDEX:
                            {
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].channel_index;
                                break;
                            }

                            case INV_PIID_PACKET_LOSS_RATE:
                            {
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.paired_inv_info[inv_index].plr;
                                break;
                            }

                            case 36:
                            {
                                sub1g_send_debug_mode(sys_param.paired_inv_info[inv_index].sub1g_addr);
                                break;
                            }

                            default:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                                break;
                            }
                        }
                    }
                    // SIID 1-3: CT设备本地处理
                    else if (wifi_msg_params[i].siid >= 1 && wifi_msg_params[i].siid <= 3)
                    {
                        switch (wifi_msg_params[i].siid)
                        {
                        case 1: // 设备信息
                        {
                            switch (wifi_msg_params[i].piid)
                            {
                            case 1:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = PRODUCT_MODEL;
                                break;
                            case 2:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                // 根据SN有效性上报不同的设备标识
                                if (sys_param.flash_sn_com_normal && strlen((char *)wifi_info.sn) == 15)
                                {
                                    wifi_msg_params[i].s_value = wifi_info.sn; // 使用SN
                                }
                                else
                                {
                                    wifi_msg_params[i].s_value = wifi_info.mac; // 使用MAC地址
                                }
                                break;
                            case 3:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = wifi_info.mac;
                                break;
                            case 4:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = SW_VERSION;
                                break;
                            case 5:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = HW_VERSION;
                                break;
                            case 6:
                                wifi_msg_params[i].value_type = (wifi_value_type_t)sys_param.wifi.restore_wifi_cmd;
                                break;
                            case 8:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = sys_param.sub1g.sw_version;
                                break;
                            case 9:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                char addr_str[7];
                                sprintf(addr_str, "%06X", sys_param.sub1g.ct_sub1g_addr);
                                wifi_msg_params[i].s_value = addr_str;
                                break;

                            default:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                                break;
                            }
                        }
                        break;
                        case 2: // 设备控制
                        {
                            switch (wifi_msg_params[i].piid)
                            {
                            case 1:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.state;
                                break;
                            case 2:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                if (fabsf(sys_param.ct1.power.fix_dir_power) < 10.0f)
                                {
                                    wifi_msg_params[i].f_value = 0.0f;
                                }
                                else
                                {
                                    wifi_msg_params[i].f_value = sys_param.ct1.power.fix_dir_power;
                                }
                                break;
                            case 3:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                if (fabsf(sys_param.ct2.power.fix_dir_power) < 10.0f)
                                {
                                    wifi_msg_params[i].f_value = 0.0f;
                                }
                                else
                                {
                                    wifi_msg_params[i].f_value = sys_param.ct2.power.fix_dir_power;
                                }
                                break;
                            case 4:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                if (fabsf(sys_param.ct3.power.fix_dir_power) < 10.0f)
                                {
                                    wifi_msg_params[i].f_value = 0.0f;
                                }
                                else
                                {
                                    wifi_msg_params[i].f_value = sys_param.ct3.power.fix_dir_power;
                                }
                                break;
                            case 5:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct1.inv_power;
                                break;
                            case 6:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct2.inv_power;
                                break;
                            case 7:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct3.inv_power;
                                break;
                            case 8:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct1.inv_power + sys_param.ct2.inv_power + sys_param.ct3.inv_power;
                                break;
                            case 9:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct1.use_power + sys_param.ct2.use_power + sys_param.ct3.use_power;
                                break;
                            case 10:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                if (fabsf(sys_param.ct1.power.fix_dir_power) < 10.0f)
                                {
                                    sys_param.ct1.power.fix_dir_power = 0.0f;
                                }
                                if (fabsf(sys_param.ct2.power.fix_dir_power) < 10.0f)
                                {
                                    sys_param.ct2.power.fix_dir_power = 0.0f;
                                }
                                if (fabsf(sys_param.ct3.power.fix_dir_power) < 10.0f)
                                {
                                    sys_param.ct3.power.fix_dir_power = 0.0f;
                                }

                                wifi_msg_params[i].f_value = sys_param.ct1.power.fix_dir_power + sys_param.ct2.power.fix_dir_power + sys_param.ct3.power.fix_dir_power;
                                break;
                            case 11:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct_today_power_time;
                                break;
                            case 12:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct_today_energy;
                                break;
                            case 13:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.hmi.electricity_generation;
                                break;
                            case 14:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.limit_state;
                                break;
                            case 15:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.grid.phase_id.sequence_k;
                                break;
                            case 16:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.grid.ua_vol_rms;
                                break;
                            case 17:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.grid.grid_frequency;
                                break;
                            case 18:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct1.power.ct_sub1g_boardcast_power_avg;
                                break;
                            case 19:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct2.power.ct_sub1g_boardcast_power_avg;
                                break;
                            case 20:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.ct3.power.ct_sub1g_boardcast_power_avg;
                                break;
                            case 21:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                                wifi_msg_params[i].f_value = sys_param.sub1g.state;
                                break;
                            default:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                                break;
                            }
                        }
                        break;
                        case 3: // 设备控制
                        {
                            switch (wifi_msg_params[i].piid)
                            {
                            case 2: // 相序识别状态（只读）
                                wifi_msg_params[i].value_type = (wifi_value_type_t)sys_param.fft_identify.is_ffting;
                                break;

                            case 3:
                            {
                                const char *paired_list = get_paired_device_list_string();
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = (char *)paired_list;
                                // printf("paired_list:%s\r\n", paired_list);
                                break;
                            }
                            case 4:
                            {
                                const char *inv_request_pair_list = get_inv_request_pair_list_string();
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = (char *)inv_request_pair_list;
                                // printf("inv_request_pair_list:%s\r\n", inv_request_pair_list);
                                break;
                            }

                            case 7:
                            {
                                const char *user_pair_list_str = get_user_pair_list_string();
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_STRING;
                                wifi_msg_params[i].s_value = (char *)user_pair_list_str;
                                // printf("user_pair_list_str:%s\r\n", user_pair_list_str);
                                break;
                            }
                            case 8: // 三相模式开关
                            {
                                if (sys_param.is_three_phase == false)
                                {
                                    wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_FALSE;
                                }
                                else
                                {
                                    wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_TRUE;
                                }
                                break;
                            }
                            case 10:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.power_work_mode;
                                break;

                            case 11:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.to_grid_power_limit;
                                break;
                            case 12:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.fft_identify.power;
                                break;

                            case 13:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.fft_identify.interval_time;
                                break;

                            case 15:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_INT;
                                wifi_msg_params[i].value = sys_param.sub1g.channel_index;
                                break;

                            default:
                                wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                                break;
                            }
                        }
                        break;
                        default:
                            wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                            break;
                        }
                    }
                    else
                    {
                        wifi_msg_params[i].value_type = WIFI_VALUE_TYPE_NOT_FOUND;
                    }

                    wifi_get_properties_reply(wifi_tx_buffer, &wifi_msg_params[i]);
                }

                strcat((char *)wifi_tx_buffer, "\r");
                bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
                tx_flag.timeout_count = 0;
                tx_type = WIFI_TX_NULL;
            }
            else if (strstr((char *)wifi_rx_buffer, "down set_properties")) // wifi下发写入属性
            {
                uint8_t i = 0;
                memset(wifi_msg_params, 0, sizeof(wifi_msg_params_t) * 24);
                wifi_properties_para(wifi_rx_buffer, wifi_msg_params);
                wifi_tx_buffer[0] = 0;
                strcat((char *)wifi_tx_buffer, "result");

                for (i = 0; i < 24; i++)
                {
                    if (wifi_msg_params[i].flag == false)
                        break;

                    // 判断SIID范围
                    if (wifi_msg_params[i].siid >= SIID_MIN && wifi_msg_params[i].siid <= SIID_MAX)
                    {
                        // SIID 4-11: INV设备，操作 sys_param.paired_inv_info[]
                        uint8_t inv_index = wifi_msg_params[i].siid - SIID_MIN;

                        // 检查设备是否有效
                        if (!sys_param.paired_inv_info[inv_index].is_valid)
                        {
                            wifi_set_properties_not_found_reply(wifi_tx_buffer, &wifi_msg_params[i]);
                        }
                        else
                        {
                            uint16_t ret = 1; // 默认失败

                            // 根据PIID设置属性
                            switch (wifi_msg_params[i].piid)
                            {
                            case INV_PIID_LIFETIME_ENERGY: // 累计发电量可更改
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FLOAT)
                                {
                                    sys_param.paired_inv_info[inv_index].lifetime_energy = (float)wifi_msg_params[i].f_value;
                                    // printf(" lifetime_energy = %.2f\r\n", sys_param.paired_inv_info[inv_index].lifetime_energy);
                                    ret = 0;
                                }
                                break;
                            case INV_PIID_ANTIFLOW_ENABLE: // 逆变器防逆流使能开关可更改
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FALSE)
                                {
                                    sys_param.paired_inv_info[inv_index].antiflow_enable = 0;
                                    // printf(" anti_flow_enable = 0 (false)\r\n");
                                    sub1g_send_set_antiflow(sys_param.paired_inv_info[inv_index].sub1g_addr, sys_param.paired_inv_info[inv_index].antiflow_enable);
                                    ret = 0;
                                }
                                else if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_TRUE)
                                {
                                    sys_param.paired_inv_info[inv_index].antiflow_enable = 1;
                                    sub1g_send_set_antiflow(sys_param.paired_inv_info[inv_index].sub1g_addr, sys_param.paired_inv_info[inv_index].antiflow_enable);
                                    // printf(" anti_flow_enable = 1 (true)\r\n");
                                    ret = 0;
                                }
                                else
                                {
                                    // printf("antiflow_enable: invalid value type\r\n");
                                    ret = 1;
                                }
                                break;

                            case INV_PIID_INV_POWER_ENABLE: // 逆变器发电使能开关可更改
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FALSE)
                                {
                                    sys_param.paired_inv_info[inv_index].power_enable = 0;
                                    sub1g_send_set_power_switch(sys_param.paired_inv_info[inv_index].sub1g_addr, sys_param.paired_inv_info[inv_index].power_enable);
                                    // printf(" power_enable = 0 (false)\r\n");
                                    ret = 0;
                                }
                                else if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_TRUE)
                                {
                                    sys_param.paired_inv_info[inv_index].power_enable = 1;
                                    sub1g_send_set_power_switch(sys_param.paired_inv_info[inv_index].sub1g_addr, sys_param.paired_inv_info[inv_index].power_enable);
                                    // printf(" power_enable = 1 (true)\r\n");
                                    ret = 0;
                                }
                                else
                                {
                                    // printf("power_enable: invalid value type\r\n");
                                    ret = 1;
                                }
                                break;

                            case INV_PIID_POWER_PHASE: // 微逆所在相位可设置
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FLOAT)
                                {
                                    uint8_t phase = (uint8_t)wifi_msg_params[i].f_value;
                                    if (phase <= 3) // 0=未识别, 1=A相, 2=B相, 3=C相
                                    {
                                        uint32_t target_addr = sys_param.paired_inv_info[inv_index].sub1g_addr;

                                        // 检查是否正在FFT识别该设备，如果是则停止识别并清理所有状态
                                        if (sys_param.fft_identify.is_ffting && sys_param.fft_identify.sub1g_addr == target_addr)
                                        {
                                            // 完整清理FFT识别状态
                                            sys_param.fft_identify.is_ffting = 0;
                                            sys_param.fft_identify.enable_collect = 0; // 停止FFT数据采集
                                            sys_param.fft_identify.resend_cmd = false;
                                            sys_param.fft_identify.sub1g_addr = 0;
                                            sys_param.fft_identify.consecutive_success_count = 0; // 清零连续成功次数
                                            sys_param.fft_identify.last_identified_ct = 0;        // 清除上次识别的CT号
                                            sys_param.fft_identify.identified_ct = 0;             // 清除识别结果
                                            sys_param.fft_identify.boardcast_interval = 0;        // 清除广播间隔计数器
                                            sys_param.fft_identify.final_confirm_pending = false; // 清除确认等待标志
                                            DEBUG_PRINTF("Stop FFT identify for inv[%d], WiFi set phase=%d\r\n", inv_index, phase);
                                        }

                                        // 更新内存和EEPROM
                                        if (eeprom_update_device_phase(target_addr, phase) == 0)
                                        {
                                            // 发送设置相位命令给微逆
                                            sub1g_send_set_inv_phase(target_addr, phase);
                                            ret = 0;

                                            DEBUG_PRINTF("WiFi set inv[%d] phase=%d, addr=0x%06X\r\n",
                                                         inv_index, phase, target_addr);
                                        }
                                    }
                                }
                                break;

                            case INV_PIID_POWER_LIMIT:
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FLOAT)
                                {
                                    uint16_t power_limit = (uint16_t)wifi_msg_params[i].f_value;
                                    sys_param.paired_inv_info[inv_index].power_limit = power_limit;

                                    // 发送设置功率限制命令给微逆
                                    sub1g_send_set_power_limit(sys_param.paired_inv_info[inv_index].sub1g_addr, power_limit);

                                    sys_param.paired_inv_info[inv_index].settings_changed = true;

                                    // printf("Set INV[%d] power_limit=%d\n", inv_index, power_limit);

                                    ret = 0;
                                }
                                break;

                            case INV_PIID_CONNECTION_POINT:
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FLOAT)
                                {
                                    uint8_t connection_point = (uint8_t)wifi_msg_params[i].f_value;
                                    if (connection_point <= 1) // 0=配电箱, 1=末端插座
                                    {
                                        sys_param.paired_inv_info[inv_index].connection_point = connection_point;

                                        // 发送设置接入点命令给微逆
                                        sub1g_send_set_connection_point(sys_param.paired_inv_info[inv_index].sub1g_addr, connection_point);
                                        ret = 0;
                                    }
                                }
                                break;

                            case INV_PIID_FFT_ENABLE:
                            {
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 1) == 0)
                                {
                                    if (wifi_msg_params[i].value_type == 1) // 写入1触发识别
                                    {
                                        // 检查是否已经在FFT识别中
                                        if (!sys_param.fft_identify.is_ffting)
                                        {
                                            sys_param.fft_identify.resend_cmd = true;
                                            sys_param.fft_identify.sub1g_addr = sys_param.paired_inv_info[inv_index].sub1g_addr;

#ifdef FFT_DEBUG_PRINT
                                            printf("WiFi手动触发FFT识别: addr=0X%06X\r\n", sys_param.paired_inv_info[inv_index].sub1g_addr);
#endif
                                        }
                                        else
                                        {
#ifdef FFT_DEBUG_PRINT
                                            printf("FFT识别忙: 设备0X%06X正在识别中\r\n", sys_param.fft_identify.sub1g_addr);
#endif
                                        }
                                    }
                                }
                                break;
                            }

                            default:
                                ret = 1; // 不支持设置的属性
                                break;
                            }

                            sys_param.paired_inv_info[inv_index].prop_changed = true; // 属性已更改，需要立即上报

                            // 生成回复
                            char array[32];
                            sprintf(array, " %d %d %d", wifi_msg_params[i].siid, wifi_msg_params[i].piid, (ret == 0 ? 0 : -4005));
                            strcat(wifi_tx_buffer, array);
                        }
                    }
                    else
                    {
                        // SIID 1-3: CT设备
                        switch (wifi_msg_params[i].siid)
                        {
                        case 1:
                        {
                            switch (wifi_msg_params[i].piid)
                            {
                            case 2:
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 6, 24) == 0)
                                {
                                    wifi_info.sn[0] = 0;
                                    strcat(wifi_info.sn, wifi_msg_params[i].s_value);
                                }
                                break;
                            case 6:
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 1) == 0)
                                {
                                    sys_param.wifi.restore_wifi_cmd = wifi_msg_params[i].value_type;
                                }
                                break;
                            case 10:
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 1) == 0)
                                {
                                    sys_param.wifi.clear_inverter_data_cmd = wifi_msg_params[i].value_type;
                                    sys_param.wifi.clear_inv_step_count = 0; // 初始化步骤计数器
                                }
                                break;
                            case 11: // 清除CT板EEPROM数据
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 1) == 0)
                                {
                                    if (wifi_msg_params[i].value_type == true)
                                    {
                                        // 清除用电量等统计数据
                                        sys_param.ct_today_energy = 0.0f;
                                        sys_param.ct_today_power_time = 0.0f;
                                        sys_param.hmi.electricity_consumption = 0;
                                        sys_param.to_grid_power_limit = 0;
                                        sys_param.power_work_mode = 1;
                                        sys_param.grid.phase_id.sequence_k = 1;
                                        sys_param.ct1.power.power_direction = 1;
                                        sys_param.ct2.power.power_direction = 1;
                                        sys_param.ct3.power.power_direction = 1;

                                        // 清除所有微逆的今日统计数据
                                        for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
                                        {
                                            if (sys_param.paired_inv_info[i].is_valid)
                                            {
                                                sys_param.paired_inv_info[i].today_energy = 0.0f;
                                                sys_param.paired_inv_info[i].today_power_time = 0.0f;
                                                sys_param.paired_inv_info[i].lifetime_energy = 0.0f;
                                            }
                                        }

                                        // 保存清零后的参数到EEPROM
                                        eeprom_save_set_param();
                                        eeprom_save_elec_consumption(); // 用电量独立区域也清零
                                        DEBUG_PRINTF("Clear CT EEPROM data completed\r\n");
                                    }
                                }
                                break;
                            default:
                                wifi_set_properties_not_found_reply(wifi_tx_buffer, &wifi_msg_params[i]);
                                break;
                            }
                        }
                        break;

                        case 2: // CT三相相序设置
                        {
                            switch (wifi_msg_params[i].piid)
                            {
                            case 15: // 相序设置 sequence_k (0-6)
                            {
                                uint8_t sequence_value = 0;
                                int ret = -4005;

                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_FLOAT)
                                {
                                    sequence_value = (uint8_t)wifi_msg_params[i].f_value;
                                }
                                else if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_INT)
                                {
                                    sequence_value = (uint8_t)wifi_msg_params[i].value;
                                }

                                if (sequence_value <= 6)
                                {
                                    wifi_set_phase_sequence(sequence_value);
                                    ret = 0;
                                }

                                char array[32];
                                sprintf(array, " %d %d %d", wifi_msg_params[i].siid, wifi_msg_params[i].piid, ret);
                                strcat(wifi_tx_buffer, array);
                                break;
                            }

                            default:
                                wifi_set_properties_not_found_reply(wifi_tx_buffer, &wifi_msg_params[i]);
                                break;
                            }
                        }
                        break;

                        case 3:
                        {
                            switch (wifi_msg_params[i].piid)
                            {
                            case 1:
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 1) == 0)
                                {
                                    sys_param.restore_sys = wifi_msg_params[i].value_type;
                                }
                                break;
                            case 5:
                            {
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_STRING)
                                {
                                    if (strlen(wifi_msg_params[i].s_value) <= SN_LENGTH)
                                    {
                                        if (user_pair_list_add(wifi_msg_params[i].s_value))
                                        {
                                            // 添加到用户配对列表成功
                                            char array[32];
                                            sprintf(array, " %d %d 0", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                            strcat(wifi_tx_buffer, array);

                                            // 设置立即上报配对列表标志
                                            tx_flag.immediate_report_bind = 1;

                                            strncpy(report_bind_sn, wifi_msg_params[i].s_value, SN_LENGTH);
                                            report_bind_sn[SN_LENGTH] = '\0';

                                            // printf("User pair request added: SN=%s\r\n", wifi_msg_params[i].s_value);
                                        }
                                        else
                                        {
                                            // 添加失败（列表已满或其他错误）
                                            char array[32];
                                            sprintf(array, " %d %d -4004", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                            strcat(wifi_tx_buffer, array);

                                            // printf("User pair request failed: SN=%s\r\n", wifi_msg_params[i].s_value);
                                        }
                                    }
                                    else
                                    {
                                        // SN长度错误
                                        wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], false, 0, 0);
                                    }
                                }
                                else
                                {
                                    // 参数类型错误
                                    wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], false, 0, 0);
                                }
                                break;
                            }

                                // case 6:
                                // {
                                //     if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_STRING)
                                //     {
                                //         char *input_str = wifi_msg_params[i].s_value;
                                //         uint16_t input_len = strlen(input_str);
                                //         bool unbind_success = false;
                                //         char unbind_sn[SN_LENGTH + 1];

                                //         if (input_len == 6)
                                //         {
                                //             // 可能是6位Sub1G地址，尝试解析
                                //             uint32_t sub1g_addr = 0;
                                //             sscanf(input_str, "%06X", &sub1g_addr);
                                //             sub1g_addr &= 0xFFFFFF;

                                //             // 调用find_inv_index_by_sub1g_addr查找设备
                                //             uint8_t inv_index = find_inv_index_by_sub1g_addr(sub1g_addr);

                                //             if (inv_index < INV_DEVICE_MAX_NUM)
                                //             {
                                //                 // 找到设备，获取SN
                                //                 strncpy(unbind_sn, sys_param.paired_inv_info[inv_index].device_sn, SN_LENGTH);
                                //                 unbind_sn[SN_LENGTH] = '\0';
                                //                 // printf("找到设备，使用SN解绑: %s\n", unbind_sn);
                                //             }
                                //             else
                                //             {
                                //                 // 未找到设备，把输入当作SN处理
                                //                 strncpy(unbind_sn, input_str, SN_LENGTH);
                                //                 unbind_sn[SN_LENGTH] = '\0';
                                //                 // printf("未找到设备，输入作为SN解绑: %s\n", unbind_sn);
                                //             }
                                //         }
                                //         else
                                //         {
                                //             // 非6位数据，直接当作SN处理
                                //             if (input_len <= SN_LENGTH)
                                //             {
                                //                 strncpy(unbind_sn, input_str, SN_LENGTH);
                                //                 unbind_sn[SN_LENGTH] = '\0';
                                //                 // printf("非6位输入，作为SN解绑: %s\n", unbind_sn);
                                //             }
                                //             else
                                //             {
                                //                 // SN长度超限
                                //                 wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], false, 0, 0);
                                //                 break;
                                //             }
                                //         }

                                //         // 统一调用user_unbind_device_by_sn
                                //         unbind_success = user_unbind_device_by_sn(unbind_sn);

                                //         // 生成回复
                                //         if (unbind_success)
                                //         {
                                //             char array[32];
                                //             sprintf(array, " %d %d 0", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                //             strcat(wifi_tx_buffer, array);

                                //             // 设置立即上报解绑标志
                                //             tx_flag.immediate_report_unbind = 1;

                                //             strncpy(report_bind_sn, wifi_msg_params[i].s_value, SN_LENGTH);
                                //             report_bind_sn[SN_LENGTH] = '\0';
                                //         }
                                //         else
                                //         {
                                //             char array[32];
                                //             sprintf(array, " %d %d -4004", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                //             strcat(wifi_tx_buffer, array);
                                //         }
                                //     }
                                //     else
                                //     {
                                //         // 参数类型错误
                                //         wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], false, 0, 0);
                                //     }
                                //     break;
                                // }
                            case 6:
                            {
                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_STRING)
                                {
                                    char *input_str = wifi_msg_params[i].s_value;
                                    uint16_t input_len = strlen(input_str);
                                    bool unbind_success = false;
                                    char unbind_sn[SN_LENGTH + 1];

                                    if (input_len == 6)
                                    {
                                        // 6位：可能是Sub1G地址
                                        uint32_t sub1g_addr = 0;
                                        uint8_t inv_index = INV_DEVICE_MAX_NUM;
                                        sscanf(input_str, "%06X", &sub1g_addr);
                                        sub1g_addr &= 0xFFFFFF;

                                        for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
                                        {
                                            if (sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr)
                                            {
                                                inv_index = i;
                                            }
                                        }

                                        if (inv_index < INV_DEVICE_MAX_NUM)
                                        {
                                            // 已配对设备：使用Sub1G地址解绑
                                            unbind_success = user_unbind_device_by_sub1g_addr(sub1g_addr);

                                            strncpy(unbind_sn, sys_param.paired_inv_info[inv_index].device_sn, SN_LENGTH);
                                        }
                                        else
                                        {
                                            // 发送解绑命令给微逆
                                            sub1g_send_unbind(sub1g_addr);
                                            unbind_success = true; // 广播解绑视为成功
                                        }
                                    }
                                    else
                                    {
                                        // 非6位：当作SN处理
                                        if (input_len <= SN_LENGTH)
                                        {
                                            strncpy(unbind_sn, input_str, SN_LENGTH);
                                            unbind_sn[SN_LENGTH] = '\0';

                                            // 尝试按SN解绑
                                            unbind_success = user_unbind_device_by_sn(unbind_sn);

                                            if (!unbind_success)
                                            {
                                                // 未找到已配对设备：使用SN广播解绑
                                                sub1g_send_broadcast_unbind_by_sn(unbind_sn);
                                                unbind_success = true; // 广播解绑视为成功
                                            }
                                        }
                                        else
                                        {
                                            // SN长度超限
                                            wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], false, 0, 0);
                                            break;
                                        }
                                    }

                                    // 生成回复
                                    if (unbind_success)
                                    {
                                        char array[32];
                                        sprintf(array, " %d %d 0", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                        strcat(wifi_tx_buffer, array);

                                        tx_flag.immediate_report_unbind = 1;
                                        // 设置立即上报解绑标志
                                        strncpy(report_bind_sn, unbind_sn, SN_LENGTH);
                                        report_bind_sn[SN_LENGTH] = '\0';
                                    }
                                    else
                                    {
                                        char array[32];
                                        sprintf(array, " %d %d -4004", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                        strcat(wifi_tx_buffer, array);
                                    }
                                }
                                else
                                {
                                    // 参数类型错误
                                    wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], false, 0, 0);
                                }
                                break;
                            }

                            case 8: // 三相模式开关

                                if (wifi_msg_params[i].value_type == WIFI_VALUE_TYPE_TRUE)
                                {
                                    sys_param.is_three_phase = true;
                                    // printf("WiFi: is_three_phase = TRUE\n");
                                }
                                else
                                {
                                    sys_param.is_three_phase = false;
                                    // printf("WiFi: is_three_phase = FALSE\n");
                                }
                                break;

                            case 10:
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 1, 3) == 0)
                                {
                                    sys_param.power_work_mode = (uint8_t)wifi_msg_params[i].f_value;
                                    DEBUG_PRINTF("WiFi: power_work_mode = %d\n", sys_param.power_work_mode);
                                    eeprom_save_set_param();
                                }
                                break;

                            case 11:
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 20000) == 0)
                                {
                                    sys_param.to_grid_power_limit = (uint16_t)wifi_msg_params[i].f_value;
                                    DEBUG_PRINTF("WiFi: to_grid_power_limit = %d\n", sys_param.to_grid_power_limit);
                                    eeprom_save_set_param();
                                }
                                break;
                            case 12: // FFT识别功率阈值（可设置范围：0-65535W）
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 1000) == 0)
                                {
                                    sys_param.fft_identify.power = (uint16_t)wifi_msg_params[i].f_value;
                                }
                                break;

                            case 13: // FFT识别时间间隔（仅允许：4, 6, 8, 10, 12）
                            {
                                uint8_t interval = (uint8_t)wifi_msg_params[i].f_value;
                                if (interval == 4 || interval == 6 || interval == 8 ||
                                    interval == 10 || interval == 12)
                                {
                                    sys_param.fft_identify.interval_time = interval;

                                    char array[32];
                                    sprintf(array, " %d %d 0", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                    strcat(wifi_tx_buffer, array);
                                }
                                else
                                {
                                    // 参数值不在允许范围内
                                    char array[32];
                                    sprintf(array, " %d %d -4005", wifi_msg_params[i].siid, wifi_msg_params[i].piid);
                                    strcat(wifi_tx_buffer, array);
                                }
                                break;
                            }

                            case 14: // 反转CT方向识别
                            {
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 1, 3) == 0)
                                {
                                    uint8_t reset_ct_dir = (uint8_t)wifi_msg_params[i].f_value;
                                    if (reset_ct_dir == 1)
                                    {
                                        sys_param.ct1.power.power_direction *= (-1);
                                    }
                                    else if (reset_ct_dir == 2)
                                    {
                                        sys_param.ct2.power.power_direction *= (-1);
                                    }
                                    else if (reset_ct_dir == 3)
                                    {
                                        sys_param.ct3.power.power_direction *= (-1);
                                    }
                                    eeprom_save_set_param();
                                }
                                break;
                            }
                            case 15: // 强制设置信道
                            {
                                if (wifi_set_properties_reply(wifi_tx_buffer, &wifi_msg_params[i], true, 0, 9) == 0)
                                {
                                    sys_param.sub1g.channel_index = (uint8_t)wifi_msg_params[i].f_value;
                                    sub1g_send_force_channel();
                                }
                                break;
                            }

                            default:
                                wifi_set_properties_not_found_reply(wifi_tx_buffer, &wifi_msg_params[i]);
                                break;
                            }
                        }
                        break;
                        default:
                            wifi_set_properties_not_found_reply(wifi_tx_buffer, &wifi_msg_params[i]);
                            break;
                        }
                    }
                }

                strcat((char *)wifi_tx_buffer, "\r");
                bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
                tx_flag.timeout_count = 0;
                tx_type = WIFI_TX_NULL;
                // wifi_comm_count = 0;
            }
            else if (strstr((char *)wifi_rx_buffer, "down action ")) // wifi下发方法指令->暂无action功能
            {
                tx_type = WIFI_TX_NULL;
                // wifi_comm_count = 0;
            }
            else // 未知消息
            {
                tx_type = WIFI_TX_NULL;
            }
        }
    }
    if (tx_type == WIFI_TX_NULL) // 已经空闲了
    {
        if (tx_flag.device == true) // 需要配置model
        {
            tx_type = SET_DEVICE;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "device ");
            strcat((char *)wifi_tx_buffer, PRODUCT_ID);
            strcat((char *)wifi_tx_buffer, " ");
            strcat((char *)wifi_tx_buffer, PRODUCT_SECRET);
            strcat((char *)wifi_tx_buffer, " ");
            strcat((char *)wifi_tx_buffer, PRODUCT_MODEL);
            strcat((char *)wifi_tx_buffer, " ");

            // 根据SN有效性上报不同的设备标识：使用SN或者MAC地址
            if (sys_param.flash_sn_com_normal && strlen(wifi_info.sn) == 15)
            {
                strcat((char *)wifi_tx_buffer, wifi_info.sn);
            }
            else
            {
                strcat((char *)wifi_tx_buffer, wifi_info.mac);
            }

            strcat((char *)wifi_tx_buffer, "\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.mcu_version == true) // 需要配置mcu固件版本号
        {
            tx_type = SET_MCU_VERSION;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "mcu_version ");
            strcat((char *)wifi_tx_buffer, SW_VERSION);
            strcat((char *)wifi_tx_buffer, "\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.slave_version == true) // 上报slave设备版本号
        {
            tx_type = GET_SLAVE_VERSION;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "slave_version ");

            DEBUG_PRINTF("[WiFi] report slave version:\r\n");

            // 类型3: CT的Sub1G版本
            strcat((char *)wifi_tx_buffer, "3 ");
            strcat((char *)wifi_tx_buffer, sys_param.sub1g.sw_version);
            DEBUG_PRINTF(" - Type3 (CT  Sub1G): %s\r\n", sys_param.sub1g.sw_version);

            // 类型4: 微逆Sub1G版本（如果有）
            if (sys_param.slave_version.inv_sub1g_version[0] != '\0')
            {
                strcat((char *)wifi_tx_buffer, " 4 ");
                strcat((char *)wifi_tx_buffer, sys_param.slave_version.inv_sub1g_version);
                DEBUG_PRINTF(" - Type4 (Inv Sub1G): %s\r\n", sys_param.slave_version.inv_sub1g_version);
            }
            else
            {
                DEBUG_PRINTF(" - Type4 (Inv Sub1G): NULL\r\n");
            }

            // 类型5: 800W微逆MCU版本（如果有）
            if (sys_param.slave_version.inv_800w_version[0] != '\0')
            {
                strcat((char *)wifi_tx_buffer, " 5 ");
                strcat((char *)wifi_tx_buffer, sys_param.slave_version.inv_800w_version);
                DEBUG_PRINTF(" - Type5 (800W  Inv): %s\r\n", sys_param.slave_version.inv_800w_version);
            }
            else
            {
                DEBUG_PRINTF(" - Type5 (800W  Inv): NULL\r\n");
            }

            // 类型6: 2500W微逆MCU版本（如果有）
            if (sys_param.slave_version.inv_2500w_version[0] != '\0')
            {
                strcat((char *)wifi_tx_buffer, " 6 ");
                strcat((char *)wifi_tx_buffer, sys_param.slave_version.inv_2500w_version);
                DEBUG_PRINTF(" - Type6 (2500W Inv): %s\r\n", sys_param.slave_version.inv_2500w_version);
            }
            else
            {
                DEBUG_PRINTF(" - Type6 (2500W Inv): NULL\r\n");
            }

            strcat((char *)wifi_tx_buffer, "\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if ((tx_flag.get_net_count >= GET_WIFI_STATE_MS_INTERVAL1 && wifi_info.net == CLOUD) ||
                 (tx_flag.get_net_count >= GET_WIFI_STATE_MS_INTERVAL0 && wifi_info.net != CLOUD)) // 需要拉取联网状态<-未连上服务器拉取频繁一些
        {
            tx_type = GET_NET;
            tx_flag.get_net_count = 0;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "net\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.mac == 1)
        {
            tx_flag.mac = 0;
            tx_type = GET_MAC;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "mac\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.version == 1)
        {
            tx_flag.version = 0;
            tx_type = GET_VERSION;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "version\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.restore == true || sys_param.wifi.restore_wifi_cmd == true) // 需要复位wifi
        {
            tx_flag.restore = false;
            sys_param.wifi.restore_wifi_cmd = false;
            tx_type = PROP_EVENT;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "restore\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (sys_param.wifi.clear_inverter_data_cmd == 1) // 正在处理清除微逆数据
        {
            uint8_t current_index = sys_param.wifi.clear_inv_step_count % INV_DEVICE_MAX_NUM; // 当前设备索引(0-7)

            // 检查当前设备是否有效
            if (sys_param.paired_inv_info[current_index].is_valid)
            {
                // 发送清除数据命令（0x26）
                sub1g_send_clear_data(sys_param.paired_inv_info[current_index].sub1g_addr);

                DEBUG_PRINTF("[WIFI] Step[%d/40] Send clear SN=%s data\r\n", sys_param.wifi.clear_inv_step_count, sys_param.paired_inv_info[current_index].device_sn);
            }

            // 步骤计数器加1
            sys_param.wifi.clear_inv_step_count++;

            // 检查是否完成全部40步 (8设备 * 5轮 = 40)
            if (sys_param.wifi.clear_inv_step_count >= 40)
            {

                // 重新初始化sys_param中的设备信息
                for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
                {
                    sys_param.paired_inv_info[i].work_state = 0; // 工作状态

                    sys_param.paired_inv_info[i].grid_power = 0;       // 发电功率(W)
                    sys_param.paired_inv_info[i].today_power_time = 0; // 今日发电时长(h)
                    sys_param.paired_inv_info[i].today_energy = 0;     // 今日发电量(Wh)
                    sys_param.paired_inv_info[i].lifetime_energy = 0;  // 累计发电量(Wh)

                    // 清除PV数据
                    for (uint8_t pv_idx = 0; pv_idx < 4; pv_idx++)
                    {
                        sys_param.paired_inv_info[i].pv[pv_idx].state = 0;
                        sys_param.paired_inv_info[i].pv[pv_idx].power = 0;
                        sys_param.paired_inv_info[i].pv[pv_idx].voltage = 0.0f;
                        sys_param.paired_inv_info[i].pv[pv_idx].current = 0.0f;
                    }
                }

                // 清除处理标志
                sys_param.wifi.clear_inverter_data_cmd = 0;
                sys_param.wifi.clear_inv_step_count = 0;
            }
        }
        else if (tx_flag.get_rssi_count >= 600000)
        {
            tx_flag.get_rssi_count = 0;
            tx_type = GET_RSSI;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "rssi\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.get_time_count >= 60000)
        {
            tx_flag.get_time_count = 0;
            tx_type = GET_TIME;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "time\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.connect == 1)
        {
            tx_flag.connect = 0;
            tx_type = PROP_EVENT;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "connect dm_office wuxianmima ++++++\r"); // 根据工厂路由器更改
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (tx_flag.prop_properties > 0) // 有属性需要更新
        {
            int i = 0;
            char arry[32];
            tx_type = PROP_EVENT;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "properties_changed");
            for (i = 0; i < 24; i++)
            {
                arry[0] = 0;
                if (tx_flag.prop_properties & PROP_RUN_STATE)
                {
                    tx_flag.prop_properties &= (PROP_RUN_STATE ^ 0xffffffff);
                    sprintf(arry, " 2 1 %d", sys_param.state);
                    strcat((char *)wifi_tx_buffer, arry);
                }
                else if (tx_flag.prop_properties & PROP_A_PHASE_POWER)
                {
                    tx_flag.prop_properties &= (PROP_A_PHASE_POWER ^ 0xffffffff);
                    if (fabsf(sys_param.ct1.power.fix_dir_power) < 10.0f)
                    {
                        sprintf(arry, " 2 2 %.2f", 0.0f);
                    }
                    else
                    {
                        sprintf(arry, " 2 2 %.2f", sys_param.ct1.power.fix_dir_power);
                    }
                    strcat((char *)wifi_tx_buffer, arry);
                }
                else if (tx_flag.prop_properties & PROP_B_PHASE_POWER)
                {
                    tx_flag.prop_properties &= (PROP_B_PHASE_POWER ^ 0xffffffff);
                    if (fabsf(sys_param.ct2.power.fix_dir_power) < 10.0f)
                    {
                        sprintf(arry, " 2 3 %.2f", 0.0f);
                    }
                    else
                    {
                        sprintf(arry, " 2 3 %.2f", sys_param.ct2.power.fix_dir_power);
                    }
                    strcat((char *)wifi_tx_buffer, arry);
                }
                else if (tx_flag.prop_properties & PROP_C_PHASE_POWER)
                {
                    tx_flag.prop_properties &= (PROP_C_PHASE_POWER ^ 0xffffffff);
                    if (fabsf(sys_param.ct3.power.fix_dir_power) < 10.0f)
                    {
                        sprintf(arry, " 2 4 %.2f", 0.0f);
                    }
                    else
                    {
                        sprintf(arry, " 2 4 %.2f", sys_param.ct3.power.fix_dir_power);
                    }
                    strcat((char *)wifi_tx_buffer, arry);
                }
                else
                {
                    tx_flag.prop_properties = 0;
                }
            }
            strcat((char *)wifi_tx_buffer, "\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
        }
        else if (tx_flag.get_down_count >= GET_DOWN_MS_INTERVAL) // 需要下拉状态
        {
            tx_type = GET_DOWN;
            tx_flag.get_down_count = 0;
            wifi_tx_buffer[0] = 0;
            strcat((char *)wifi_tx_buffer, "get_down\r");
            bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
            tx_flag.timeout_count = 0;
        }
        else if (wifi_info.net == CLOUD)
        {
            bool found_changed = false;
            uint8_t changed_inv_idx = 0;

            // 查找第一个有变化的微逆设备
            for (uint8_t inv_idx = 0; inv_idx < INV_DEVICE_MAX_NUM; inv_idx++)
            {
                if (sys_param.paired_inv_info[inv_idx].is_valid && (sys_param.paired_inv_info[inv_idx].prop_changed || sys_param.paired_inv_info[inv_idx].settings_changed) && sys_param.paired_inv_info[inv_idx].siid >= SIID_MIN && sys_param.paired_inv_info[inv_idx].siid <= SIID_MAX)
                {
                    found_changed = true;
                    changed_inv_idx = inv_idx;
                    break;
                }
            }

            if (found_changed)
            {
                if (sys_param.paired_inv_info[changed_inv_idx].settings_changed)
                {
                    report_inverter_set_param(changed_inv_idx);
                    sys_param.paired_inv_info[changed_inv_idx].settings_changed = false;
                }
                else if (sys_param.paired_inv_info[changed_inv_idx].prop_changed)
                {
                    report_inverter_properties(changed_inv_idx);
                    sys_param.paired_inv_info[changed_inv_idx].prop_changed = false;
                }
                // 清除变化标志
                tx_flag.timeout_count = 0;
            }
            // 如果没有变化，检查是否到了定时上报时间
            else if (tx_flag.prop_inv_count >= PROP_INV_MS_INTERVAL)
            {
                tx_flag.prop_inv_count = 0; // 重置计数器

                // 静态变量记录上次上报的设备索引
                static uint8_t last_reported_inv_idx = 0;

                // 从上次上报的下一个设备开始
                uint8_t start_idx = (last_reported_inv_idx + 1) % INV_DEVICE_MAX_NUM;
                bool reported = false;

                // 遍历所有设备，找到下一个有效设备上报
                for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
                {
                    uint8_t inv_idx = (start_idx + i) % INV_DEVICE_MAX_NUM;

                    if (sys_param.paired_inv_info[inv_idx].is_valid &&
                        sys_param.paired_inv_info[inv_idx].siid >= SIID_MIN &&
                        sys_param.paired_inv_info[inv_idx].siid <= SIID_MAX)
                    {
                        report_inverter_properties_scheduled(inv_idx);
                        last_reported_inv_idx = inv_idx;
                        reported = true;
                        tx_flag.timeout_count = 0;
                        break;
                    }
                }

                // 如果没有找到有效设备上报，重置索引
                if (!reported)
                {
                    last_reported_inv_idx = 0;
                }
            }
            // 立即上报处理：绑定SN
            if (tx_flag.immediate_report_bind)
            {
                tx_flag.immediate_report_bind = 0;
                wifi_tx_buffer[0] = 0;
                sprintf(wifi_tx_buffer, "properties_changed 3 5 \"%s\"\r", report_bind_sn);
                bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
                tx_flag.timeout_count = 0;
            }
            // 立即上报处理：解绑
            else if (tx_flag.immediate_report_unbind)
            {
                tx_flag.immediate_report_unbind = 0;
                wifi_tx_buffer[0] = 0;
                sprintf(wifi_tx_buffer, "properties_changed 3 6 \"%s\"\r", report_bind_sn);
                bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
                tx_flag.timeout_count = 0;
            }
            else if (sys_param.inv_0x51_report.valid) // 立即上报处理：0x51原始数据
            {
                uint8_t inv_idx = sys_param.inv_0x51_report.inv_index;
                uint8_t siid = sys_param.paired_inv_info[inv_idx].siid;

                wifi_tx_buffer[0] = 0;
                sprintf(wifi_tx_buffer, "properties_changed %d %d \"", siid, 36);

                // 需要将二进制数据转换为hex字符串
                char hex_str[INV_REPORT_51_SIZE * 2 + 1]; // 每字节转换为2个hex字符加上最后的结束符号
                for (uint8_t i = 0; i < sys_param.inv_0x51_report.data_len; i++)
                {
                    sprintf(hex_str + i * 2, "%02X", sys_param.inv_0x51_report.data[i]);
                }
                hex_str[sys_param.inv_0x51_report.data_len * 2] = '\0';

                strcat(wifi_tx_buffer, hex_str);
                strcat(wifi_tx_buffer, "\"\r");

                bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
                tx_flag.timeout_count = 0;

                // 清除缓存
                sys_param.inv_0x51_report.valid = 0;

                DEBUG_PRINTF("[WIFI]Report 0x51: %d %d %s\r\n", siid, sys_param.inv_0x51_report.data_len, hex_str);
            }
            // CT设备属性定时上报（5分钟周期，轮流上报siid3和siid2）
            else if (tx_flag.prop_ct_count >= PROP_CT_MS_INTERVAL)
            {
                tx_flag.prop_ct_count = 0;

                static uint8_t ct_report_toggle = 0;
                if (ct_report_toggle == 0)
                {
                    report_ct_siid3_properties();
                    ct_report_toggle = 1;
                }
                else
                {
                    report_ct_siid2_properties();
                    ct_report_toggle = 0;
                }

                tx_flag.timeout_count = 0;
            }
        }
    }
    else if (tx_flag.timeout_count >= WAIT_COMMAND_TIMEOUT_MS) // 发送消息回复超时
    {
        tx_type = WIFI_TX_NULL;
    }

    // 每2秒检查一次检查并触发slave版本上报
    if (tx_flag.version_check_count >= 2000)
    {
        tx_flag.version_check_count = 0;
        check_and_report_versions();
    }
#endif
}

void wifi_reboot(void)
{
    GPIO_ResetPins(GPIO_PORT_B, GPIO_PIN_13);
    delay_us(1000);
    GPIO_SetPins(GPIO_PORT_B, GPIO_PIN_13);
}

void wifi_timer_1ms_excute(void)
{
    tx_flag.get_down_count++;
    tx_flag.prop_ct_count++;
    tx_flag.prop_inv_count++;
    tx_flag.get_net_count++;
    tx_flag.timeout_count++;
    tx_flag.offline_count++;
    tx_flag.get_rssi_count++;
    tx_flag.ble_off_count++;
    tx_flag.get_time_count++;
    tx_flag.comm_timeout_count++;
    tx_flag.version_check_count++;
    tx_flag.slave_version_report_ms++;
    check_wifi_communication(); // 检测WiFi通信状态
}

void wifi_init(void)
{
    wifi_reboot();

    // 初始化 WiFi 信息结构体
    memset(&wifi_info, 0, sizeof(wifi_info_t));

    tx_flag.device = true;
    tx_flag.mcu_version = true;
    tx_flag.get_down_count = 0;
    tx_flag.prop_ct_count = PROP_CT_MS_INTERVAL;
    tx_flag.prop_inv_count = PROP_INV_MS_INTERVAL;
    tx_flag.immediate_report_pair = 0;
    tx_flag.immediate_report_bind = 0;
    tx_flag.immediate_report_unbind = 0;
    tx_flag.get_net_count = 2000;
    tx_flag.get_time_count = 45000;
    tx_flag.timeout_count = 0;
    tx_flag.restore = false;
    tx_flag.offline_count = 0;
    tx_flag.version = 1;
    tx_flag.mac = 1;
    tx_flag.comm_timeout_count = 0;
    tx_flag.slave_version_report_ms = 0;
}

void wifi_properties_para(char *string, wifi_msg_params_t *params_list)
{
    uint16_t i = 0;
    char *root[128];
    uint16_t num = 0;
    root[0] = (char *)string;
    for (num = 1; num < 128; num++)
    {
        root[num] = strstr(root[num - 1], " "); // 找到第1个空格
        if (root[num])                          // 找到了
        {
            *root[num] = 0; // 将空格替换成null
            root[num] += 1; // 指针偏移到null后面的字节
        }
        else // 空格已经找完
        {
            break;
        }
    }
    // root[0]-down root[1]-get_properties/set_properties root[2] ... root[n]-对应键值
    if (strstr(root[1], "get_properties")) // 这是一个读属性命令
    {
        for (i = 0; i < 32; i++) // 最大只支持32个属性同时下发
        {
            if (i * 2 + 4 > num) // 数量完了
            {
                params_list[i].flag = false;
                break;
            }
            if (*root[i * 2 + 2] >= 0x30 && *root[i * 2 + 2] <= 0x39) // SIID
                sscanf(root[i * 2 + 2], "%d", &params_list[i].siid);
            else
                break;
            if (*root[i * 2 + 3] >= 0x30 && *root[i * 2 + 3] <= 0x39) // PIID
                sscanf(root[i * 2 + 3], "%d", &params_list[i].piid);
            else
                break;
            params_list[i].flag = true;
        }
    }
    else if (strstr(root[1], "set_properties")) // 这是一个写属性命令
    {
        for (i = 0; i < 32; i++)
        {
            if (i * 3 + 5 > num) // 数量完了
            {
                params_list[i].flag = false;
                break;
            }
            if (*root[i * 3 + 2] >= 0x30 && *root[i * 3 + 2] <= 0x39) // SIID
                sscanf(root[i * 3 + 2], "%d", &params_list[i].siid);
            else
                break;
            if (*root[i * 3 + 3] >= 0x30 && *root[i * 3 + 3] <= 0x39) // PIID
                sscanf(root[i * 3 + 3], "%d", &params_list[i].piid);
            else
                break;

            if (*root[i * 3 + 4] == 't') //"true"
            {
                params_list[i].value_type = WIFI_VALUE_TYPE_TRUE;
            }
            else if (*root[i * 3 + 4] == 'f') //"false"
            {
                params_list[i].value_type = WIFI_VALUE_TYPE_FALSE;
            }
            else if (*root[i * 3 + 4] == '"') // 字符串
            {
                params_list[i].value_type = WIFI_VALUE_TYPE_STRING;
                root[i * 3 + 4] += 1;                                 // 偏移开头的"
                *(root[i * 3 + 4] + strlen(root[i * 3 + 4]) - 1) = 0; // 将结尾的"清零
                params_list[i].s_value = (char *)root[i * 3 + 4];
            }
            else // 数字
            {
                params_list[i].value_type = WIFI_VALUE_TYPE_FLOAT;
                sscanf(root[i * 3 + 4], "%f", &params_list[i].f_value);
            }
            params_list[i].flag = true;
        }
    }
}

void wifi_get_properties_reply(char *string, wifi_msg_params_t *params)
{
    char array[256];
    array[0] = 0;
    sprintf(array, " %d %d ", params->siid, params->piid);
    strcat(string, array);
    array[0] = 0;
    switch (params->value_type)
    {
    case WIFI_VALUE_TYPE_FALSE:
        strcat((char *)string, "0 false");
        break;
    case WIFI_VALUE_TYPE_TRUE:
        strcat((char *)string, "0 true");
        break;
    case WIFI_VALUE_TYPE_INT:
        sprintf(array, "0 %ld", params->value);
        strcat((char *)string, array);
        break;
    case WIFI_VALUE_TYPE_FLOAT:
        sprintf(array, "0 %.2f", params->f_value);
        strcat((char *)string, array);
        break;
    case WIFI_VALUE_TYPE_STRING:
        sprintf(array, "0 \"%s\"", params->s_value);
        strcat((char *)string, array);
        break;
    case WIFI_VALUE_TYPE_NOT_FOUND:
    default:
        strcat((char *)string, "-4003");
        break;
    }
}

uint16_t wifi_set_properties_reply(char *string, wifi_msg_params_t *params, bool enable_thr, float low_thr, float high_thr)
{
    uint16_t ret = 0;
    char array[32];
    array[0] = 0;
    if (params->value_type == WIFI_VALUE_TYPE_FALSE || params->value_type == WIFI_VALUE_TYPE_TRUE)
    {
        if (enable_thr == true && (WIFI_VALUE_TYPE_FALSE < low_thr || WIFI_VALUE_TYPE_TRUE > high_thr))
        {
            ret = 1;
        }
    }
    else if (params->value_type == WIFI_VALUE_TYPE_STRING)
    {
        int len = strlen(params->s_value);
        if (enable_thr == true && (len < low_thr || len > high_thr))
        {
            ret = 1;
        }
    }
    else if (params->value_type == WIFI_VALUE_TYPE_FLOAT)
    {
        if (enable_thr == true && (params->f_value < low_thr || params->f_value > high_thr))
        {
            ret = 1;
        }
    }
    else
    {
        ret = 1;
    }
    sprintf(array, " %d %d %d", wifi_msg_params->siid, wifi_msg_params->piid, (ret == 0 ? 0 : -4005));
    strcat((char *)string, array);
    return ret;
}

void wifi_set_properties_not_found_reply(char *string, wifi_msg_params_t *params)
{
    char array[32];
    array[0] = 0;
    sprintf(array, " %d %d -4003", params->siid, params->piid);
    strcat((char *)string, array);
}

void bytes_to_wifi(char *buffer, uint16_t len)
{
    uint16_t i = 0;
    for (i = 0; i < len; i++)
    {
        /* Wait Tx data register empty */
        USART_WriteData(CM_USART2, buffer[i]);
        while (RESET == USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY))
        {
        }
    }
}

/* INT_SRC_USART2_RI Callback. */
void USART2_Handler(void)
{
    // add your codes here
    if (USART_GetStatus(CM_USART2, USART_FLAG_RX_FULL))
    {
        wifi_rx_buffer[wifi_rx_index] = USART_ReadData(CM_USART2);

        // 接收到数据，重置WiFi通信超时计数器
        tx_flag.comm_timeout_count = 0;

        if (wifi_rx_buffer[wifi_rx_index] == '\r')
        {
            // 先在wifi_rx_buffer中添加字符串结束符
            wifi_rx_buffer[wifi_rx_index] = '\0';

            msg_flag = true;

            wifi_rx_index = 0;
        }
        else
        {
            wifi_rx_index++;
            if (wifi_rx_index >= 256)
            {
                wifi_rx_index = 0;
            }
        }
    }

    USART_ClearStatus(CM_USART2, USART_FLAG_ALL);
    __DSB(); /* Arm Errata 838869 */
}

/*---------------------------------------------------------------------------
 Name        : static void report_ct_siid2_properties(void)
 Input       : 无
 Output      : 无
 Description : 上报CT设备siid2的所有属性参数到WiFi
               包含：piid1-17的所有参数
---------------------------------------------------------------------------*/
static void report_ct_siid2_properties(void)
{
    char arry[64];

    tx_type = PROP_EVENT;
    wifi_tx_buffer[0] = 0;
    strcat((char *)wifi_tx_buffer, "properties_changed");

    // piid1: 系统状态
    sprintf(arry, " 2 1 %d", sys_param.state);
    strcat((char *)wifi_tx_buffer, arry);

    // piid2: CT1功率
    float ct1_power = sys_param.ct1.power.fix_dir_power;
    // 单相系统中,不在线的CT功率设置为0
    if (!sys_param.is_three_phase && sys_param.ct1.status.connect_status != CT_STATUS_ONLINE)
    {
        ct1_power = 0.0f;
    }
    if (fabsf(ct1_power) < 10.0f)
    {
        sprintf(arry, " 2 2 %.2f", 0.0f);
    }
    else
    {
        sprintf(arry, " 2 2 %.2f", ct1_power);
    }
    strcat((char *)wifi_tx_buffer, arry);

    // piid3: CT2功率
    float ct2_power = sys_param.ct2.power.fix_dir_power;
    // 单相系统中,不在线的CT功率设置为0
    if (!sys_param.is_three_phase && sys_param.ct2.status.connect_status != CT_STATUS_ONLINE)
    {
        ct2_power = 0.0f;
    }
    if (fabsf(ct2_power) < 10.0f)
    {
        sprintf(arry, " 2 3 %.2f", 0.0f);
    }
    else
    {
        sprintf(arry, " 2 3 %.2f", ct2_power);
    }
    strcat((char *)wifi_tx_buffer, arry);

    // piid4: CT3功率
    float ct3_power = sys_param.ct3.power.fix_dir_power;
    // 单相系统中,不在线的CT功率设置为0
    if (!sys_param.is_three_phase && sys_param.ct3.status.connect_status != CT_STATUS_ONLINE)
    {
        ct3_power = 0.0f;
    }
    if (fabsf(ct3_power) < 10.0f)
    {
        sprintf(arry, " 2 4 %.2f", 0.0f);
    }
    else
    {
        sprintf(arry, " 2 4 %.2f", ct3_power);
    }
    strcat((char *)wifi_tx_buffer, arry);

    // piid5: CT1微逆功率
    sprintf(arry, " 2 5 %.2f", sys_param.ct1.inv_power);
    strcat((char *)wifi_tx_buffer, arry);

    // piid6: CT2微逆功率
    sprintf(arry, " 2 6 %.2f", sys_param.ct2.inv_power);
    strcat((char *)wifi_tx_buffer, arry);

    // piid7: CT3微逆功率
    sprintf(arry, " 2 7 %.2f", sys_param.ct3.inv_power);
    strcat((char *)wifi_tx_buffer, arry);

    // piid8: 总微逆功率
    sprintf(arry, " 2 8 %.2f", sys_param.ct1.inv_power + sys_param.ct2.inv_power + sys_param.ct3.inv_power);
    strcat((char *)wifi_tx_buffer, arry);

    // piid9: 总使用功率
    sprintf(arry, " 2 9 %.2f", sys_param.ct1.use_power + sys_param.ct2.use_power + sys_param.ct3.use_power);
    strcat((char *)wifi_tx_buffer, arry);

    // piid10: 总电网功率
    if (fabsf(sys_param.ct1.power.fix_dir_power) < 10.0f)
    {
        sys_param.ct1.power.fix_dir_power = 0.0f;
    }
    if (fabsf(sys_param.ct2.power.fix_dir_power) < 10.0f)
    {
        sys_param.ct2.power.fix_dir_power = 0.0f;
    }
    if (fabsf(sys_param.ct3.power.fix_dir_power) < 10.0f)
    {
        sys_param.ct3.power.fix_dir_power = 0.0f;
    }
    sprintf(arry, " 2 10 %.2f", sys_param.ct1.power.fix_dir_power + sys_param.ct2.power.fix_dir_power + sys_param.ct3.power.fix_dir_power);
    strcat((char *)wifi_tx_buffer, arry);

    // piid11: 今日发电时长
    sprintf(arry, " 2 11 %.2f", sys_param.ct_today_power_time);
    strcat((char *)wifi_tx_buffer, arry);

    // piid12: 今日发电量
    sprintf(arry, " 2 12 %.2f", sys_param.ct_today_energy);
    strcat((char *)wifi_tx_buffer, arry);

    // piid13: 发电总量
    sprintf(arry, " 2 13 %d", sys_param.hmi.electricity_generation);
    strcat((char *)wifi_tx_buffer, arry);

    // piid14: 限流状态
    sprintf(arry, " 2 14 %d", sys_param.limit_state);
    strcat((char *)wifi_tx_buffer, arry);

    // piid15: 相序识别结果
    sprintf(arry, " 2 15 %d", sys_param.grid.phase_id.sequence_k);
    strcat((char *)wifi_tx_buffer, arry);

    // piid16: 电网电压
    sprintf(arry, " 2 16 %.2f", sys_param.grid.ua_vol_rms);
    strcat((char *)wifi_tx_buffer, arry);

    // piid17: 电网频率
    sprintf(arry, " 2 17 %.2f", sys_param.grid.grid_frequency);
    strcat((char *)wifi_tx_buffer, arry);

    strcat((char *)wifi_tx_buffer, "\r");
    bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
}

/*---------------------------------------------------------------------------
 Name        : static void report_ct_siid3_properties(void)
 Input       : 无
 Output      : 无
 Description : 上报CT设备siid3的属性参数到WiFi
               包含：piid3(配对列表)、piid10(工作模式)、piid11(限功率值)
---------------------------------------------------------------------------*/
static void report_ct_siid3_properties(void)
{
    char arry[200];

    tx_type = PROP_EVENT;
    wifi_tx_buffer[0] = 0;
    strcat((char *)wifi_tx_buffer, "properties_changed");

    // siid3 piid3 - 配对列表
    const char *paired_list = get_paired_device_list_string();
    sprintf(arry, " 3 3 \"%s\"", paired_list);
    strcat((char *)wifi_tx_buffer, arry);

    // siid3 piid10 - 工作模式
    sprintf(arry, " 3 10 %d", sys_param.power_work_mode);
    strcat((char *)wifi_tx_buffer, arry);

    // siid3 piid11 - 限功率值
    sprintf(arry, " 3 11 %d", sys_param.to_grid_power_limit);
    strcat((char *)wifi_tx_buffer, arry);

    // siid3 piid15 - CT的sub1g信道索引
    sprintf(arry, " 3 15 %d", sys_param.sub1g.channel_index);
    strcat((char *)wifi_tx_buffer, arry);

    strcat((char *)wifi_tx_buffer, "\r");
    bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
}

/*---------------------------------------------------------------------------
 Name        : void report_inverter_properties(uint8_t inv_idx)
 Input       : 哪一个微逆
 Output      : 无
 Description : 上报指定微逆的属性参数到WiFi（变化立即上报）
               包含：在线状态、工作状态、所在相位、环境温度、故障码
---------------------------------------------------------------------------*/
static void report_inverter_properties(uint8_t inv_idx)
{
    char arry[32];
    uint8_t siid = sys_param.paired_inv_info[inv_idx].siid;

    tx_type = PROP_EVENT;
    wifi_tx_buffer[0] = 0;
    strcat((char *)wifi_tx_buffer, "properties_changed");

    // 在线状态
    sprintf(arry, " %d %d %d", siid, INV_PIID_ONLINE_STATE, sys_param.paired_inv_info[inv_idx].online_state);
    strcat((char *)wifi_tx_buffer, arry);

    // 工作状态
    sprintf(arry, " %d %d %d", siid, INV_PIID_WORK_STATE, sys_param.paired_inv_info[inv_idx].work_state);
    strcat((char *)wifi_tx_buffer, arry);

    // 所在相位
    sprintf(arry, " %d %d %d", siid, INV_PIID_POWER_PHASE, sys_param.paired_inv_info[inv_idx].phase);
    strcat((char *)wifi_tx_buffer, arry);

    // 防逆流开关状态：0->false, 1->true
    if (sys_param.paired_inv_info[inv_idx].antiflow_enable == 0)
    {
        sprintf(arry, " %d %d false", siid, INV_PIID_ANTIFLOW_ENABLE);
    }
    else
    {
        sprintf(arry, " %d %d true", siid, INV_PIID_ANTIFLOW_ENABLE);
    }
    strcat((char *)wifi_tx_buffer, arry);

    // 发电开关状态：0->false, 1->true
    if (sys_param.paired_inv_info[inv_idx].power_enable == 0)
    {
        sprintf(arry, " %d %d false", siid, INV_PIID_INV_POWER_ENABLE);
    }
    else
    {
        sprintf(arry, " %d %d true", siid, INV_PIID_INV_POWER_ENABLE);
    }
    strcat((char *)wifi_tx_buffer, arry);

    // 故障码
    sprintf(arry, " %d %d %d", siid, INV_PIID_FAULT_PARAM, sys_param.paired_inv_info[inv_idx].fault_param);
    strcat((char *)wifi_tx_buffer, arry);

    strcat((char *)wifi_tx_buffer, "\r");
    bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
}

/*---------------------------------------------------------------------------
 Name        : void report_inverter_properties_scheduled(uint8_t inv_idx)
 Input       : 哪一个微逆
 Output      : 无
 Description : 定时上报指定微逆的发电相关参数到WiFi（仅定时上报）
               包含：发电功率、今日发电时长、今日发电量、累计发电量、PV1-4功率
---------------------------------------------------------------------------*/
static void report_inverter_properties_scheduled(uint8_t inv_idx)
{
    char arry[32];
    uint8_t siid = sys_param.paired_inv_info[inv_idx].siid;

    tx_type = PROP_EVENT;
    wifi_tx_buffer[0] = 0;
    strcat((char *)wifi_tx_buffer, "properties_changed");

    // 发电功率
    sprintf(arry, " %d %d %.2f", siid, INV_PIID_GRID_POWER, sys_param.paired_inv_info[inv_idx].grid_power);
    strcat((char *)wifi_tx_buffer, arry);

    // 今日发电时长
    sprintf(arry, " %d %d %.2f", siid, INV_PIID_TODAY_POWER_TIME, sys_param.paired_inv_info[inv_idx].today_power_time);
    strcat((char *)wifi_tx_buffer, arry);

    // 今日发电量
    sprintf(arry, " %d %d %.3f", siid, INV_PIID_TODAY_ENERGY, sys_param.paired_inv_info[inv_idx].today_energy);
    strcat((char *)wifi_tx_buffer, arry);

    // 累计发电量
    sprintf(arry, " %d %d %.3f", siid, INV_PIID_LIFETIME_ENERGY, sys_param.paired_inv_info[inv_idx].lifetime_energy);
    strcat((char *)wifi_tx_buffer, arry);

    // 上报PV功率
    for (uint8_t i = 0; i < sys_param.paired_inv_info[inv_idx].pv_num && i < 4; i++)
    {
        sprintf(arry, " %d %d %d", siid, INV_PIID_PV1_POWER + i, sys_param.paired_inv_info[inv_idx].pv[i].power);
        strcat((char *)wifi_tx_buffer, arry);
    }

    // 环境温度
    sprintf(arry, " %d %d %.3f", siid, INV_PIID_TEMPERATURE, sys_param.paired_inv_info[inv_idx].ambient_temperature);
    strcat((char *)wifi_tx_buffer, arry);

    // 丢包率
    sprintf(arry, " %d %d %d", siid, INV_PIID_PACKET_LOSS_RATE, sys_param.paired_inv_info[inv_idx].plr);
    strcat((char *)wifi_tx_buffer, arry);

    strcat((char *)wifi_tx_buffer, "\r");
    bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
}

/*---------------------------------------------------------------------------
 Name        : void report_inverter_set_param(uint8_t inv_idx)
 Input       : 哪一个微逆
 Output      : 无
 Description : 上报指定微逆的设置参数到WiFi
               包含：接入点、功率限制、版本、型号、防逆流开关、发电开关
               这些参数变化频率低，仅在接收到0x53上报且参数变化时调用
---------------------------------------------------------------------------*/
void report_inverter_set_param(uint8_t inv_idx)
{
    char arry[64];
    uint8_t siid = sys_param.paired_inv_info[inv_idx].siid;

    tx_type = PROP_EVENT;
    wifi_tx_buffer[0] = 0;
    strcat((char *)wifi_tx_buffer, "properties_changed");

    // 接入点
    sprintf(arry, " %d %d %d", siid, INV_PIID_CONNECTION_POINT, sys_param.paired_inv_info[inv_idx].connection_point);
    strcat((char *)wifi_tx_buffer, arry);

    // 功率限制
    sprintf(arry, " %d %d %d", siid, INV_PIID_POWER_LIMIT, sys_param.paired_inv_info[inv_idx].power_limit);
    strcat((char *)wifi_tx_buffer, arry);

    // // MCU软件版本
    // sprintf(arry, " %d %d \"%s\"", siid, INV_PIID_SW_VERSION, sys_param.paired_inv_info[inv_idx].sw_version);
    // strcat((char *)wifi_tx_buffer, arry);

    // // Sub软件版本
    // sprintf(arry, " %d %d \"%s\"", siid, INV_PIID_SUB1G_VERSION, sys_param.paired_inv_info[inv_idx].sw_version);
    // strcat((char *)wifi_tx_buffer, arry);

    // 产品型号
    sprintf(arry, " %d %d \"%s\"", siid, INV_PIID_PRODUCT_MODEL, sys_param.paired_inv_info[inv_idx].product_model);
    strcat((char *)wifi_tx_buffer, arry);

    // 信道索引
    sprintf(arry, " %d %d %d", siid, INV_PIID_CHANNEL_INDEX, sys_param.paired_inv_info[inv_idx].channel_index);
    strcat((char *)wifi_tx_buffer, arry);

    strcat((char *)wifi_tx_buffer, "\r");
    bytes_to_wifi(wifi_tx_buffer, strlen((char *)wifi_tx_buffer));
}

/*---------------------------------------------------------------------------
 Name        : void update_slave_versions(void)
 Input       : 无
 Output      : 无
 Description : 更新slave设备版本信息，找出各类型最旧的版本号
               类型4: 微逆的Sub1G版本（最旧）
               类型5: 800W微逆MCU版本（最旧，产品型号GE-MI800S）
               类型6: 2500W微逆MCU版本（最旧，产品型号GE-MI2500S）
---------------------------------------------------------------------------*/
void update_slave_versions(void)
{
    // 初始化最旧版本为空
    sys_param.slave_version.inv_sub1g_version[0] = '\0';
    sys_param.slave_version.inv_800w_version[0] = '\0';
    sys_param.slave_version.inv_2500w_version[0] = '\0';

    // 遍历所有已配对设备
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (!sys_param.paired_inv_info[i].is_valid)
            continue;

        DEBUG_PRINTF(" - Dev[0x%06X]: Model=%s, Version: SW=%s, Sub1g=%s \r\n",
                     sys_param.paired_inv_info[i].sub1g_addr,
                     sys_param.paired_inv_info[i].product_model,
                     sys_param.paired_inv_info[i].sw_version,
                     sys_param.paired_inv_info[i].sub1g_version);

        // 检查Sub1G版本（类型4）
        if (sys_param.paired_inv_info[i].sub1g_version[0] != '\0' && sys_param.paired_inv_info[i].online_state == CT_STATUS_ONLINE)
        {
            if (sys_param.slave_version.inv_sub1g_version[0] == '\0' ||
                strcmp(sys_param.paired_inv_info[i].sub1g_version, sys_param.slave_version.inv_sub1g_version) < 0)
            {
                strncpy(sys_param.slave_version.inv_sub1g_version, sys_param.paired_inv_info[i].sub1g_version, VERSION_STRING_MAX_LEN);
                sys_param.slave_version.inv_sub1g_version[VERSION_STRING_MAX_LEN] = '\0';
            }
        }

        // 检查MCU版本（类型5/6）- 根据产品型号区分
        if (sys_param.paired_inv_info[i].sw_version[0] != '\0')
        {
            // 判断是800W还是2500W
            if (strcmp(sys_param.paired_inv_info[i].product_model, "GE-MI800S") == 0)
            {
                // 800W微逆（类型5）
                if (sys_param.slave_version.inv_800w_version[0] == '\0' ||
                    strcmp(sys_param.paired_inv_info[i].sw_version, sys_param.slave_version.inv_800w_version) < 0)
                {
                    strncpy(sys_param.slave_version.inv_800w_version,
                            sys_param.paired_inv_info[i].sw_version, VERSION_STRING_MAX_LEN);
                    sys_param.slave_version.inv_800w_version[VERSION_STRING_MAX_LEN] = '\0';
                }
            }
            else if (strcmp(sys_param.paired_inv_info[i].product_model, "GE-MI2500S") == 0)
            {
                // 2500W微逆（类型6）
                if (sys_param.slave_version.inv_2500w_version[0] == '\0' ||
                    strcmp(sys_param.paired_inv_info[i].sw_version, sys_param.slave_version.inv_2500w_version) < 0)
                {
                    strncpy(sys_param.slave_version.inv_2500w_version,
                            sys_param.paired_inv_info[i].sw_version, VERSION_STRING_MAX_LEN);
                    sys_param.slave_version.inv_2500w_version[VERSION_STRING_MAX_LEN] = '\0';
                }
            }
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : void check_and_report_versions(void)
 Input       : 无
 Output      : 无
 Description : 检查版本是否收集完成并触发上报，允许OTA
               - 每1小时定时上报一次slave版本
               - 如果没有绑定设备，只需CT Sub1G版本即可上报
               - 如果有绑定设备，需要CT Sub1G版本以及所有绑定设备的版本（sw_version和sub1g_version）
---------------------------------------------------------------------------*/
static void check_and_report_versions(void)
{
    // 检查是否到达1小时上报周期
    if (tx_flag.slave_version_report_ms < 3600000)
    {
        // 如果是第一次上报（未上报过），立即检查
        if (sys_param.slave_version.slave_version_reported)
        {
            return; // 已经上报过且未到1小时，不执行上报
        }
    }

    // 检查CT Sub1G版本是否已获取
    if (sys_param.sub1g.sw_version[0] == '\0')
    {
        DEBUG_PRINTF("[Version] CT Sub1G version has not been collected: %s\r\n", sys_param.sub1g.sw_version);
        return;
    }

    // 检查是否有微逆已在线的设备
    bool has_online_device = false;
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].product_model[0] != '\0' && sys_param.paired_inv_info[i].online_state == CT_STATUS_ONLINE)
        {
            has_online_device = true;
            break;
        }
    }

    // 如果没有在线设备，只要CT Sub1G版本有效，就可以上报
    if (!has_online_device)
    {
        DEBUG_PRINTF("[Version] No online device. CT Sub1G version collected: %s\r\n", sys_param.sub1g.sw_version);
        tx_flag.slave_version = true;
        return;
    }

    // 有在线设备，需要等待所有在线设备的版本都已收集
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].online_state == CT_STATUS_ONLINE)
        {
            if (sys_param.paired_inv_info[i].sw_version[0] == '\0' || sys_param.paired_inv_info[i].sub1g_version[0] == '\0')
            {
                return; // 有设备版本未收集完成
            }
        }
    }

    // 所有版本已收集完成，更新并触发上报
    DEBUG_PRINTF("[Version] All online device version collected, ready report to WIFI\r\n");
    update_slave_versions();
    tx_flag.slave_version = true;
}

/*---------------------------------------------------------------------------
 Name        : static void update_power_date_check(void)
 Input       : 无
 Output      : 无
 Description : 检查日期是否变更，如果变更则清零所有今日统计参数
               - 连续5次检测到日期不匹配才执行清零操作(避免误触发)
               - 清零CT层面的today_power_time和today_energy
               - 清零所有微逆设备的today_power_time和today_energy
---------------------------------------------------------------------------*/
static void update_power_date_check(void)
{
    static int date_mismatch_count = 0; // 用于跟踪日期不匹配的连续次数

    if (strlen(sys_param.time.date) >= 10) // "YYYY-MM-DD"格式至少有10个字符
    {
        // 从日期字符串中提取日号
        char day_str[3];
        day_str[0] = sys_param.time.date[8];
        day_str[1] = sys_param.time.date[9];
        day_str[2] = '\0';
        uint8_t current_day = (uint8_t)atoi(day_str);

        // 检查日号是否有效（1-31）且与存储的日号不同
        if (current_day >= 1 && current_day <= 31 && sys_param.time.today_date != current_day)
        {
            date_mismatch_count++; // 增加不匹配计数

            // 只有在连续多次检测到不匹配时才执行清零操作
            if (date_mismatch_count >= 2)
            {
                DEBUG_PRINTF("[Date] Date changed: %d -> %d, Clear today's statistics\r\n", sys_param.time.today_date, current_day);

                // 新的一天,清零CT层面的今日统计参数
                sys_param.ct_today_power_time = 0.0f;
                sys_param.ct_today_energy = 0.0f;

                // 清零所有微逆设备的今日统计参数
                for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
                {
                    if (sys_param.paired_inv_info[i].is_valid)
                    {
                        sys_param.paired_inv_info[i].today_power_time = 0.0f;
                        sys_param.paired_inv_info[i].today_energy = 0.0f;
                    }
                }

                // 更新存储的日号
                sys_param.time.today_date = current_day;

                DEBUG_PRINTF("[Date] Today's statistics reset to 0. New date: %d\r\n", current_day);

                // 重置不匹配计数
                date_mismatch_count = 0;
            }
        }
        else
        {
            if (date_mismatch_count > 0)
            {
                date_mismatch_count = 0;
            }
        }
    }
    else
    {
        if (date_mismatch_count > 0)
        {
            date_mismatch_count = 0;
        }
    }
}

/*---------------------------------------------------------------------------
Name        : void check_wifi_communication(void)
Input       : None
Output      : None
Description : 检测WiFi通信状态。如果3分钟内没有数据传输，则重启WiFi。
              通过监控串口接收中断来判断通信活动。
---------------------------------------------------------------------------*/
void check_wifi_communication(void)
{
    // 检查是否达到3分钟超时时间
    if (tx_flag.comm_timeout_count >= WIFI_COMM_3MIN_TIMEOUT)
    {
        tx_flag.comm_timeout_count = 0; // 重置超时计数器

        wifi_init(); // 重启WiFi并重新初始化WiFi参数
    }
}
