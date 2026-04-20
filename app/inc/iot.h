#ifndef IOT_H
#define IOT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief IoT transmit flags for tracking report states
 */
typedef struct {
    uint32_t prop_ct_count;          // CT定时上报计数器(ms)
    uint32_t prop_inv_count;        // INV定时上报计数器(ms)
    uint8_t immediate_report_bind;   // 绑定立即上报标志
    uint8_t immediate_report_unbind; // 解绑立即上报标志
} iot_tx_flag_t;

extern iot_tx_flag_t iot_tx_flag;

/**
 * @brief IoT task initialization (called by ml307r after MQTT connected)
 */
void iot_task(void);

/**
 * @brief MQTT downlink handler callback (called by ml307r)
 */
void iot_mqtt_downlink_handler(const char *topic, const char *payload);

/**
 * @brief Immediate property report - INV (called by other modules)
 */
void iot_report_immediate(uint8_t inv_idx);

/**
 * @brief Immediate property report - Bind (called by other modules)
 */
void iot_report_bind(const char *sn);

/**
 * @brief Immediate property report - Unbind (called by other modules)
 */
void iot_report_unbind(const char *sn);

/**
 * @brief Trigger CT periodic property report (called by ml307r_task or main)
 */
void iot_trigger_ct_report(void);

/**
 * @brief Trigger INV periodic property report (called by ml307r_task or main)
 */
void iot_trigger_inverter_report(uint8_t inv_idx);

/**
 * @brief Get IoT module connection status
 */
bool iot_is_connected(void);

#endif /* IOT_H */
