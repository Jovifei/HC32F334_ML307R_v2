/*
 * mmi.h
 *
 *  Created on: 2025年7月2日
 *      Author: yao
 */

#ifndef APP_INC_MMI_H_
#define APP_INC_MMI_H_

#include "main.h"

#define MSG_DATA_MAX_LEN 100 // 内存不够，暂定512改为400

typedef struct
{
    uint8_t flag;                       // 消息标志
    uint8_t type;                       // 0x00-请求，0x01回复
    uint16_t cmd;                       // 15位
    uint16_t code;                      // 0x8000-0x8010
    uint16_t cmd_data_length;           // 命令长度
    uint8_t cmd_data[MSG_DATA_MAX_LEN]; // 命令数据
} serial_msg_t;

void mmi_task(void);

// LED????
void led_state_machine_update(mmi_t *mmi);
void hmi_update_all_params(void);
void serial_msg_parse(uint8_t byte, serial_msg_t *msg_in);
void serial_msg_send(serial_msg_t *msg_out);

extern void serial_msg_send(serial_msg_t *msg_out);

extern serial_msg_t msg_output;

#endif /* APP_INC_MMI_H_ */
