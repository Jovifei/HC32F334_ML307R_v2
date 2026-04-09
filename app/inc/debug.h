/*
 * debug.h
 *
 *  Created on: 2025Äê7ÔÂ2ÈÕ
 *      Author: yao
 */

#ifndef APP_INC_DEBUG_H_
#define APP_INC_DEBUG_H_

#include "hc32_ll.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>

void debug_task(void);
void debug_send_bytes(uint8_t *buffer, uint8_t len);
uint16_t KLPF_Function_Float(uint16_t inputValue, float LPFK, uint8_t channelIndex);
void boot_logo_print(void);
void debug_sn_task(void); // ????????SN

#endif /* APP_INC_DEBUG_H_ */
