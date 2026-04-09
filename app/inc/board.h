/*lic*/
#ifndef __BOARD_H__
#define __BOARD_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll.h"
#include "hc32_ll_aos.h"
#include "hc32_ll_clk.h"
// #include "hc32_ll_dma.h"
#include "hc32_ll_efm.h"
#include "hc32_ll_fcg.h"
// #include "hc32_ll_fcm.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_tmr6.h"
#include "hc32_ll_i2c.h"
#include "hc32_ll_interrupts.h"
#include "hc32_ll_pwc.h"
#include "hc32_ll_spi.h"
#include "hc32_ll_sram.h"
#include "hc32_ll_usart.h"
#include "hc32_ll_utility.h"

void board_init(void);

extern uint8_t spi_dma_finish_flag;

#endif /* __BOARD_H__ */

/*eof*/
