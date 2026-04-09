/**
 *******************************************************************************
 * @file  main.c
 * @brief Main program.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2025-09-23       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "board.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/* Configures ADC. */
static void App_ADCCfg(void);
/* Configures Timer0. */
static void App_Timer0Cfg(void);
/* Configures USARTx. */
static void App_USARTxCfg(void);
/* Configures I2C. */
static void App_I2CCfg(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
// Clock Config
static void App_ClkCfg(void)
{
    /* Peripheral registers write unprotected */
    LL_PERIPH_WE(LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);
    EFM_FWMC_Cmd(ENABLE);
    EFM_SequenceSectorOperateCmd(EFM_START_ADDR, 32, ENABLE);

    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 |
                                      CLK_PCLK2_DIV8 | CLK_PCLK3_DIV8 | CLK_PCLK4_DIV2));
    /* flash read wait cycle setting */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE2);
    /* XTAL config */
    stc_clock_xtal_init_t stcXtalInit;
    (void)CLK_XtalStructInit(&stcXtalInit);
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_HIGH;
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);
    /* PLLH config */
    stc_clock_pll_init_t stcPLLHInit;
    (void)CLK_PLLStructInit(&stcPLLHInit);
    stcPLLHInit.PLLCFGR = 0UL;
    stcPLLHInit.PLLCFGR_f.PLLM = (1UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLN = (60UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLP = (4UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLQ = (4UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLR = (4UL - 1UL);
    stcPLLHInit.u8PLLState = CLK_PLL_ON;
    stcPLLHInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcPLLHInit);
    /* 2 cycles for 100MHz ~ 120MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT2);
    /* Set the system clock source */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);

    /* Reset cache ram */
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    /* Enable cache */
    EFM_PrefetchCmd(ENABLE);
    EFM_DCacheCmd(ENABLE);
    EFM_ICacheCmd(ENABLE);
}

// Port Config
static void App_PortCfg(void)
{
    /* GPIO initialize */
    stc_gpio_init_t stcGpioInit;
    /* PA0 set to ADC1-IN0 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_00, &stcGpioInit);

    /* PA1 set to ADC1-IN1 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_01, &stcGpioInit);

    /* PA2 set to ADC1-IN2 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_02, &stcGpioInit);

    /* PA3 set to ADC1-IN3 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_03, &stcGpioInit);

    /* PA4 set to ADC12-IN4 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_04, &stcGpioInit);

    /* PB3 set to GPIO-Output */
    GPIO_SetDebugPort(GPIO_PIN_TDO, DISABLE);
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_03, &stcGpioInit); // subg pb5
    GPIO_SetPins(GPIO_PORT_B, GPIO_PIN_03);

    /* PB15 set to GPIO-Output */
    GPIO_SetDebugPort(GPIO_PIN_TDI, DISABLE);
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_15, &stcGpioInit); // subg reset
    GPIO_SetPins(GPIO_PORT_A, GPIO_PIN_15);

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_F, GPIO_PIN_02, &stcGpioInit); // LED GREEN
    GPIO_ResetPins(GPIO_PORT_F, GPIO_PIN_02);

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_C, GPIO_PIN_13, &stcGpioInit); // LED RED
    GPIO_ResetPins(GPIO_PORT_C, GPIO_PIN_13);

    /* PB13 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_13, &stcGpioInit); // WiFi reset
    GPIO_SetPins(GPIO_PORT_B, GPIO_PIN_13);

    GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);
    (void)GPIO_StructInit(&stcGpioInit);
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_04, &stcGpioInit);
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_05, &stcGpioInit);

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_04, GPIO_FUNC_53); // I2C1-SCL

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_05, GPIO_FUNC_52); // I2C1-SDA

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_32); // USART1-TX

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_33); // USART1-RX

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_14, GPIO_FUNC_36); // USART2-TX

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_15, GPIO_FUNC_37); // USART2-RX

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_40); // USART3-TX

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_41); // USART3-RX

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_08, GPIO_FUNC_44); // USART4-TX

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_09, GPIO_FUNC_45); // USART4-RX
}

// Int Config
static void App_IntCfg(void)
{
    /* NVIC config */
    NVIC_ClearPendingIRQ(ADC1_IRQn);
    NVIC_SetPriority(ADC1_IRQn, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(ADC1_IRQn);

    /* NVIC config */
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(USART1_IRQn);

    /* NVIC config */
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, DDL_IRQ_PRIO_12);
    NVIC_EnableIRQ(USART2_IRQn);

    /* NVIC config */
    NVIC_ClearPendingIRQ(USART3_IRQn);
    NVIC_SetPriority(USART3_IRQn, DDL_IRQ_PRIO_15);
    NVIC_EnableIRQ(USART3_IRQn);

    /* NVIC config */
    NVIC_ClearPendingIRQ(USART4_IRQn);
    NVIC_SetPriority(USART4_IRQn, DDL_IRQ_PRIO_14);
    NVIC_EnableIRQ(USART4_IRQn);
}

/**
 * @brief  Main function of the project
 * @param  None
 * @retval int32_t return value, if needed
 */
void board_init(void)
{
    /* Register write unprotected for some required peripherals. */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    // Clock Config
    App_ClkCfg();
    // Port Config
    App_PortCfg();
    // Int Config
    App_IntCfg();
    // ADC Config
    App_ADCCfg();
    // Timer0 Config
    App_Timer0Cfg();
    // USARTx Config
    App_USARTxCfg();
    // I2C Config
    App_I2CCfg();
    /* Register write protected for some required peripherals. */
    LL_PERIPH_WP(LL_PERIPH_FCG);
    (void)SysTick_Init(1000U);
}

// ADC Config
static void App_ADCCfg(void)
{
    // independent mode
    // ADC1 config
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC1 peripheral clock. */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);

    /* 2. Modify the default value depends on the application. */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign = ADC_DATAALIGN_RIGHT;

    /* 3. Initializes ADC. */
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    /* 4. ADC sequence configuration. */
    /* Config trigger event source if needed*/
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_TMR0_1_CMP_A); // remember to config the source first
    // AOS_SetTriggerEventSrc(AOS_ADC1_1, EVT_SRC_PORT_EIRQ0);//remember to config the source first
    /* ADC sequence A configuration. */
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH0, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH1, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH2, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH3, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH4, ENABLE);

    ADC_TriggerConfig(CM_ADC1, ADC_SEQ_A, ADC_HARDTRIG_EVT0);
    ADC_TriggerCmd(CM_ADC1, ADC_SEQ_A, ENABLE);
    /* ADC Int configuration */
    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, ENABLE);
}

// Timer0 Config
static void App_Timer0Cfg(void)
{
    stc_tmr0_init_t stcTmr0Init;

    /* Enable AOS clock */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
    /* Timer0 trigger event set */
    AOS_SetTriggerEventSrc(AOS_TMR0, EVT_SRC_TMR0_1_CMP_A);

    /* Enable timer0_1 clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE);

    /************************* Configure TMR0_1_A***************************/
    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;
    stcTmr0Init.u32Func = TMR0_FUNC_CMP;
    stcTmr0Init.u16CompareValue = 0x177U;
    (void)TMR0_Init(CM_TMR0_1, TMR0_CH_A, &stcTmr0Init);
    TMR0_HWStartCondCmd(CM_TMR0_1, TMR0_CH_A, ENABLE);
    TMR0_HWClearCondCmd(CM_TMR0_1, TMR0_CH_A, ENABLE);
    /* TMR0 start counting */
    TMR0_Start(CM_TMR0_1, TMR0_CH_A);
}

// USARTx Config
static void App_USARTxCfg(void)
{
    stc_usart_uart_init_t stcUartInit;

    /* Enable USART1 clock */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART1, ENABLE);
    /************************* Configure USART1***************************/
    (void)USART_DeInit(CM_USART1);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_NONE;
    (void)USART_UART_Init(CM_USART1, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX | USART_INT_RX function */
    USART_FuncCmd(CM_USART1, (USART_TX | USART_RX | USART_INT_RX), ENABLE);

    /* Enable USART2 clock */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART2, ENABLE);
    /************************* Configure USART2***************************/
    (void)USART_DeInit(CM_USART2);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_NONE;
    (void)USART_UART_Init(CM_USART2, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX | USART_INT_RX function */
    USART_FuncCmd(CM_USART2, (USART_TX | USART_RX | USART_INT_RX), ENABLE);

    /* Enable USART3 clock */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART3, ENABLE);
    /************************* Configure USART3***************************/
    (void)USART_DeInit(CM_USART3);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_NONE;
    (void)USART_UART_Init(CM_USART3, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX | USART_INT_RX function */
    USART_FuncCmd(CM_USART3, (USART_TX | USART_RX | USART_INT_RX), ENABLE);

    /* Enable USART4 clock */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART4, ENABLE);
    /************************* Configure USART4***************************/
    (void)USART_DeInit(CM_USART4);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_NONE;
    (void)USART_UART_Init(CM_USART4, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX | USART_INT_RX function */
    USART_FuncCmd(CM_USART4, (USART_TX | USART_RX | USART_INT_RX), ENABLE);
}

// I2C Config
static void App_I2CCfg(void)
{
    int32_t i32Ret;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    /* Enable I2C clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_I2C, ENABLE);
    /************************* Configure I2C***************************/
    (void)I2C_DeInit(CM_I2C);

    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV16;
    stcI2cInit.u32Baudrate = 100000UL;
    stcI2cInit.u32SclTime = 0UL;
    i32Ret = I2C_Init(CM_I2C, &stcI2cInit, &fErr);
    if (LL_OK != i32Ret)
    {
        // Initialized failed, add your code here.
    }
    I2C_BusWaitCmd(CM_I2C, ENABLE);
    I2C_Cmd(CM_I2C, ENABLE);
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
