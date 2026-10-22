/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 主函数源文件
 * @version 0.1
 * @date 2026-10-22
 * 
 * @copyright Copyright (c) 2026
 * 
 * */

#include "main.h"


/** 时间结构体, 用于存储系统时间 */
typedef struct
{
    uint32_t        cnt0;
    uint32_t        cnt1;
    uint32_t        cnt2;
    uint8_t         mS;
    uint8_t         SS;
    uint8_t         MM;
    uint8_t         HH;
} stc_time_t;           //size: 16 bytes(4+4+4+1+1+1+1)

static volatile stc_time_t  m_stcTime = {0U, 0U, 0U, 0U, 0U, 0U, 0U};



/**
 * @brief  Main function.
 * @retval None
 */
int main(void)
{
    static uint8_t  u8LastSec = 0xFFU;
    char            UartStr[64];

    /* 初始化系统时钟和外设 */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    Delay_ms(1000U);                // 等待系统时钟稳定
    App_USARTxCfg();                // 初始化USART1
    App_PortCfg();                  // 初始化GPIO端口
    App_Tmra2Cfg();                 // 初始化TMRA2
    __enable_irq();                 // 使能中断总线

    debug_output();                 // 输出系统信息

    PSFB_BSP_Init();                // 初始化PSFB_HC


    /* 主循环 */
    for (;;)
    {
        if (m_stcTime.cnt0 >= 1000U)
        {
            m_stcTime.cnt0 = 0U;
            GPIO_TogglePins(GPIO_PORT_A, GPIO_PIN_02);
        }
        if (m_stcTime.SS != u8LastSec)
        {
            u8LastSec = (uint8_t)m_stcTime.SS;
            snprintf(UartStr, sizeof(UartStr), "LED blink\nTime: %02d:%02d:%02d\r\n", m_stcTime.HH, m_stcTime.MM, m_stcTime.SS);
            Log_SendString(UartStr);
        }
    }
}

/***********************************************************************************************************/

/**
 * @brief  Send string to USART1.
 * @param  pcStr: String to be sent.
 * @retval None
 * @note   可用snprintf()格式化字符串，再调用该函数发送
 */
static void Log_SendString(const char *pcStr)
{
    while (*pcStr != '\0')
    {
        while (RESET == USART_GetStatus(CM_USART1, USART_FLAG_TX_EMPTY)){;}

        if (USART_GetStatus(CM_USART1, USART_FLAG_TX_EMPTY) != RESET)
            USART_WriteData(CM_USART1, (uint16_t)(*pcStr));

        pcStr++;
    }
}

/**
 * @brief  Delay milliseconds.
 * @param  u32Ms: Delay time in milliseconds.
 * @retval None
 */
void Delay_ms(volatile uint32_t u32Ms)
{
    for (volatile uint32_t i = 0U; i < u32Ms; i++)
        for (volatile uint32_t j = 0U; j < 1000U; j++)
            __NOP();

}

/**
 * @brief  Debug output system information.
 * @retval None
 */
static void debug_output(void)
{
    char    UadebugData[64];

    Log_SendString("***************************************************************\r\n");
    /* 系统时钟输出 */
    snprintf(UadebugData, sizeof(UadebugData), "System Build time: %s %s\r\n", __DATE__, __TIME__);
    Log_SendString(UadebugData);
    snprintf(UadebugData, sizeof(UadebugData), "SystemCoreClock = %d MHz\r\n", (int)(SystemCoreClock / 1000000U));
    Log_SendString(UadebugData);

    /* 确认系统时钟源和 PLL 源 */
    const char *const ClkSource[8] = {"HRC",  "MRC", "LRC", "XTAL", "XTAL32", "PLLH", "RSV", "RSV"};
    const char *const PllSource[3] = {"XTAL", "HRC", "RSV"};

    snprintf(UadebugData,
            sizeof(UadebugData),
            "SystemClockSource = %s, PLLSource = %s\r\n",
            ClkSource[(uint8_t)(CM_CMU->CKSWR & CMU_CKSWR_CKSW)],
            PllSource[(uint8_t)((CM_CMU->PLLHCFGR & CMU_PLLHCFGR_PLLSRC) >> CMU_PLLHCFGR_PLLSRC_POS)]);
    Log_SendString(UadebugData);
    Log_SendString("***************************************************************\r\n\r\n");

}


/**
 * @brief  GPIO port configuration.
 * @retval None
 */
static void App_PortCfg(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState            = PIN_SET;
    stcGpioInit.u16PinDir              = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr             = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_02, &stcGpioInit);

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState            = PIN_SET;
    stcGpioInit.u16PinDir              = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr             = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_12, &stcGpioInit);

    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_12,  GPIO_FUNC_32);
    GPIO_SubFuncCmd(GPIO_PORT_A, GPIO_PIN_12, ENABLE);
    GPIO_SetSubFunc(GPIO_FUNC_32);

}


/**
 * @brief  TMRA_2 configuration.
 * @retval None
 */
static void App_Tmra2Cfg(void)
{
    stc_tmra_init_t stcTmraInit;

    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_2, ENABLE);

    (void)TMRA_StructInit(&stcTmraInit);
    stcTmraInit.u8CountSrc              = TMRA_CNT_SRC_SW;
    stcTmraInit.sw_count.u8ClockDiv     = TMRA_CLK_DIV16;
    stcTmraInit.sw_count.u8CountMode    = TMRA_MD_SAWTOOTH;
    stcTmraInit.sw_count.u8CountDir     = TMRA_DIR_UP;
    stcTmraInit.u32PeriodValue          = 7499U;
    stcTmraInit.u8CountReload           = TMRA_CNT_RELOAD_ENABLE;
    (void)TMRA_Init(CM_TMRA_2, &stcTmraInit);

    TMRA_IntCmd(CM_TMRA_2, TMRA_INT_OVF, ENABLE);

    INTC_IntSrcCmd(INT_SRC_TMRA_2_OVF, ENABLE);

    NVIC_ClearPendingIRQ(TMRA_2_OVF_UDF_IRQn);
    NVIC_SetPriority(TMRA_2_OVF_UDF_IRQn, DDL_IRQ_PRIO_00);
    NVIC_EnableIRQ(TMRA_2_OVF_UDF_IRQn);

    TMRA_Start(CM_TMRA_2);
}


/**
 * @brief  USART configuration.
 * @retval None
 */
static void App_USARTxCfg(void)
{
    stc_usart_uart_init_t stcUartInit;

    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART1, ENABLE);

    (void)USART_DeInit(CM_USART1);
    (void)USART_UART_StructInit(&stcUartInit);

    stcUartInit.u32ClockSrc              = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv              = USART_CLK_DIV1;
    stcUartInit.u32CKOutput              = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate              = 115200UL;
    stcUartInit.u32DataWidth             = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit               = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity                = USART_PARITY_NONE;   
    stcUartInit.u32OverSampleBit         = USART_OVER_SAMPLE_8BIT;
    stcUartInit.u32FirstBit              = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity      = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl         = USART_HW_FLOWCTRL_NONE;
    if (LL_OK != USART_UART_Init(CM_USART1, &stcUartInit, NULL))
    {
        for (;;)  {;}
    }
    USART_ClearStatus(CM_USART1, USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR);
    (void)USART_ReadData(CM_USART1);
    (void)USART_ReadData(CM_USART1);
    
    USART_FuncCmd(CM_USART1, USART_TX, ENABLE);

}


/**
 * @brief  TMRA_2 overflowVF handler.
 * @retval None
 */
void TMRA_2_Ovf_Udf_Handler(void)
{
    if (SET == TMRA_GetStatus(CM_TMRA_2, TMRA_FLAG_OVF))
    {
        TMRA_ClearStatus(CM_TMRA_2, TMRA_FLAG_OVF);
        m_stcTime.cnt0++;
        m_stcTime.cnt1++;
        m_stcTime.cnt2++;

        m_stcTime.mS  =  m_stcTime.cnt1 % 100U;
        m_stcTime.SS  = (m_stcTime.cnt1 / 1000U) % 60U;
        m_stcTime.MM  = (m_stcTime.cnt1 / 60000U) % 60U;
        m_stcTime.HH  =  m_stcTime.cnt1 / 3600000U;
    }
}


/********************** end of file **********************/