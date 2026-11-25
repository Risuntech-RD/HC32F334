/******************************************************************************
 * @file     psfb.c
 * @brief    移相全桥(PSFB)驱动初始化 —— HC32F334 + EG3013S + HYG026N10NS1C2
 *           HRPWM2(主) + HRPWM3(从) 硬件互补 + 相位偏移 + ADC触发 + DMA搬运
 *
 * 硬件连接 (HC32F334JATI LQ48):
 *   PA8  (HRPWM2_PWMA) -> PWM1P -> U4 HIN -> HO -> R14(10R) -> Q1  (左桥高)
 *   PA9  (HRPWM2_PWMB) -> PWM1N -> U4 LIN -> LO -> R15(10R) -> Q2  (左桥低)
 *   PA10 (HRPWM3_PWMA) -> PWM2P -> U5 HIN -> HO -> R16(10R) -> Q3  (右桥高)
 *   PA11 (HRPWM3_PWMB) -> PWM2N -> U5 LIN -> LO -> R17(10R) -> Q4  (右桥低)
 *
 * 移相全桥工作原理:
 *   HRPWM1: 50%占空比互补PWM, 产生同步脉冲给HRPWM2/3, PHSCMP1~5
 *   HRPWM2(左桥): 50%占空比互补PWM, 相位=0, (HRPWM_PH_MATCH_IDX1)
 *   HRPWM3(右桥): 50%占空比互补PWM, 相位=phase(0~PERIOD, 对应0°~180°), (HRPWM_PH_MATCH_IDX2)
 *   phase=0       -> 左右桥同相, 变压器电压最大
 *   phase=PERIOD  -> 左右桥反相, 变压器电压为零
 *   死区由 HRPWM 硬件死区发生器保证, 无需软件干预
 *
 * 数据流 (采样):
 *   HRPWM2 周期事件 -> ADC内部触发 -> ADC转换 -> EOC
 *   -> DMA硬件请求 -> DMA写入 isenseDmaBuf[](循环回绕)
 ******************************************************************************/

#include "adc.h"


/*=========================== 全局变量 ===========================*/

static uint16_t     isenseBuf[ISENSE_FILTER_WINDOW] = {0};      // 采样数据滤波缓冲区
static uint8_t      isenseIdx       = 0;                        // 采样数据索引
static float        isenseLatestA   = 0.0f;                     // 最新采样数据

volatile uint16_t   isenseDmaBuf[ISENSE_DMA_BUF_DEPTH] = {0};   // DMA搬运缓冲区



/*======================= 电流采样 (ADC+DMA) =======================*/
/**
 * @brief  初始化电流采样 GPIO
 * @param  None
 * @retval None
 */
static void ISENSE_GPIO_Init(void)
{
    stc_gpio_init_t stcGpio;

    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinAttr = PIN_ATTR_ANALOG;
    GPIO_Init(ISENSE_ADC_PORT, ISENSE_ADC_PIN, &stcGpio);
}

/**
 * @brief  初始化电流采样 ADC
 * @param  None
 * @retval None
 */
static void ISENSE_ADC_Init(void)
{
    stc_adc_init_t stcAdcInit;

    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode      = ADC_MD_SEQA_CONT;
    stcAdcInit.u16Resolution    = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign     = ADC_DATAALIGN_RIGHT;
    ADC_Init(ISENSE_ADC_UNIT, &stcAdcInit);

    ADC_ChCmd(ISENSE_ADC_UNIT, ADC_SEQ_A, ISENSE_ADC_CH, ENABLE);

    /* ADC1 序列A 触发源选择 EVT0 */
    ADC_TriggerConfig(ISENSE_ADC_UNIT, ADC_SEQ_A, ADC_HARDTRIG_EVT0);
    /* AOS 将 HRPWM1 下溢(CTR=valley)事件映射到 ADC1 EVT0 触发输入 */
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_HRPWM_1_OVF);
    ADC_TriggerCmd(ISENSE_ADC_UNIT, ADC_SEQ_A, ENABLE);

    ADC_Start(ISENSE_ADC_UNIT);
}

/**
 * @brief  初始化电流采样 DMA
 * @param  None
 * @retval None
 */
static void ISENSE_DMA_Init(void)
{
    stc_dma_init_t stcDmaInit;
    stc_dma_repeat_init_t stcDmaRepeat;

    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA, ENABLE);

    DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn         = DMA_INT_DISABLE;
    stcDmaInit.u32SrcAddr       = (uint32_t)&ISENSE_ADC_UNIT->DR0;
    stcDmaInit.u32DestAddr      = (uint32_t)isenseDmaBuf;
    stcDmaInit.u32DataWidth     = DMA_DATAWIDTH_16BIT;
    stcDmaInit.u32BlockSize     = 1U;
    stcDmaInit.u32TransCount    = ISENSE_DMA_BUF_DEPTH;
    stcDmaInit.u32SrcAddrInc    = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc   = DMA_DEST_ADDR_INC;
    DMA_Init(ISENSE_DMA_UNIT, ISENSE_DMA_CH, &stcDmaInit);

    DMA_RepeatStructInit(&stcDmaRepeat);
    stcDmaRepeat.u32Mode        = DMA_RPT_DEST;
    stcDmaRepeat.u32DestCount   = ISENSE_DMA_BUF_DEPTH;
    DMA_RepeatInit(ISENSE_DMA_UNIT, ISENSE_DMA_CH, &stcDmaRepeat);

    DMA_ChCmd(ISENSE_DMA_UNIT, ISENSE_DMA_CH, ENABLE);
}

/**
 * @brief  从 DMA 缓冲区读取原始电流采样值
 * @param  None
 * @retval 16位原始电流采样值
 */
static uint16_t ISENSE_ReadRaw(void)
{
    uint32_t u32Remain;
    uint16_t u16Idx;

    u32Remain = DMA_GetTransCount(ISENSE_DMA_UNIT, ISENSE_DMA_CH);
    u16Idx = (uint16_t)((ISENSE_DMA_BUF_DEPTH - u32Remain) % ISENSE_DMA_BUF_DEPTH);
    if (u16Idx == 0U)
    {
        u16Idx = ISENSE_DMA_BUF_DEPTH - 1U;
    }
    else
    {
        u16Idx--;
    }
    return isenseDmaBuf[u16Idx];
}

/**
 * @brief  电流采样
 * @param  None
 * @retval 16位电流采样值
 */
float ISENSE_Sample(void)
{
    uint16_t u16Raw;
    uint32_t u32Sum = 0;
    uint8_t i;

    u16Raw = ISENSE_ReadRaw();
    isenseBuf[isenseIdx] = u16Raw;
    isenseIdx++;
    if (isenseIdx >= ISENSE_FILTER_WINDOW)
    {
        isenseIdx = 0;
    }

    for (i = 0; i < ISENSE_FILTER_WINDOW; i++)
    {
        u32Sum += isenseBuf[i];
    }
    u16Raw = (uint16_t)(u32Sum / ISENSE_FILTER_WINDOW);
    isenseLatestA = ISENSE_ADC_TO_CURRENT(u16Raw);
    return isenseLatestA;
}

/**
 * @brief  获取当前采样值
 * @param  None
 * @retval 16位电流采样值
 */
float ISENSE_GetCurrent(void)
{
    return isenseLatestA;
}


extern void PSFB_Stop(void);
/**
 * @brief  检查是否超过最大电流
 * @param  None
 * @retval 1: 超过最大电流, 0: 未超过最大电流
 */
uint8_t ISENSE_CheckOverCurrent(void)
{
    if (isenseLatestA >= ISENSE_OVERCURRENT_A)
    {
        PSFB_Stop();                // 超过最大电流, 停止系统
        return 1U;
    }
    return 0U;
}

/*********************************************************************************** */
/**
 * @brief  初始化电流采样 BSP
 * @param  None
 * @retval None
 */
void ISENSE_BSP_Init(void)
{
    ISENSE_GPIO_Init();
    ISENSE_ADC_Init();
    ISENSE_DMA_Init();
}


/*  ****** end of file  */
