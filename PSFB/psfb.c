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
 *   HRPWM2(左桥): 50%占空比互补PWM, 相位=0(基准), 产生同步脉冲给HRPWM3
 *   HRPWM3(右桥): 50%占空比互补PWM, 相位=phase(0~PERIOD, 对应0°~180°)
 *   phase=0       -> 左右桥同相, 变压器电压最大
 *   phase=PERIOD  -> 左右桥反相, 变压器电压为零
 *   死区由 HRPWM 硬件死区发生器保证, 无需软件干预
 *
 * 数据流 (采样):
 *   HRPWM2 周期事件 -> ADC内部触发 -> ADC转换 -> EOC
 *   -> DMA硬件请求 -> DMA写入 isenseDmaBuf[](循环回绕)
 ******************************************************************************/

#include "psfb.h"


/*=========================== 全局变量 ===========================*/
static uint32_t     phaseCnt        = PSFB_PHASE_MIN;

static uint16_t     isenseBuf[ISENSE_FILTER_WINDOW] = {0};      // 采样数据滤波缓冲区
static uint8_t      isenseIdx       = 0;                        // 采样数据索引
static float        isenseLatestA   = 0.0f;                     // 最新采样数据

volatile uint16_t   isenseDmaBuf[ISENSE_DMA_BUF_DEPTH] = {0};   // DMA搬运缓冲区


/*=========================== PWM 部分 ===========================*/
/**
 * @brief  初始化 GPIO 引脚
 * @param  None
 * @retval None
 */
static void PSFB_GPIO_Init(void)
{
    stc_gpio_init_t stcGpio;

    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir   = PIN_DIR_OUT;
    stcGpio.u16PinState = PIN_STAT_RST;
    stcGpio.u16PinDrv   = PIN_HIGH_DRV;

    GPIO_Init(GPIO_PORT_A, GPIO_PIN_08, &stcGpio);
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_08, GPIO_HRPWM2_PWMA);

    GPIO_Init(GPIO_PORT_A, GPIO_PIN_09, &stcGpio);
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_09, GPIO_HRPWM2_PWMB);

    GPIO_Init(GPIO_PORT_A, GPIO_PIN_10, &stcGpio);
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_10, GPIO_HRPWM3_PWMA);

    GPIO_Init(GPIO_PORT_A, GPIO_PIN_11, &stcGpio);
    GPIO_SetFunc(GPIO_PORT_A, GPIO_PIN_11, GPIO_HRPWM3_PWMB);

    // GPIO_SubFuncCmd(GPIO_PORT_A, GPIO_PIN_08, ENABLE);
    // GPIO_SubFuncCmd(GPIO_PORT_A, GPIO_PIN_09, ENABLE);
    // GPIO_SubFuncCmd(GPIO_PORT_A, GPIO_PIN_10, ENABLE);
    // GPIO_SubFuncCmd(GPIO_PORT_A, GPIO_PIN_11, ENABLE);

    GPIO_HrpwmPinCmd(GPIO_HRPWM2_PWMA | GPIO_HRPWM2_PWMB | GPIO_HRPWM3_PWMA | GPIO_HRPWM3_PWMB, ENABLE);

}


/**
 * @brief  设置移相全桥(PSFB)相位
 * @param  u32PhaseCnt: 相位值, 范围 [PSFB_PHASE_MIN, PSFB_PHASE_MAX]
 * @retval None
 */
void PSFB_SetPhase(uint32_t u32PhaseCnt)
{
    if (u32PhaseCnt < PSFB_PHASE_MIN)
        u32PhaseCnt = PSFB_PHASE_MIN;
    if (u32PhaseCnt > PSFB_PHASE_MAX)
        u32PhaseCnt = PSFB_PHASE_MAX;
    phaseCnt = u32PhaseCnt;
    HRPWM_PH_SetCompareValue(HRPWM_PH_MATCH_IDX2, u32PhaseCnt);
    HRPWM_PH_SetCompareValue_Buf(HRPWM_PH_MATCH_IDX2, u32PhaseCnt);
}

/**
 * @brief  获取当前相位
 * @param  None
 * @retval 当前相位值, 范围 [PSFB_PHASE_MIN, PSFB_PHASE_MAX]
 */
uint32_t PSFB_GetPhase(void)
{
    return phaseCnt;
}


/**
 * @brief  启动移相全桥(PSFB)驱动
 * @note   从 PSFB_PHASE_MAX(170°, 零功率) 逐步递减到 phaseCnt, 每步延时使功率缓慢爬升
 * @param  None
 * @retval None
 */
void PSFB_Start(psfb_state_t state)
{
    PSFB_SetPhase(PSFB_PHASE_MAX);

    /* 退出空闲状态, 使能 PWM 输出通道 */
    HRPWM_IDLE_Exit(HRPWM_SW_SYNC_UNIT2 | HRPWM_SW_SYNC_UNIT3, HRPWM_SW_SYNC_CH_ALL);

    HRPWM_CountStart(HRPWM_SLAVE);
    HRPWM_CountStart(HRPWM_MASTER);
    HRPWM_IDLE_Exit(HRPWM_SW_SYNC_UNIT3, HRPWM_SW_SYNC_CH_ALL);
    /* 软启动: 从 PSFB_PHASE_MAX(170°, 零功率) 逐步递减到 phaseCnt, 每步延时使功率缓慢爬升 */
    if (state == PSFB_SS_ENABLE)
    {
        uint32_t i;
        uint32_t step = (PSFB_PHASE_MAX - phaseCnt) / PSFB_SS_STEPS;
        if (step == 0U)
        {
            step = 1U;
        }
        for (i = PSFB_PHASE_MAX; i > phaseCnt; i -= step)
        {
            PSFB_SetPhase(i);
            DDL_DelayMS(PSFB_SS_DELAY_MS);
        }
        PSFB_SetPhase(phaseCnt);
    }
    else
    {
        PSFB_SetPhase(phaseCnt);
    }
}

/**
 * @brief  停止移相全桥(PSFB)驱动
 * @param  None
 * @retval None
 */
void PSFB_Stop(void)
{
    HRPWM_CountStop(HRPWM_MASTER);
    HRPWM_CountStop(HRPWM_SLAVE);
    HRPWM_PWM_ChAOutputDisable(HRPWM_MASTER);
    HRPWM_PWM_ChBOutputDisable(HRPWM_MASTER);
    HRPWM_PWM_ChAOutputDisable(HRPWM_SLAVE);
    HRPWM_PWM_ChBOutputDisable(HRPWM_SLAVE);

    HRPWM_PH_Disable(HRPWM_SLAVE);    /* 禁用 PH 移相模块 */
    PSFB_SetPhase(PSFB_PHASE_MAX);    /* 相位配置为 PSFB_PHASE_MAX(170°, 零功率) */
}

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
    /* AOS 将 HRPWM2 下溢(CTR=valley)事件映射到 ADC1 EVT0 触发输入 */
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_HRPWM_2_UDF);
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

/**
 * @brief  检查是否超过最大电流
 * @param  None
 * @retval 1: 超过最大电流, 0: 未超过最大电流
 */
uint8_t ISENSE_CheckOverCurrent(void)
{
    if (isenseLatestA >= ISENSE_OVERCURRENT_A)
    {
        PSFB_Stop();
        return 1U;
    }
    return 0U;
}

/**
 * @brief  初始化电流采样
 * @param  None
 * @retval None
 */
void ISENSE_Init(void)
{
    ISENSE_GPIO_Init();
    ISENSE_ADC_Init();
    ISENSE_DMA_Init();
}

/*=========================== 总初始化 ===========================*/
void PSFB_BSP_Init(void)
{
    PSFB_GPIO_Init();

    /* HRPWM 时钟使能 */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_HRPWM_1 | FCG2_PERIPH_HRPWM_2 | FCG2_PERIPH_HRPWM_3, ENABLE);

    HRPWM_CommonDeInit();
    HRPWM_CALIB_ProcessSingle();
    HRPWM_Enable(HRPWM_MASTER);
    HRPWM_Enable(HRPWM_SLAVE);

    PWM_Config(PSFB_PERIOD, PSFB_PERIOD / 3);
    Phase_Config();
    DeadTime_Config();

    HRPWM_IDLE_Exit(HRPWM_SW_SYNC_UNIT2|HRPWM_SW_SYNC_UNIT3, HRPWM_SW_SYNC_CH_ALL);
    HRPWM_CountStart(HRPWM_MASTER);
    HRPWM_CountStart(HRPWM_SLAVE);

    //ISENSE_Init();

}

/**
 * @brief  Configure PWM phase function
 * @param  None
 * @retval None
 */
void Phase_Config(void)
{
    stc_hrpwm_ph_config_t stcPHConfig;

    HRPWM_PH_SetCompareValue(HRPWM_PH_MATCH_IDX2, PSFB_PHASE_MID);
    HRPWM_PH_SetCompareValue_Buf(HRPWM_PH_MATCH_IDX2, PSFB_PHASE_MID);

    (void)HRPWM_PH_StructInit(&stcPHConfig);
    stcPHConfig.u32PhaseIndex   = HRPWM_PH_MATCH_IDX2;
    stcPHConfig.u32ForceChA     = HRPWM_PH_MATCH_FORCE_CHA_OFF;
    stcPHConfig.u32ForceChB     = HRPWM_PH_MATCH_FORCE_CHB_OFF;
    stcPHConfig.u32PeriodLink   = HRPWM_PH_PERIOD_LINK_ON;
    (void)HRPWM_PH_Config(HRPWM_SLAVE, &stcPHConfig);

    HRPWM_PH_Enable(HRPWM_SLAVE);

}

/**
 * @brief  Configure PWM dead time function
 * @param  None
 * @retval None
 */
void DeadTime_Config(void)
{
    stc_hrpwm_deadtime_config_t stcDeadTimeConfig;
    (void)HRPWM_DeadTimeStructInit(&stcDeadTimeConfig);
    /* Set dead time value */
    HRPWM_SetDeadTimeUpValue(HRPWM_MASTER, HRPWM_DEADTIME_VALUE);
    HRPWM_SetDeadTimeDownValue(HRPWM_MASTER, HRPWM_DEADTIME_VALUE / 2U);
    HRPWM_SetDeadTimeUpValue(HRPWM_SLAVE, HRPWM_DEADTIME_VALUE);
    HRPWM_SetDeadTimeDownValue(HRPWM_SLAVE, HRPWM_DEADTIME_VALUE / 2U);
    
    /* DeadTime function configure */
    stcDeadTimeConfig.u32EqualUpDown = HRPWM_DEADTIME_EQUAL_OFF;
    stcDeadTimeConfig.u32BufUp       = HRPWM_DEADTIME_CNT_UP_BUF_OFF;
    stcDeadTimeConfig.u32BufDown     = HRPWM_DEADTIME_CNT_DOWN_BUF_OFF; 
    (void)HRPWM_DeadTimeConfig(HRPWM_MASTER, &stcDeadTimeConfig);
    (void)HRPWM_DeadTimeConfig(HRPWM_SLAVE, &stcDeadTimeConfig);
}

/**
 * @brief   Configure PWM output
 * @param   [in] HRPWMx               HRPWM unit
 * @arg     CM_HRPWM_x
 * @retval  None
 */
void PWM_Config(uint32_t u32PeriodValue, uint32_t u32CompareValue)
{
    stc_hrpwm_init_t     stcHrpwmInit;
    stc_hrpwm_pwm_init_t stcPwmInit;

    /* HRPWM general count function configure */
    (void)HRPWM_StructInit(&stcHrpwmInit);
    stcHrpwmInit.u32CountMode               = HRPWM_MD_SAWTOOTH;
    stcHrpwmInit.u32CountReload             = HRPWM_CNT_RELOAD_ON;
    stcHrpwmInit.u32PeriodValue             = u32PeriodValue;
    (void)HRPWM_Init(HRPWM_MASTER, &stcHrpwmInit);

    stcHrpwmInit.u32CountReload             = HRPWM_CNT_RELOAD_OFF;
    (void)HRPWM_Init(HRPWM_SLAVE,  &stcHrpwmInit);


    /* Configure PWM output */
    (void)HRPWM_PWM_StructInit(&stcPwmInit);
    stcPwmInit.u32CompareValue              = u32CompareValue;
    stcPwmInit.u32StartPolarity             = HRPWM_PWM_START_LOW;          // LOW
    stcPwmInit.u32PeakPolarity              = HRPWM_PWM_PEAK_INVT;          // HRPWM_PWM_PEAK_INVT;
    stcPwmInit.u32ValleyPolarity            = HRPWM_PWM_VALLEY_HOLD;
    stcPwmInit.u32UpMatchAPolarity          = HRPWM_PWM_UP_MATCH_A_HIGH;    // HRPWM_PWM_UP_MATCH_A_INVT;
    stcPwmInit.u32DownMatchAPolarity        = HRPWM_PWM_DOWN_MATCH_A_HOLD;
    stcPwmInit.u32UpMatchBPolarity          = HRPWM_PWM_UP_MATCH_B_HOLD;
    stcPwmInit.u32DownMatchBPolarity        = HRPWM_PWM_DOWN_MATCH_B_HOLD;
    stcPwmInit.u32UpMatchEPolarity          = HRPWM_PWM_UP_MATCH_E_HOLD;
    stcPwmInit.u32DownMatchEPolarity        = HRPWM_PWM_DOWN_MATCH_E_HOLD;
    stcPwmInit.u32UpMatchFPolarity          = HRPWM_PWM_UP_MATCH_F_HOLD;
    stcPwmInit.u32DownMatchFPolarity        = HRPWM_PWM_DOWN_MATCH_F_HOLD;
    stcPwmInit.u32UpMatchSpecialAPolarity   = HRPWM_PWM_UP_MATCH_SPECIAL_A_HOLD;
    stcPwmInit.u32DownMatchSpecialAPolarity = HRPWM_PWM_DOWN_MATCH_SPECIAL_A_HOLD;
    stcPwmInit.u32UpMatchSpecialBPolarity   = HRPWM_PWM_UP_MATCH_SPECIAL_B_HOLD;
    stcPwmInit.u32DownMatchSpecialBPolarity = HRPWM_PWM_DOWN_MATCH_SPECIAL_B_HOLD;
    (void)HRPWM_PWM_ChAInit(HRPWM_MASTER, &stcPwmInit);
    (void)HRPWM_PWM_ChAInit(HRPWM_SLAVE, &stcPwmInit);

    stcPwmInit.u32StartPolarity             = HRPWM_PWM_START_HIGH;         // HIGH
    stcPwmInit.u32UpMatchAPolarity          = HRPWM_PWM_UP_MATCH_A_LOW;
    stcPwmInit.u32DownMatchAPolarity        = HRPWM_PWM_DOWN_MATCH_A_HIGH;
    (void)HRPWM_PWM_ChBInit(HRPWM_MASTER, &stcPwmInit);
    (void)HRPWM_PWM_ChBInit(HRPWM_SLAVE, &stcPwmInit);

    /* 同步信号输出配置 (主单元输出同步脉冲给从单元, 作为PH移相基准) */
    stc_hrpwm_sync_output_config_t stcSyncConfig;
    
    (void)HRPWM_SYNC_StructInit(&stcSyncConfig);
    stcSyncConfig.u32Src        = HRPWM_SYNC_SRC_U2_CNT_VALLEY;     // 源: U2谷底
    stcSyncConfig.u32Pulse      = HRPWM_SYNC_PULSE_POSITIVE;        // 正脉冲
    stcSyncConfig.u32PulseWidth = 0x10UL;
    (void)HRPWM_SYNC_Config(&stcSyncConfig);                        // 同步脉冲配置


#if 0
    /* 死区配置 */
    stc_hrpwm_deadtime_config_t stcDeadtimeInit;
    (void)HRPWM_DeadTimeStructInit(&stcDeadtimeInit);
    stcDeadtimeInit.u32EqualUpDown = HRPWM_DEADTIME_EQUAL_ON;
    (void)HRPWM_DeadTimeConfig(HRPWMx, &stcDeadtimeInit);
    HRPWM_SetDeadTimeUpValue(HRPWMx, PSFB_DT_CNT);
    HRPWM_DeadTimeEnable(HRPWMx);

    /* PH移相配置 (仅从单元HRPWM3需要移相, 主单元HRPWM2相位=0不需要) */
    if (HRPWMx == HRPWM_SLAVE) {
        stc_hrpwm_ph_config_t stcPhInit;

        (void)HRPWM_PH_StructInit(&stcPhInit);
        stcPhInit.u32PeriodLink     = HRPWM_PH_PERIOD_LINK_ON;
        stcPhInit.u32PhaseIndex     = HRPWM_PH_MATCH_IDX2;
        (void)HRPWM_PH_Config(HRPWMx, &stcPhInit);

        HRPWM_PH_SetCompareValue(HRPWM_PH_MATCH_IDX2, PSFB_PHASE_MIN);      /* 写入 active 寄存器 PHSCMP2A */
        HRPWM_PH_SetCompareValue_Buf(HRPWM_PH_MATCH_IDX2, PSFB_PHASE_MIN);  /* 同步写入 buffer   PHSCMP2B, 防止传输覆盖 */
        HRPWM_PH_Enable(HRPWMx);
    }
#endif

    /* PWM output enable */
    HRPWM_PWM_ChAOutputEnable(HRPWM_MASTER);
    HRPWM_PWM_ChBOutputEnable(HRPWM_MASTER);
    HRPWM_PWM_ChAOutputEnable(HRPWM_SLAVE);     // 使能从单元 HRPWM3 的 PWM 输出
    HRPWM_PWM_ChBOutputEnable(HRPWM_SLAVE);

}


/*  ****** end of file  */