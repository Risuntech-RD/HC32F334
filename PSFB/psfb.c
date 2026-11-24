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

    /* 设置 HRPWM3(右桥) 相位, 并更新缓冲区, 左桥相位保持不变 */
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
 * @note   从 PSFB_PHASE_MAX(最小功率) 逐步递减到目标相位, 每步延时使功率缓慢爬升
 * @param  None
 * @retval None
 */
void PSFB_Soft_Start(psfb_state_t state)
{
    uint32_t u32TargetPhase = phaseCnt;
    
    /* 1. 相位置于零功率(170°)起步, 避免启动瞬间功率冲击 */
    PSFB_SetPhase(PSFB_PHASE_MAX);

    /* 2. 清零所有计数器(主+从): 保证首个 PWM 周期从 0 开始、主从对齐, 消除重启乱波
     *    (PSFB_Stop 仅进入 IDLE 并停止计数, 模块/输出/PH/时钟/GPIO 配置均保持不变, 无需重新使能) */
    HRPWM_SWSyncClear(HRPWM_UNIT1 | HRPWM_UNIT2 | HRPWM_UNIT3);

    /* 3. 退出 IDLE, 释放输出到运行态 (此时相位=170°零功率, 输出干净无冲击) */
    HRPWM_IDLE_Exit(HRPWM_SW_SYNC_UNIT1 | HRPWM_SW_SYNC_UNIT2 | HRPWM_SW_SYNC_UNIT3, HRPWM_SW_SYNC_CH_ALL);

    /* 4. 启动主时基计数器: 移相从单元(HRPWM2/3)由主时基相位事件重装计数, 无需单独 CountStart */
    HRPWM_CountStart(CM_HRPWM1);

    /* 软启动: 从 PSFB_PHASE_MAX(170°, 零功率) 逐步递减到 phaseCnt, 每步延时使功率缓慢爬升 */
    if (state == PSFB_SS_ENABLE)
    {
        uint32_t i;
        uint32_t step = (PSFB_PHASE_MAX - u32TargetPhase) / PSFB_SS_STEPS;
        if (step == 0U)
        {
            step = 1U;
        }
        for (i = PSFB_PHASE_MAX; i > u32TargetPhase; i -= step)
        {
            PSFB_SetPhase(i);
            DDL_DelayMS(PSFB_SS_DELAY_MS);
        }
        PSFB_SetPhase(u32TargetPhase);
    }
    else
    {
        PSFB_SetPhase(u32TargetPhase);
    }

}


/**
 * @brief  停止移相全桥(PSFB)驱动
 * @param  None
 * @retval None
 */
void PSFB_Stop(void)
{
    /* 1. 相位拉到零功率(170°), 降低关断瞬间变压器伏秒冲击 */
    //PSFB_SetPhase(PSFB_PHASE_MAX);

    /* 2. 立即进入 IDLE: 强制所有 PWM 输出钳位到空闲低电平, 与计数器当前位置无关, 消除关断毛刺 */
    HRPWM_IDLE_EnterImmediate(HRPWM_SW_SYNC_UNIT1 | HRPWM_SW_SYNC_UNIT2 | HRPWM_SW_SYNC_UNIT3,  HRPWM_SW_SYNC_CH_ALL);

    /* 3. 同步停止并清零所有计数器(主+从): 输出已被 IDLE 钳低, 此处仅停止计数;
     *    保持模块/输出使能/PH/时钟/GPIO 配置不变, 确保下次启动状态干净可直接恢复 */
    HRPWM_SWSyncStop(HRPWM_UNIT1 | HRPWM_UNIT2 | HRPWM_UNIT3);
    HRPWM_SWSyncClear(HRPWM_UNIT1 | HRPWM_UNIT2 | HRPWM_UNIT3);
}

/**
 * @brief  启动移相全桥(PSFB)驱动
 * @note   仅清零计数器、退出 IDLE 并启动主时基计数, 保持当前相位 phaseCnt 不变
 * @param  None
 * @retval None
 */
void PSFB_Start(void)
{
    /* 1. 清零所有计数器(主+从): 保证首个 PWM 周期从 0 开始、主从对齐, 消除重启乱波 */
    HRPWM_SWSyncClear(HRPWM_UNIT1 | HRPWM_UNIT2 | HRPWM_UNIT3);

    /* 2. 退出 IDLE, 释放输出到运行态: 输出恢复由计数器/比较值控制 (保持当前相位, 无冲击) */
    HRPWM_IDLE_Exit(HRPWM_SW_SYNC_UNIT1 | HRPWM_SW_SYNC_UNIT2 | HRPWM_SW_SYNC_UNIT3, HRPWM_SW_SYNC_CH_ALL);

    /* 3. 启动主时基计数器: 移相从单元(HRPWM2/3)由主时基相位事件重装计数, 无需单独 CountStart */
    HRPWM_CountStart(CM_HRPWM1);
}

/**
 * @brief  Configure PWM phase function
 * @param  [in] u32PhaseValue  PWM phase value
 * @retval None
 */
void Phase_Config(uint32_t u32PhaseValue)
{
    stc_hrpwm_ph_config_t stcPHConfig;

    /* HRPWM1 为移相主时基, PHSCMP1~5 全局共享: IDX1=左桥 0° 基准, IDX2=右桥移相角 */
    HRPWM_PH_SetCompareValue(HRPWM_PH_MATCH_IDX1, 0U);
    HRPWM_PH_SetCompareValue_Buf(HRPWM_PH_MATCH_IDX1, 0U);
    HRPWM_PH_SetCompareValue(HRPWM_PH_MATCH_IDX2, u32PhaseValue);
    HRPWM_PH_SetCompareValue_Buf(HRPWM_PH_MATCH_IDX2, u32PhaseValue);

    (void)HRPWM_PH_StructInit(&stcPHConfig);
    stcPHConfig.u32ForceChA   = HRPWM_PH_MATCH_FORCE_CHA_OFF;
    stcPHConfig.u32ForceChB   = HRPWM_PH_MATCH_FORCE_CHB_OFF;
    stcPHConfig.u32PeriodLink = HRPWM_PH_PERIOD_LINK_ON;

    /* 左桥 HRPWM2 锁到 IDX1, 作为 0° 相位基准 */
    stcPHConfig.u32PhaseIndex = HRPWM_PH_MATCH_IDX1;
    (void)HRPWM_PH_Config(HRPWM_MASTER, &stcPHConfig);
    HRPWM_PH_Enable(HRPWM_MASTER);

    /* 右桥 HRPWM3 锁到 IDX2, 实现相对左桥的移相 */
    stcPHConfig.u32PhaseIndex = HRPWM_PH_MATCH_IDX2;
    (void)HRPWM_PH_Config(HRPWM_SLAVE, &stcPHConfig);
    HRPWM_PH_Enable(HRPWM_SLAVE);
}


/**
 * @brief   Configure PWM output
 * @param   [in] u32PeriodValue       PWM period value
 * @param   [in] u32CompareValue      PWM compare value
 * @param   [in] u32DeadTimeValue     PWM dead time value
 * @arg     CM_HRPWM_x
 * @retval  None
 */
void PWM_Config(uint32_t u32PeriodValue, uint32_t u32CompareValue, uint32_t u32DeadTimeValue)
{
    stc_hrpwm_init_t            stcHrpwmInit;
    stc_hrpwm_pwm_init_t        stcPwmInit;
    stc_hrpwm_deadtime_config_t stcDeadTimeConfig;

    /* HRPWM1 为移相(PH)模块硬件固定主时基: 必须先设置其周期, PHSCMP 匹配才会产生相位事件 */
    (void)HRPWM_StructInit(&stcHrpwmInit);
    stcHrpwmInit.u32CountMode   = HRPWM_MD_SAWTOOTH;
    stcHrpwmInit.u32CountReload = HRPWM_CNT_RELOAD_ON;
    stcHrpwmInit.u32PeriodValue = u32PeriodValue;                       // PWM period value
    (void)HRPWM_Init(HRPWM_MAIN, &stcHrpwmInit);

    /* 左桥 HRPWM2 / 右桥 HRPWM3 均为移相从单元: CountReload=OFF, 由相位匹配事件重装计数器实现移相 */
    stcHrpwmInit.u32CountReload = HRPWM_CNT_RELOAD_OFF;
    (void)HRPWM_Init(HRPWM_MASTER, &stcHrpwmInit);
    (void)HRPWM_Init(HRPWM_SLAVE,  &stcHrpwmInit);

    /* Configure PWM output */
    (void)HRPWM_PWM_StructInit(&stcPwmInit);
    stcPwmInit.u32CompareValue              = u32CompareValue;          // PWM compare value
    stcPwmInit.u32StartPolarity             = HRPWM_PWM_START_LOW;
    stcPwmInit.u32PeakPolarity              = HRPWM_PWM_PEAK_INVT;
    stcPwmInit.u32ValleyPolarity            = HRPWM_PWM_VALLEY_HOLD;
    stcPwmInit.u32UpMatchAPolarity          = HRPWM_PWM_UP_MATCH_A_HIGH;
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

    stcPwmInit.u32StartPolarity             = HRPWM_PWM_START_HIGH;
    stcPwmInit.u32UpMatchAPolarity          = HRPWM_PWM_UP_MATCH_A_LOW;
    stcPwmInit.u32DownMatchAPolarity        = HRPWM_PWM_DOWN_MATCH_A_HIGH;
    (void)HRPWM_PWM_ChBInit(HRPWM_MASTER, &stcPwmInit);
    (void)HRPWM_PWM_ChBInit(HRPWM_SLAVE, &stcPwmInit);

    /* 死区: 左右桥对称死区, 防止上下管直通 */
    (void)HRPWM_DeadTimeStructInit(&stcDeadTimeConfig);
    stcDeadTimeConfig.u32EqualUpDown = HRPWM_DEADTIME_EQUAL_ON;
    (void)HRPWM_DeadTimeConfig(HRPWM_MASTER, &stcDeadTimeConfig);
    (void)HRPWM_DeadTimeConfig(HRPWM_SLAVE,  &stcDeadTimeConfig);
    HRPWM_SetDeadTimeUpValue(HRPWM_MASTER, u32DeadTimeValue);
    HRPWM_SetDeadTimeUpValue(HRPWM_SLAVE,  u32DeadTimeValue);
    HRPWM_DeadTimeEnable(HRPWM_MASTER);
    HRPWM_DeadTimeEnable(HRPWM_SLAVE);

    /* IDLE 空闲电平配置为低: 停止/异常进入 IDLE 时, 输出安全钳位到低, 避免毛刺 */
    HRPWM_IDLE_SetChAIdleLevel(HRPWM_MASTER, HRPWM_IDLE_CHA_LVL_LOW);
    HRPWM_IDLE_SetChBIdleLevel(HRPWM_MASTER, HRPWM_IDLE_CHB_LVL_LOW);
    HRPWM_IDLE_SetChAIdleLevel(HRPWM_SLAVE,  HRPWM_IDLE_CHA_LVL_LOW);
    HRPWM_IDLE_SetChBIdleLevel(HRPWM_SLAVE,  HRPWM_IDLE_CHB_LVL_LOW);

    /* PWM output enable (HRPWM1 仅作移相主时基, 不使能输出) */
    HRPWM_PWM_ChAOutputEnable(HRPWM_MASTER);
    HRPWM_PWM_ChBOutputEnable(HRPWM_MASTER);
    HRPWM_PWM_ChAOutputEnable(HRPWM_SLAVE);
    HRPWM_PWM_ChBOutputEnable(HRPWM_SLAVE);
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
    PWM_Config(PSFB_PERIOD, PSFB_PERIOD / 2, PSFB_DT_CNT);
    Phase_Config(PSFB_PHASE_MIN);

    HRPWM_Enable(CM_HRPWM1);        // HRPWM1(主)使能
    HRPWM_Enable(HRPWM_MASTER);
    HRPWM_Enable(HRPWM_SLAVE);

    HRPWM_IDLE_Exit(HRPWM_SW_SYNC_UNIT1 | HRPWM_SW_SYNC_UNIT2 | HRPWM_SW_SYNC_UNIT3, HRPWM_SW_SYNC_CH_ALL);
    //HRPWM_CountStart(CM_HRPWM1);


    //ISENSE_Init();

}
    





/*  ****** end of file  */