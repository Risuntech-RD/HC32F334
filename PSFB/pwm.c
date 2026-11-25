/**
 * @file pwm.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "pwm.h"

/*======================= PWM =======================*/
/**
 * @brief  初始化 PWM GPIO
 * @param  None
 * @retval None
 */
void PWM_GPIO_Init(void)
{
    stc_gpio_init_t stcGpio;

    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    GPIO_Init(PWM_TIM_PORT, PWM_TIM_PIN, &stcGpio);

    /* PA3 复用为 TMRA_4_PWM3 (Func13) */
    GPIO_SetFunc(PWM_TIM_PORT, PWM_TIM_PIN, PWM_TIM_FUNC);
}

void PWM_TIM_Init(void)
{
    stc_tmra_init_t         stcTimInit;
    stc_tmra_pwm_init_t     stcPwmInit;

    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_4, ENABLE);

    /* 配置 TMRA 4 为 16 位 SAWTOOTH 模式 */
    (void)TMRA_StructInit(&stcTimInit);
    stcTimInit.u8CountSrc              = TMRA_CNT_SRC_SW;
    stcTimInit.sw_count.u8ClockDiv     = TMRA_CLK_DIV1;
    stcTimInit.sw_count.u8CountMode    = TMRA_MD_SAWTOOTH;      // TMRA_MD_TRIANGLE, 
    stcTimInit.sw_count.u8CountDir     = TMRA_DIR_UP;
    stcTimInit.u32PeriodValue          = 7499U;                 // 120MHz / 1 / 7500 = 16kHz
    stcTimInit.u8CountReload           = TMRA_CNT_RELOAD_ENABLE;
    (void)TMRA_Init(CM_TMRA_4, &stcTimInit);

    /* 配置 PWM 通道3: 先写比较值/极性, 再使能输出 */
    (void)TMRA_PWM_StructInit(&stcPwmInit);
    stcPwmInit.u32CompareValue          = 3749U;
    stcPwmInit.u16StartPolarity         = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity          = TMRA_PWM_LOW;
    stcPwmInit.u16CompareMatchPolarity  = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity   = TMRA_PWM_HIGH;
    TMRA_PWM_Init(CM_TMRA_4, TMRA_CH3, &stcPwmInit);

    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH3, ENABLE);

    TMRA_Start(CM_TMRA_4);
}



/*********************************************************************************** */
/**
 * @brief  初始化 PWM BSP
 * @param  None
 * @retval None
 */
void PWM_BSP_Init(void)
{
    PWM_GPIO_Init();
    PWM_TIM_Init();

}


/*  ****** end of file  */
