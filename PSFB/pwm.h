/**
 ******************************************************************************
 * @file    pwm.h
 * @brief   PWM 驱动 - HC32F334JATI TMRA_4_CH3
 * @note    基于 HC32F334_DDL_Rev1.1.1 (hc32_ll_xxx 驱动库)
 *
 * 硬件连接:
 *   PA3  -> TMRA_4_PWM3  (GPIO_FUNC_13)
 *   PA3  亦可复用为 TMR6_1_PWMA, 同一引脚二者只能选其一
 *
 * 时钟: 24MHz XTAL -> PLL -> 120MHz HCLK
 ******************************************************************************
 */

#ifndef __PWM_H__
#define __PWM_H__

#include "hc32_ll_def.h"
#include "hc32_ll.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define     PWM_TIM_PORT         (GPIO_PORT_A)
#define     PWM_TIM_PIN          (GPIO_PIN_03)
#define     PWM_TIM_FUNC         (GPIO_FUNC_13)   /* PA3 -> TMRA_4_PWM3 */
#define     PWM_TIM_UNIT         (CM_TMRA_4)


/*===================== 函数声明 =====================*/

void                PWM_BSP_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __PWM_H__ */