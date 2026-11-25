/**
 ******************************************************************************
 * @file    psfb_hc.h
 * @brief   移相全桥(PSFB)驱动 - HC32F334JATI HRPWM2/HRPWM3
 * @note    基于 HC32F334_DDL_Rev1.1.1 (hc32_ll_xxx 驱动库)
 *
 * 硬件连接:
 *   PA8  -> HRPWM2_PWMA -> U4 HIN -> Q1 (左桥高端)
 *   PA9  -> HRPWM2_PWMB -> U4 LIN -> Q2 (左桥低端)
 *   PA10 -> HRPWM3_PWMA -> U5 HIN -> Q3 (右桥高端)
 *   PA11 -> HRPWM3_PWMB -> U5 LIN -> Q4 (右桥低端)
 *   PA1  -> ADC1_IN1 (电流采样, R18=0.5Ω + LM358 Gain=2)
 *
 * 时钟: 24MHz XTAL -> PLL -> 120MHz HCLK
 ******************************************************************************
 */

#ifndef __ADC_H__
#define __ADC_H__

#include "hc32_ll_def.h"
#include "hc32_ll.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/*----- 电流采样配置 (R18=0.5Ω, U6.1 LM358 同相放大 Gain=2) -----*/
#define ISENSE_ADC_PORT         (GPIO_PORT_A)
#define ISENSE_ADC_PIN          (GPIO_PIN_01)
#define ISENSE_ADC_UNIT         (CM_ADC1)
#define ISENSE_ADC_CH           (ADC_CH1)
#define ISENSE_SHUNT_OHM        (0.5f)      // 电流采样电阻 R18 = 0.5Ω
#define ISENSE_AMP_GAIN         (2.0f)
#define ISENSE_ADC_VREF_V       (3.3f)
#define ISENSE_ADC_MAXCODE      (4095U)
#define ISENSE_OVERCURRENT_A    (3.0f)
#define ISENSE_DMA_BUF_DEPTH    (16U)
#define ISENSE_FILTER_WINDOW    (8U)

#define ISENSE_ADC_TO_CURRENT(adc)\
        ((float)(adc) * ISENSE_ADC_VREF_V / ISENSE_ADC_MAXCODE / (ISENSE_AMP_GAIN * ISENSE_SHUNT_OHM))


/*----- DMA 配置: ADC1 -> 内存循环缓冲区 -----*/
#define ISENSE_DMA_UNIT         (CM_DMA)
#define ISENSE_DMA_CH           (DMA_CH0)


/*===================== 全局变量 =====================*/
extern volatile uint16_t        isenseDmaBuf[ISENSE_DMA_BUF_DEPTH];



/*===================== 函数声明 =====================*/
float               ISENSE_Sample(void);
float               ISENSE_GetCurrent(void);
uint8_t             ISENSE_CheckOverCurrent(void);
void                ISENSE_BSP_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

