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

#ifndef __PSFB_HC_H__
#define __PSFB_HC_H__

#include "hc32_ll_def.h"
#include "hc32_ll.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/*=========================== 可配置参数 ===========================*/
#define PSFB_HRPWM_CLK_HZ       (120000000UL)
#define PSFB_PWM_FREQ_HZ        (20000UL)
#define PSFB_DEAD_TIME_NS       (500UL)


/* 死区计数: (120MHz * 500ns) << 6 = 60 << 6 = 3840 */
#define PSFB_HRPWM_DT_CNT       (uint32_t)(((PSFB_HRPWM_CLK_HZ / 1000000UL) * PSFB_DEAD_TIME_NS / 1000UL) << 6)
#define PSFB_HRPWM_HALF_PERIOD  (uint32_t)(PSFB_HRPWM_PERIOD / 2U)
#define PSFB_INIT_PHASE_CNT     (uint32_t)(250U << 6)   /* 250 PCLK0 周期的相位偏移 */

/*--------------------------- 计算值 ---------------------------*/
/* HRPWM 高精度寄存器格式: 1 PCLK0 周期 = 64 计数 (<<6), 低6位为小数 */
#define PSFB_PERIOD             (uint32_t)((PSFB_HRPWM_CLK_HZ / (2UL * PSFB_PWM_FREQ_HZ)) << 6)
#define PSFB_DT_CNT             (uint32_t)(((PSFB_HRPWM_CLK_HZ / 1000000UL) * PSFB_DEAD_TIME_NS / 1000UL) << 6)
#define PSFB_HALF_PERIOD        (uint32_t)(PSFB_PERIOD / 2U)

/* 中心对齐: 频率 = HRPWM_CLK / (2 * PERIOD)
 * HRPWM 高精度寄存器: 1 PCLK0 周期 = 64 计数 (<<6)
 * PERIOD = (120M / (2*20k)) << 6 = 3000 << 6 = 192000 */
#define PSFB_HRPWM_PERIOD       (uint32_t)((PSFB_HRPWM_CLK_HZ / (2UL * PSFB_PWM_FREQ_HZ)) << 6)

/* 相位限幅: 5°~170°, 避免 0°(满电压冲击/偏磁) 与 180°(直通/环流损耗) */
#define PSFB_PHASE_MIN          (uint32_t)((PSFB_PERIOD * 5UL)   / 180UL)
#define PSFB_PHASE_MAX          (uint32_t)((PSFB_PERIOD * 170UL) / 180UL)

/*--------------------- HRPWM 模块定义 ---------------------*/
#define HRPWM_MASTER            (CM_HRPWM2)
#define HRPWM_SLAVE             (CM_HRPWM3)



/*===================== 系统时钟参数 (24MHz XTAL -> 120MHz) =====================*/
#define PSFB_XTAL_FREQ_HZ       (24000000UL)
#define PSFB_SYSCLK_HZ          (120000000UL)

/*===================== HRPWM 参数 =====================*/
#define PSFB_PWM_FREQ_HZ        (20000UL)       /* 开关频率 20kHz */
#define PSFB_HRPWM_CLK_HZ       (120000000UL)   /* HRPWM 时钟 = PCLK0 = 120MHz */
#define PSFB_DEAD_TIME_NS       (500UL)


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
#define ISENSE_DMA_BUF_DEPTH    (16U)


/*===================== 全局变量 =====================*/
extern volatile uint16_t        isenseDmaBuf[ISENSE_DMA_BUF_DEPTH];


/*===================== 状态机 =====================*/
typedef enum
{
    PSFB_SOFTSTART_ON       = (0U),
    PSFB_SOFTSTART_OFF      = (1U),
} psfb_state_t;

/* 软启动延时 = PSFB_SOFTSTART_STEPS * PSFB_SOFTSTART_DELAY_MS = 400mS */
#define PSFB_SOFTSTART_STEPS    (40U)
#define PSFB_SOFTSTART_DELAY_MS (10U)


/*===================== 函数声明 =====================*/
// static void         PSFB_GPIO_Init(void);
// static void         PSFB_HRPWM2_Init(void);
// static void         PSFB_HRPWM3_Init(void);
void                PSFB_SetPhase(uint32_t u32PhaseCnt);
uint32_t            PSFB_GetPhase(void);
void                PSFB_Start(psfb_state_t state);
void                PSFB_Stop(void);

// static void         ISENSE_GPIO_Init(void);
// static void         ISENSE_ADC_Init(void);
// static void         ISENSE_DMA_Init(void);
// static uint16_t     ISENSE_ReadRaw(void);
float               ISENSE_Sample(void);
float               ISENSE_GetCurrent(void);
uint8_t             ISENSE_CheckOverCurrent(void);
void                ISENSE_Init(void);

void                PSFB_BSP_Init(void);



#ifdef __cplusplus
}
#endif

#endif /* __PSFB_HC_H__ */

