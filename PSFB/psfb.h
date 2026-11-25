/**
 ******************************************************************************
 * @file    psfb.h
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

#ifndef __PSFB_H__
#define __PSFB_H__

#include "hc32_ll_def.h"
#include "hc32_ll.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===================== 系统时钟参数 (24MHz XTAL -PLL-> 120MHz) =====================*/
#define PSFB_SYSCLK_HZ          (120000000UL)

/*===================== HRPWM时钟参数 =====================*/
#define PSFB_HRPWM_CLK_HZ       (PSFB_SYSCLK_HZ)    /* HRPWM 时钟 = PCLK0 = 120MHz */
#define PSFB_PWM_FREQ_HZ        (40000UL)           /* 开关频率 40kHz */
#define PSFB_DEAD_TIME_NS       (500UL)             /* 死区时间 500nS */


/*--------------------------- 计算值 ---------------------------*/
/* HRPWM 高精度寄存器格式: 1 PCLK0 周期 = 64 计数 (<<6), 低6位为小数 */
#define PSFB_PERIOD             (uint32_t)((PSFB_HRPWM_CLK_HZ / (PSFB_PWM_FREQ_HZ)-1) << 6)                         // 120000000/(40000)<<6 = 3000<<6 = 192000
#define PSFB_DT_CNT             (uint32_t)(((PSFB_HRPWM_CLK_HZ / 1000000UL) * PSFB_DEAD_TIME_NS / 1000UL) << 6)     // 60<<6 = 3840

/* 相位限幅: 5°~170°, 避免 0°(满电压冲击/偏磁) 与 180°(直通/环流损耗)
 * 主时基 HRPWM1 一整个计数周期 PSFB_PERIOD 对应 360°(T); PSFB 移相角 θ∈[0°,180°] 映射到 0~T/2,
 * 故相位计数 = (θ/360°)*PSFB_PERIOD. 若误除以 180 会使相位值翻倍, 超过半周期后硬件回绕,
 * 导致 MAX(170°) 实际等效为一个接近 0° 的小相位 */
#define PSFB_PHASE_MIN          (uint32_t)((PSFB_PERIOD * 10UL)  / 360UL)       //相移越小, 功率越大, 10°对应最大功率
#define PSFB_PHASE_MAX          (uint32_t)((PSFB_PERIOD * 160UL) / 360UL)       //相移越大, 功率越小, 160°对应最小功率

/*--------------------- HRPWM 模块定义 ---------------------*/

#define HRPWM_MAIN              (CM_HRPWM1)    // HRPWM1 为主时基模块, 产生周期事件用于触发 ADC/DMA
#define HRPWM_MASTER            (CM_HRPWM2)
#define HRPWM_SLAVE             (CM_HRPWM3)

/* Dead time value 130pS * 100000 = 13uS */
#define HRPWM_DEADTIME_VALUE            (10000UL)



/*===================== 状态机 =====================*/
typedef enum
{
    PSFB_SS_ENABLE      = (0U),        // 软启动开启
    PSFB_SS_DISABLE     = (1U),        // 软启动关闭
} psfb_state_t;

/* 软启动延时 = PSFB_SS_STEPS * PSFB_SS_DELAY_MS = 400mS */
#define PSFB_SS_STEPS    (40U)
#define PSFB_SS_DELAY_MS (10U)


/*===================== 函数声明 =====================*/
void                PSFB_SetPhase(uint32_t u32PhaseCnt);
uint32_t            PSFB_GetPhase(void);
void                PSFB_Soft_Start(psfb_state_t state);
void                PSFB_Start(void);
void                PSFB_Stop(void);

void                PSFB_BSP_Init(void);

void                PWM_Config(uint32_t u32PeriodValue, uint32_t u32CompareValue, uint32_t u32DeadTimeValue);
void                Phase_Config(uint32_t u32PhaseValue);

#ifdef __cplusplus
}
#endif

#endif /* __PSFB_H__ */
