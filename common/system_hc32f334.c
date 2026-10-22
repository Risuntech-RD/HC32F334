/**
 *******************************************************************************
 * @file  system_hc32f334.c
 * @brief This file provides two functions and two global variables to be called
 *        from user application.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2024-01-15       CDT             First version
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
#include "system_hc32f334.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_efm.h"
#include "hc32_ll_pwc.h"
#include "hc32_ll.h"

/**
 * @addtogroup CMSIS
 * @{
 */

/**
 * @addtogroup HC32F334_System
 * @{
 */

/*******************************************************************************
 * Global pre-processor symbols/macros ('define')
 ******************************************************************************/
/**
 * @defgroup HC32F334_System_Local_Macros HC32F334 System Local Macros
 * @{
 */
#define HRC_16MHz_VALUE                 (16000000UL)  /*!< Internal high speed RC freq. */
#define HRC_20MHz_VALUE                 (20000000UL)  /*!< Internal high speed RC freq. */
/* HRC select */
#define HRC_FREQ_MON()                  (*((volatile uint32_t *)(0x40010684UL)))

/* Vector Table base offset field */
#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET                 (0x0UL)     /*!< This value must be a multiple of 0x400. */
#endif
/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
/**
 * @addtogroup HC32F334_System_Global_Variable
 * @{
 */

/*!< System clock frequency */
__NO_INIT uint32_t SystemCoreClock;
/*!< High speed RC frequency (HRC clock) */
__NO_INIT uint32_t HRC_VALUE;

/**
 * @}
 */

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @addtogroup HC32F334_System_Global_Functions
 * @{
 */

/**
 * @brief  Setup the microcontroller system. Initialize the System and update
 *         the SystemCoreClock variable.
 * @param  None
 * @retval None
 * @note   时钟树配置：
 *         1 默认PLL源为XTAL(24MHz)启动, XTAL失败退回HRC
 *         2 启动PLL失败退回HRC(16MHz)
 * XTAL───(Y)───────────>┐
 *     └──(N)──> HRC──> PLL ─(Y)────────────────────>┐
 *                          └──(N)──> HRC(16MHz)──> SYSCLK ──(/1)──> HCLK
 * 
 */
void SystemInit(void)
{
    stc_clock_xtal_init_t   stcXtalInit;
    stc_clock_pll_init_t    stcPLLHInit;
    uint32_t                u32PllN_Value;      // 默认XTAL启动, 40倍频
    uint32_t                u32PllSrc;          // 默认XTAL源


    /* FPU settings */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22)); /* set CP10 and CP11 Full Access */
#endif

    /* Peripheral write protection off */
    LL_PERIPH_WE(LL_PERIPH_ALL);

    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL,
                    (CLK_HCLK_DIV1 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | CLK_PCLK2_DIV2 | CLK_PCLK3_DIV2 | CLK_PCLK4_DIV2));

    /* Flash read wait cycle setting (>=100MHz requires 2 wait cycles) */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE2);

    /* XTAL 24MHz configuration */
    (void)CLK_XtalStructInit(&stcXtalInit);
    stcXtalInit.u8State      = CLK_XTAL_ON;
    stcXtalInit.u8Drv        = CLK_XTAL_DRV_HIGH;
    stcXtalInit.u8Mode       = CLK_XTAL_MD_OSC;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    if (LL_OK == CLK_XtalInit(&stcXtalInit)) {          /* XTAL成功 */
        u32PllN_Value       = 30UL;
        u32PllSrc           = CLK_PLL_SRC_XTAL;         // 24MHz / 2 * 30 / 3 = 120MHz
    } else {                                            /* XTAL失败, 退回 HRC */
        if ((HRC_FREQ_MON() & 0x01UL) == 0UL) {
            HRC_VALUE       = HRC_20MHz_VALUE;          // bit0=0 -> HRC=20MHz
            u32PllN_Value   = 36UL;                     // 20MHz / 2 * 36 / 3 = 120MHz
        } else {
            HRC_VALUE       = HRC_16MHz_VALUE;          // bit0=1 -> HRC=16MHz
            u32PllN_Value   = 45UL;                     // 16MHz / 2 * 45 / 3 = 120MHz
        }
        CLK_HrcCmd(ENABLE);
        u32PllSrc           = CLK_PLL_SRC_HRC;
    }

    /* PLLH configuration: VCO = 360MHz
        XTAL 24MHz / M(2) * N(30) / P(3) = 120MHz
        HRC  16MHz / M(2) * N(45) / P(3) = 120MHz
        HRC  20MHz / M(2) * N(36) / P(3) = 120MHz
    */
    (void)CLK_PLLStructInit(&stcPLLHInit);
    stcPLLHInit.PLLCFGR             = 0UL;
    stcPLLHInit.PLLCFGR_f.PLLM      = (2UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLN      = (u32PllN_Value - 1UL);       //  PLLN = N + 1
    stcPLLHInit.PLLCFGR_f.PLLP      = (3UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLQ      = (3UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLR      = (3UL - 1UL);
    stcPLLHInit.u8PLLState          = CLK_PLL_ON;
    stcPLLHInit.PLLCFGR_f.PLLSRC    = u32PllSrc;
    if (LL_OK != CLK_PLLInit(&stcPLLHInit)) {
        CLK_SetSysClockSrc(CLK_SYSCLK_SRC_HRC);     /* PLL start failed. Use HRC as system clock source */
        // CLK_SetSysClockSrc(CLK_SYSCLK_SRC_XTAL);     /* PLL start failed. Use XTAL as system clock source */
    } else {
        CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);     /* PLL start success. Use PLL as system clock source */
    }

    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();

    /* Configure the Vector Table relocation */
    SCB->VTOR = VECT_TAB_OFFSET;    /* Vector Table Relocation */

}

/**
 * @brief  Update SystemCoreClock variable according to Clock Register Values.
 * @param  None
 * @retval None
 */
void SystemCoreClockUpdate(void)
{
    uint8_t     u8SysClkSrc;
    uint32_t    plln;
    uint32_t    pllp;
    uint32_t    pllm;
    uint32_t    u32PllSrcFreq;

    /* Select proper HRC_VALUE according to ICG1.HRCFREQSEL bit */
    if (1UL == (HRC_FREQ_MON() & 1UL)) {
        HRC_VALUE = HRC_16MHz_VALUE;
    } else {
        HRC_VALUE = HRC_20MHz_VALUE;
    }
    u8SysClkSrc = CM_CMU->CKSWR & CMU_CKSWR_CKSW;
    switch (u8SysClkSrc) {
        case 0x00U: /* use internal high speed RC */
            SystemCoreClock = HRC_VALUE;
            break;
        case 0x01U: /* use internal middle speed RC */
            SystemCoreClock = MRC_VALUE;
            break;
        case 0x02U: /* use internal low speed RC */
            SystemCoreClock = LRC_VALUE;
            break;
        case 0x03U: /* use external high speed OSC */
            SystemCoreClock = XTAL_VALUE;
            break;
        case 0x04U: /* use external low speed OSC */
            SystemCoreClock = XTAL32_VALUE;
            break;
        case 0x05U:  /* use PLLH */
            /* PLLCLK = ((pllsrc / pllm) * plln) / pllp */
            plln = (CM_CMU->PLLHCFGR & CMU_PLLHCFGR_PLLHN) >> CMU_PLLHCFGR_PLLHN_POS;
            pllp = (CM_CMU->PLLHCFGR & CMU_PLLHCFGR_PLLHP) >> CMU_PLLHCFGR_PLLHP_POS;
            pllm = (CM_CMU->PLLHCFGR & CMU_PLLHCFGR_PLLHM) >> CMU_PLLHCFGR_PLLHM_POS;
            if (0UL == bCM_CMU->PLLHCFGR_b.PLLSRC) {    /* use external high speed OSC as PLL source */
                u32PllSrcFreq = XTAL_VALUE;
            } else {                                    /* use internal high RC as PLL source */
                u32PllSrcFreq = HRC_VALUE;
            }
            SystemCoreClock = u32PllSrcFreq / (pllm + 1UL) * (plln + 1UL) / (pllp + 1UL);
            break;
        default:
            break;
    }
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/