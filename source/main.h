/**
 * @file main.h
 * @author your name (you@domain.com)
 * @brief 主函数头文件
 * @version 0.1
 * @date 2026-10-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>
#include "psfb.h"
#include "hc32_ll.h"

static void     App_PortCfg(void);
static void     Log_SendString(const char *pcStr);
static void     Delay_ms(volatile uint32_t u32Ms);
static void     debug_output(void);

static void     App_PortCfg(void);
static void     App_Tmra2Cfg(void);
void            TMRA_2_Ovf_Udf_Handler(void);
static void     App_USARTxCfg(void);

#endif