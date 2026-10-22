#ifndef __HC32F3XX_CONF_H__
#define __HC32F3XX_CONF_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define LL_ICG_ENABLE                               (DDL_ON)
#define LL_UTILITY_ENABLE                           (DDL_ON)
#define LL_PRINT_ENABLE                             (DDL_OFF)

#define LL_ADC_ENABLE                               (DDL_ON)        // 启用 ADC 模块
#define LL_AOS_ENABLE                               (DDL_ON)        // 启用 AOS 事件系统(HRPWM->ADC触发依赖)
#define LL_CLK_ENABLE                               (DDL_ON)
#define LL_CMP_ENABLE                               (DDL_OFF)
#define LL_CRC_ENABLE                               (DDL_OFF)
#define LL_CTC_ENABLE                               (DDL_OFF)
#define LL_DAC_ENABLE                               (DDL_OFF)
#define LL_DBGC_ENABLE                              (DDL_OFF)
#define LL_DMA_ENABLE                               (DDL_ON)        // 启用 DMA 模块
#define LL_EFM_ENABLE                               (DDL_ON)
#define LL_EMB_ENABLE                               (DDL_OFF)
#define LL_EVENT_PORT_ENABLE                        (DDL_OFF)
#define LL_FCG_ENABLE                               (DDL_ON)
#define LL_FCM_ENABLE                               (DDL_OFF)
#define LL_HRPWM_ENABLE                             (DDL_ON)        // 启用 HRPWM 模块
#define LL_GPIO_ENABLE                              (DDL_ON)
#define LL_I2C_ENABLE                               (DDL_OFF)
#define LL_INTERRUPTS_ENABLE                        (DDL_ON)
#define LL_INTERRUPTS_SHARE_ENABLE                  (DDL_ON)
#define LL_MCAN_ENABLE                              (DDL_OFF)
#define LL_MPU_ENABLE                               (DDL_OFF)
#define LL_PLA_ENABLE                               (DDL_OFF)
#define LL_PWC_ENABLE                               (DDL_ON)
#define LL_RMU_ENABLE                               (DDL_OFF)
#define LL_RTC_ENABLE                               (DDL_OFF)
#define LL_SPI_ENABLE                               (DDL_OFF)
#define LL_SRAM_ENABLE                              (DDL_ON)
#define LL_SWDT_ENABLE                              (DDL_OFF)
#define LL_TMR0_ENABLE                              (DDL_OFF)
#define LL_TMR4_ENABLE                              (DDL_OFF)
#define LL_TMR6_ENABLE                              (DDL_OFF)
#define LL_TMRA_ENABLE                              (DDL_ON)
#define LL_USART_ENABLE                             (DDL_ON)
#define LL_WDT_ENABLE                               (DDL_OFF)

#define BSP_EV_HC32F334_LQFP64                      (1U)
#define BSP_EV_HC32F3XX                             (BSP_EV_HC32F334_LQFP64)

#define BSP_24CXX_ENABLE                            (DDL_OFF)
#define BSP_W25QXX_ENABLE                           (DDL_OFF)
#define BSP_INT_KEY_ENABLE                          (DDL_OFF)

#ifdef __cplusplus
}
#endif

#endif