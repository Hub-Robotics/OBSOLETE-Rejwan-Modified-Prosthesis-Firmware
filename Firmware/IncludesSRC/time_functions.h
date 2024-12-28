/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIME_FUNC_H
#define __TIME_FUNC_H

#include "main.h"

void SystemClock_Config_MSI_48MHz(void);
void Delay_10ms_Multiple (uint32_t Delay);
void Timeout_loop(uint32_t wait_loop_index);
void SystemClock_Config_MSI_80MHz(void);

void delay_us(uint32_t us);
void Wait_for_VCP(void);
void Wait_for_DFU(void);
void Wait_for_MSC(void);

void LSE_ON(void);
void LSE_OFF(void);

void MSI_ON(void);
void MSI_OFF(void);

void HSI_ON(void);
void HSI_OFF(void);

void EnterStop(void);
void EnterLPS(void);
void wakeUpFromLPS (void);

// Low Power Mode
void EnterStopMode(void);
void wakeUpFromLPsleep (void);
void EnterLPS(void);

//LPTIM2
void Configure_LPTIM2_Int(void);
void Configure_LPTIM1_Int(void);
void Start_LPTIMCounter1(uint32_t Auto_reload);
void Start_LPTIMCounter2(uint32_t Auto_reload);

#endif /* __GPIO_FUNC_H */
