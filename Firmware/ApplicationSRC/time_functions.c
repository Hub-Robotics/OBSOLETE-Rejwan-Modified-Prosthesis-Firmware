
/* Includes ------------------------------------------------------------------*/
#include "time_functions.h"
#include "gpio_functions.h"

extern volatile uint8_t LP_Timout;
//LPUART clock source can be HSI,LSE,PCLK,

extern uint8_t USB_Mode;
extern uint8_t Enter_Into_DFU,MSC_continue,VCP_continue;

void delay_us(uint32_t us){
    uint32_t i,k;
    for(k=0;k<us;k++)
    {
    	for(i=0;i<11;i++)
         __NOP();  // Timed at 48 MHz clock
    }
}

void Wait_for_VCP(void){
    uint32_t VCP_timer;
    MSC_continue = 0;  // USB VCP
    VCP_timer= 0;
    while(!LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))
    {
        VCP_timer++;
        if( VCP_timer > 1000000)
             {
        	MSC_continue = 1; // Time Up, Continue MSC
             break;
             }
    }
}

void Wait_for_MSC(void){
    uint32_t MSC_timer;
    VCP_continue = 0;  // USB VCP
    MSC_timer= 0;
    while(!LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))
    {
        MSC_timer++;
        if( MSC_timer > 1000) // formula= 5*1000ms= 48000*value
             {
        	VCP_continue = 1; // Time Up, Continue MSC
             break;
             }
    }
}

void Wait_for_DFU(void){
    uint32_t DFU_timer;
    DFU_timer= 0;
    Enter_Into_DFU=1;

    while(!LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))
    {
    	DFU_timer++;
        if( DFU_timer > 1000000) // formula= 5*1000ms= 48000*value
             {
        	Enter_Into_DFU = 0; // Time Up, Continue MSC
             break;
             }
    }
}

void Timeout_loop(uint32_t wait_loop_index)
{
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
}
volatile uint32_t Low_Power_delay;



void Start_LPTIMCounter2(uint32_t Auto_reload)      //Start LPTIM2 with assigned auto-reload value
{
	LL_LPTIM_Enable(LPTIM2);                          // Enable LPTIM2
    LL_LPTIM_SetAutoReload(LPTIM2, Auto_reload);      // ARR value= Desired time in sec* 32768
    LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);  // Start the Counter

}

void Start_LPTIMCounter1(uint32_t Auto_reload)      //Start LPTIM2 with assigned auto-reload value
{

	LL_LPTIM_Enable(LPTIM1);                          // Enable LPTIM2
	LL_LPTIM_EnableIT_ARRM(LPTIM1);

	NVIC_EnableIRQ(LPTIM1_IRQn);
    LL_LPTIM_SetAutoReload(LPTIM1, Auto_reload);      // ARR value= Desired time in sec* 32768
    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);  // Start the Counter
  //  EnterStop();   // Enter Stop Mode
    LP_Timout=0;
    while (LP_Timout==0)
    {
    	EnterStop();   // Enter Stop Mode
    }
    LP_Timout=0;
	NVIC_DisableIRQ(LPTIM1_IRQn);
	LL_LPTIM_Disable(LPTIM1);                          // Enable LPTIM2
}

void LSE_ON(void)
{
	/* Following 2lines are necessary before LSE
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // PWR clock is necessary for LSE
	PWR->CR1 |= PWR_CR1_DBP;  // Enable Backup access
	*/

	RCC->BDCR |= RCC_BDCR_LSEON;  // LSE ON
	while(((RCC->BDCR) & RCC_BDCR_LSERDY) != (RCC_BDCR_LSERDY)); // wait for LSE to get ready

}

void MSI_ON(void)
{
	  LL_RCC_MSI_Enable();
  while(LL_RCC_MSI_IsReady() != 1)
  {
  };
}

void MSI_OFF(void)
{
	LL_RCC_MSI_Disable();
}

void LSE_OFF(void)
{
	LL_RCC_LSE_Disable();
}


void HSI_OFF(void)
{
	LL_RCC_HSI_Disable();
}

void HSI_ON(void)
{
	LL_RCC_HSI_Enable();
	/* Wait till HSI is ready */
	while(LL_RCC_HSI_IsReady() != 1)
	{
	}
}

void EnterStop(void) // Enter Stop Mode
{
	HAL_SuspendTick();  // Before entering STOP mode, SYS_tick needs to disable
	LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);  // Enter Stop 1/2 Mode
	LL_LPM_EnableDeepSleep();               // Set SLEEPDEEP bit of Cortex System Control Register
	__WFI();                                // Request Wait For Interrupt

}



void EnterLPS(void)
{
//	LL_RCC_MSI_EnableRangeSelection();      // MSI Range select already configured
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_3); // Change clock rate
	SystemCoreClock = 800000;               // Set clock time
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);

    // SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;  /* Disable SysTick Interrupt, if necessary */
    LL_PWR_EnableLowPowerRunMode();  // witch the regulator from main mode to low-power mode

	  /* Set SLEEPDEEP bit of Cortex System Control Register */
	  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

	  /* Request Wait For Interrupt */
	  __WFI();
}

void wakeUpFromLPS (void)
{
  LL_PWR_DisableLowPowerRunMode();

}

//Configure main clock
//MSI at 48Mhz
/*
*         If define USB_USE_LSE_MSI_CLOCK enabled:
*            System Clock source            = PLL (MSI)
*            SYSCLK(Hz)                     = 80000000
*            HCLK(Hz)                       = 80000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 1
*            APB2 Prescaler                 = 2
*            MSI Frequency(Hz)              = 4800000
*            LSE Frequency(Hz)              = 32768
*            PLL_M                          = 6
*            PLL_N                          = 40
*            PLL_P                          = 7
*            PLL_Q                          = 4
*            PLL_R                          = 4
*            Flash Latency(WS)              = 4
*/

void SystemClock_Config_MSI_80MHz(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);   	// Enable Power Clock necessary for LSE, RTC, LPTIM
	  LL_PWR_EnableBkUpAccess();                              // Enable Backup access (needed for LSE clock)
	  LL_PWR_EnableVddUSB();                                  // Enable VDDUSB supply for USB
	  while(!LL_PWR_IsEnabledVddUSB());                       // Wait for VDDUSB supply to activate

	  /* Enable MSI Oscillator and activate PLL with MSI as source */
	  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
	//  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
	  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
	  RCC_OscInitStruct.PLL.PLLM            = 6;
	  RCC_OscInitStruct.PLL.PLLN            = 40;
	  RCC_OscInitStruct.PLL.PLLP            = 7;
	  RCC_OscInitStruct.PLL.PLLQ            = 4;
	  RCC_OscInitStruct.PLL.PLLR            = 4;

	  HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSECSSON) ;
	  CLEAR_BIT(RCC->CIER, (RCC_IT_LSECSS));

	  LSE_ON();                                                     //Start LSE for RTC and LPTIM timer
	  HAL_RCCEx_EnableMSIPLLMode();


	  LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_MSI);
	  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_MSI);
	  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2); //1
	  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

	  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
	  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4);

	  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);  // MSI clock is set as default clock after wake up from Stop
	  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();               // Disable USB clock to save power

		}


void SystemClock_Config_MSI_48MHz(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);   	// Enable Power Clock necessary for LSE, RTC, LPTIM
	  LL_PWR_EnableBkUpAccess();                              // Enable Backup access (needed for LSE clock)
	  LL_PWR_EnableVddUSB();                                  // Enable VDDUSB supply for USB
	  while(!LL_PWR_IsEnabledVddUSB());                       // Wait for VDDUSB supply to activate

	  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
	  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11; // For 48 MHz MSI
	  HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSECSSON) ;
	  CLEAR_BIT(RCC->CIER, (RCC_IT_LSECSS));

	  LSE_ON();                                                     //Start LSE for RTC and LPTIM timer

	  LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_MSI);
	  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_MSI);
	  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

	  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2);

	  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);  // MSI clock is set as default clock after wake up from Stop
	  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();               // Disable USB clock to save power

		}

//Configure LPTIM2 to generate the XX Hz interrupt
void Configure_LPTIM2_Int(void)
{
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM2_CLKSOURCE_LSE);  // SET LSE as LPTIM2 source
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPTIM2);     // Enable LPTIM2 Clock
	NVIC_DisableIRQ(LPTIM2_IRQn);
	LL_LPTIM_DisableIT_ARRM(LPTIM2);                           // Enable auto reload match interrupt (ARRMIE).
	LL_LPTIM_Disable(LPTIM2);                                // LPTIM2 disable if needed

}

//Configure LPTIM2 to generate the XX Hz interrupt
void Configure_LPTIM1_Int(void)
{
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSE);  // SET LSE as LPTIM2 source
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);     // Enable LPTIM2 Clock
	NVIC_DisableIRQ(LPTIM1_IRQn);
	LL_LPTIM_DisableIT_ARRM(LPTIM1);                           // Enable auto reload match interrupt (ARRMIE).
	LL_LPTIM_Disable(LPTIM1);                                // LPTIM2 disable if needed

}

