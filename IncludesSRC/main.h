#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_comp.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_dac.h"
#include "stm32l4xx.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_lptim.h"
#include "stm32l4xx_ll_lpuart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_hal.h"
#include "time.h"
#include "string.h"
#define LED_Signal_En
#define software_vr 181012

struct Error_reg {
	uint8_t Error[10];
	uint32_t ErrorTime[10];
};
struct Error_reg Error_reg_log;


#define LED_Signal_En
#define Embedded_IMU_1
#define Embedded_IMU
#define Portable_IMU
#define Mag_Encoder_1
#define Mag_Encoder_2
#define ADC_Pressure_1
#define ADC_Pressure_2


/*
 * Scheme0: Embedded IMU, Mag Enc2, ADC1
 * Scheme1: Embedded IMU, Mag Enc2
 * Scheme2: Embedded IMU, Mag Enc2, Mag Enc1,
 * Scheme3: Embedded IMU, Mag Enc2, Mag Enc1, ADC1
 * Scheme4: Embedded IMU, Mag Enc2, Mag Enc1, ADC1, ADC2
 * Scheme5: Embedded IMU, Mag Enc2, Mag Enc1, ADC1, ADC2, Portable IMU
 * */

//#define File_name_Autoincrement


#define Highest_sensor_count 480//480     // 481-1 from Storage structure // 16kb=16384
struct Storage{
				int16_t Bat; int16_t Blank1;int16_t Blank2;int16_t Blank3; int16_t Blank4; int16_t Blank5;
				int16_t Blank6; int16_t Blank7;int16_t Blank8; int16_t Blank9; int16_t Blank10; //int16_t Blank11;
				// IMU1
				int16_t AX1[Highest_sensor_count+1]; int16_t AY1[Highest_sensor_count+1]; int16_t AZ1[Highest_sensor_count+1];
				int16_t GX1[Highest_sensor_count+1]; int16_t GY1[Highest_sensor_count+1]; int16_t GZ1[Highest_sensor_count+1];

						// knee data
				int16_t data1[Highest_sensor_count+1]; int16_t data2[Highest_sensor_count+1]; int16_t data3[Highest_sensor_count+1];
				int16_t data4[Highest_sensor_count+1]; int16_t data5[Highest_sensor_count+1]; int16_t data6[Highest_sensor_count+1];
				int16_t data7[Highest_sensor_count+1]; int16_t data8[Highest_sensor_count+1];

				int16_t Loadcel1[Highest_sensor_count+1]; int16_t Loadcel2[Highest_sensor_count+1];
				int16_t Enc2[Highest_sensor_count+1];

				uint32_t RTC_Time; uint32_t RTC_Date;
			  };


//#define Highest_sensor_count 185     // 186-1 from Storage structure
//struct Storage{
//				int16_t Bat; int16_t Blank1;int16_t Blank2;int16_t Blank3;
//
//				// IMU1
//				int16_t AX1[Highest_sensor_count+1]; int16_t AY1[Highest_sensor_count+1]; int16_t AZ1[Highest_sensor_count+1];
//				int16_t GX1[Highest_sensor_count+1]; int16_t GY1[Highest_sensor_count+1]; int16_t GZ1[Highest_sensor_count+1];
//
//				// IMU2
//				int16_t AX2[Highest_sensor_count+1]; int16_t AY2[Highest_sensor_count+1]; int16_t AZ2[Highest_sensor_count+1];
//				int16_t GX2[Highest_sensor_count+1]; int16_t GY2[Highest_sensor_count+1]; int16_t GZ2[Highest_sensor_count+1];
//
//				// IMU3
//				int16_t AX3[Highest_sensor_count+1]; int16_t AY3[Highest_sensor_count+1]; int16_t AZ3[Highest_sensor_count+1];
//				int16_t GX3[Highest_sensor_count+1]; int16_t GY3[Highest_sensor_count+1]; int16_t GZ3[Highest_sensor_count+1];
//
//				// IMU4
//				int16_t AX4[Highest_sensor_count+1]; int16_t AY4[Highest_sensor_count+1]; int16_t AZ4[Highest_sensor_count+1];
//				int16_t GX4[Highest_sensor_count+1]; int16_t GY4[Highest_sensor_count+1]; int16_t GZ4[Highest_sensor_count+1];
//
//				// IMU5
//				int16_t AX5[Highest_sensor_count+1]; int16_t AY5[Highest_sensor_count+1]; int16_t AZ5[Highest_sensor_count+1];
//				int16_t GX5[Highest_sensor_count+1]; int16_t GY5[Highest_sensor_count+1]; int16_t GZ5[Highest_sensor_count+1];
//
//
//				int16_t Flex1[Highest_sensor_count+1]; int16_t Flex2[Highest_sensor_count+1];
//				// Flex1 = instrumented side Toe;   Flex2 = instrumented side heel
//				int16_t other_fsr1[Highest_sensor_count+1]; int16_t other_fsr2[Highest_sensor_count+1];
//				//other_fsr1= un_instrumented side toe / loadcell1; other_fsr2= un_instrumented side heel / loadcell2
//
//				int16_t Enc1[Highest_sensor_count+1]; int16_t Enc2[Highest_sensor_count+1];
//				int16_t marking_sw[Highest_sensor_count+1];int16_t future_data1[Highest_sensor_count+1];
//				int16_t future_data2[Highest_sensor_count+1];int16_t future_data3[Highest_sensor_count+1];
//				int16_t future_data4[Highest_sensor_count+1];int16_t future_data5[Highest_sensor_count+1];
//				int16_t future_data6[Highest_sensor_count+1];int16_t future_data7[Highest_sensor_count+1];
//
//				uint32_t RTC_Time; uint32_t RTC_Date;
//			  };


			struct Storage BSbuffer[2];
enum Pros_states {
	Data_Log_Pause_Mode,LP_STOP,Sensor_FATFS_Write,Battery_Monitor, USB_MSC_VCP_Mode,Low_Battery_Mode,Dormant_Idle_Stop,Fatal_Error_State              // SRAM2 content = (uint32_t)0x09
				};

enum Pros_states Pros_state;


#define USB_USE_LSE_MSI_CLOCK                            // Not sure if mandatory to declare or not


#define Reset_Ram_Key_Address ((uint32_t*) ((uint32_t) 0x20017EF0))
#define Bootloader_Ram_Key_Address ((uint32_t*) ((uint32_t) 0x20017FF0))
#define Bootloader_Key_Value 0xDEADBEEF
#define Reset_Key_Value 0xDEADBEEF

#define Bottle_Ram_Key_Address ((uint32_t*) ((uint32_t) 0x20017DF0))

#define SRAM_STATE_Address ((uint32_t*) ((uint32_t) 0x10007FF0))
#define SRAM_STATE_Value 0xABCDABCD


/* Private define ------------------------------------------------------------*/

// Just Dummy
#define LED_G_PIN LL_GPIO_PIN_2
#define LED_G_GPIO_PORT GPIOA
#define LED_R_PIN LL_GPIO_PIN_3
#define LED_R_GPIO_PORT GPIOA

// Greg start comment out
// #define LED_B_PIN LL_GPIO_PIN_7
// Greg end comment out

// Greg start
#define LED_B_PIN LL_GPIO_PIN_14
#define LED_B_GPIO_PORT GPIOA
// Greg end

// Greg start comment
//#define LED_B_GPIO_PORT GPIOB
// Greg end comment

//Previous Not using any more
#define MPU_CS_GPIO_PORT GPIOA
#define MPU_CS_PIN LL_GPIO_PIN_4



//#define SD_DETECT_PIN LL_GPIO_PIN_15
//#define SD_DETECT_GPIO_PORT GPIOA

#define SD_DETECT_PIN LL_GPIO_PIN_4
#define SD_DETECT_GPIO_PORT GPIOC


#define USB_CONNECTIVITY LL_GPIO_PIN_9
#define USB_CONNECTIVITY_GPIO_PORT GPIOA

//

// Greg start comment out
//#define ENC1_CS_PIN LL_GPIO_PIN_1
//#define ENC1_CS_GPIO_PORT GPIOB
//#define ENC1_DATA_PIN LL_GPIO_PIN_2
//#define ENC1_DATA_GPIO_PORT GPIOB
//#define ENC1_SCLK_PIN LL_GPIO_PIN_10
//#define ENC1_SCLK_GPIO_PORT GPIOB
// Greg end comment out

#define ENC2_CS_PIN LL_GPIO_PIN_6
#define ENC2_CS_GPIO_PORT GPIOC
#define ENC2_DATA_PIN LL_GPIO_PIN_7
#define ENC2_DATA_GPIO_PORT GPIOC
#define ENC2_SCLK_PIN LL_GPIO_PIN_7
#define ENC2_SCLK_GPIO_PORT GPIOB

#define PUSH_BTN_Pin LL_GPIO_PIN_4
#define PUSH_BTN_GPIO_Port GPIOA
#define GROUND_TRUTH_Pin LL_GPIO_PIN_0
#define GROUND_TRUTH_GPIO_Port GPIOB



#define SPI1_SCK_IMU_PIN LL_GPIO_PIN_5
#define SPI1_SCK_IMU_GPIO_PORT GPIOA
#define SPI1_MISO_IMU_PIN LL_GPIO_PIN_6
#define SPI1_MISO_IMU_GPIO_PORT GPIOA
#define SPI1_MOSI_IMU_PIN LL_GPIO_PIN_7
#define SPI1_MOSI_IMU_GPIO_PORT GPIOA

#define SPI2_SCK_IMU_PIN LL_GPIO_PIN_13
#define SPI2_SCK_IMU_GPIO_PORT GPIOB
#define SPI2_MISO_IMU_PIN LL_GPIO_PIN_14
#define SPI2_MISO_IMU_GPIO_PORT GPIOB
#define SPI2_MOSI_IMU_PIN LL_GPIO_PIN_15
#define SPI2_MOSI_IMU_GPIO_PORT GPIOB

#define SPI3_SCK_IMU_PIN LL_GPIO_PIN_3
#define SPI3_SCK_IMU_GPIO_PORT GPIOB
#define SPI3_MISO_IMU_PIN LL_GPIO_PIN_4
#define SPI3_MISO_IMU_GPIO_PORT GPIOB
#define SPI3_MOSI_IMU_PIN LL_GPIO_PIN_5
#define SPI3_MOSI_IMU_GPIO_PORT GPIOB


#define SPI1_CS_GPIO_PORT GPIOA
#define SPI1_CS_PIN LL_GPIO_PIN_8
#define SPI2_CS_PIN LL_GPIO_PIN_12
#define SPI2_CS_GPIO_PORT GPIOB
#define SPI3_CS_PIN LL_GPIO_PIN_0
#define SPI3_CS_GPIO_PORT GPIOH

//Additional IMU
#define SPI1_IMU2_CS_Pin LL_GPIO_PIN_15
#define SPI1_IMU2_CS_GPIO_Port GPIOA
#define SPI3_IMU5_CS_Pin LL_GPIO_PIN_1
#define SPI3_IMU5_CS_GPIO_Port GPIOH

// Greg start
#define BOOST_EN_PIN LL_GPIO_PIN_10
#define BOOST_EN_GPIO_PORT GPIOB
// Greg end

#define LOADCELL2_ADC1_CH1_Pin LL_GPIO_PIN_0
#define LOADCELL2_ADC1_CH1_GPIO_Port GPIOC
#define LOADCELL1_ADC1_CH2_Pin LL_GPIO_PIN_1
#define LOADCELL1_ADC1_CH2_GPIO_Port GPIOC
#define FSR2_ADC1_IN3_Pin LL_GPIO_PIN_2
#define FSR2_ADC1_IN3_GPIO_Port GPIOC
#define FSR1_ADC1_IN4_Pin LL_GPIO_PIN_3
#define FSR1_ADC1_IN4_GPIO_Port GPIOC

#define Error_preceding LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY)


	/* Define used to indicate date/time updated */
	#define RTC_BKP_DATE_TIME_UPDTATED ((uint32_t)0x32F2)           // RTC Back up register used to check whether RTC enabled or not
	#define __RTC_CONVERT_BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))
	#define _RCC_LPTIM2_CLKSOURCE_LSE        (uint32_t)(RCC_CCIPR_LPTIM2SEL | (RCC_CCIPR_LPTIM2SEL >> 16))   /*!< LSE clock used as LPTIM2 clock source */

	// ADC variable
	 #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
	 #define VDDA_APPLI                       ((uint32_t)2800)            // ADC reference Voltage 2.8 V



#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
