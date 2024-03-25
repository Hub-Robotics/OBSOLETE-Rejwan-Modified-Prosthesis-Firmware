#include "main.h"
#include "fatfs.h"
#include "gpio_functions.h"
#include "time_functions.h"
#include "sensor.h"
#include "peripherals.h"
#include "usbd_storage.h"
#include <math.h>
// SDFG
#include "controller.h"
#include "StateFormulas.h"


// Greg Start
#include "MadgwickAHRS.h"
// Greg End

#include "CAN.h"
#include "EPOS4.h"
#include "mpu9255.h"
#include "knee_control.h"

int CAN_ID = 0x601;

#define torque_const_kv100lite 0.095
#define gear_ratio_Chain_knee 40
#define peak_current 19 // Amp set in epos
#define nominal_current 8 // Amp
#define rad2deg 180/3.14


#define max_torque_cmd 400
#define max_CST_CMD_EPOS 400

/*filter coeff*/
#define coeff_x 0.0689
#define coeff_y 0.8621

/*PID*/
#define kp 65//65   //40
#define kd .68  //.68
#define kI 0
#define set_position 25

#define AIM_Start_Data_Collection_on_Reset   // For Debug Purpose, AIM Datalog will start at power on reset
int data111=0,data222=0;
#define Battery_3200mV 2130

uint8_t SD_cnt_limit, Data_log_Start_Resume,ADC_Busy_with_Battery_Monitor,ErrorCodeBuf;
uint32_t ErrorTimeStampBuf;
extern uint16_t Battery_ADC_Value;
extern uint8_t Err_A,Err_Pr,Err_SD,Err_IMG,Err_Cam,Err_FPGA; // Error Code to display in Debug message // all needs to be Volatile
extern uint8_t FATAL_Error,ACC_Reset_Ongoing;

// Motion blur detection
int16_t volatile calcParam = 0;  // Calculated parameter from acc values
uint8_t volatile measure = 0;    // Flag if above parameter needs to be calculated (Calculated only during image capture)
uint8_t volatile measureState = 0;
int16_t volatile calcParam2 = 0;
int16_t volatile calcParam3 = 0;

// Camera stages measurement
uint8_t volatile camStage = 0;
uint8_t volatile camfps = 0;   /* Motion Based Imaging Ends here*/

// USB Parameters
USBD_HandleTypeDef USBD_Device;
extern PCD_HandleTypeDef hpcd;
extern USBD_CDC_ItfTypeDef USBD_CDC_fops;

// Flag to be set in interrupt handlers
volatile uint8_t USB_Present_=0,DataLog_Pause_=0;
volatile uint32_t TimeOut_For_Capture;
volatile uint8_t Take_Image;

// FATFS SD Flag to be set in interrupt handlers
volatile uint32_t Sub_cnt=0; 		// Storage Buffer Index Needed in LPTIM2 Interrupt
volatile uint8_t s_flag = 0 ; 		// flag 1 for Buffer one, 2 for Buffer two
volatile uint8_t w_flag = 1 ; 		// flag 1 for Buffer one, 2 for Buffer two
volatile uint8_t SD_write_Flag; 	// SD Card write Flag
volatile uint8_t SD_Write_Count=0;  // File Write Count
/**/

struct st_impedance my_st_impedance;

#define fc 10
float T= (float) 1/640; // interrupt duration
extern volatile uint16_t Test1,Test2,Test1_mV,Test2_mV,Test3,Test4,Test3_mV,Test4_mV,Test5,Test6,Test5_mV,Test6_mV;
extern int16_t bit_data[10],bit_data1[10],bit_data2[10],bit_data3[10];
uint8_t PrintBuf[50],PrintBuf1[50];

// Greg start
uint8_t isProcessKneeRequired = 0;
// Greg end


int main(void) {
	Pros_state = LP_STOP;                  // Default state after power ON reset
	SystemClock_Config_MSI_80MHz();	// Configure the system clock to 48 MHz from MSI, PLL disabled for power saving

	// If RTC is previously configured and running via backup battery i.e. BackUp Register Data is available, No need to update RTC time again
	if (RTC_BAK_GetRegister(RTC, 0x01) != RTC_BKP_DATE_TIME_UPDTATED) //    0x01 value was arbitrarily set from previous Configure_RTC_Calendar()
	{
		MX_RTC_Init();   // Set RTC clock source and parameters
	}

	MX_GPIO_Init();
	HAL_Init(); // Reset of all peripherals, Initializes the Flash interface and the Systick.

	DFU_Bypass();
	Configure_USART_1();  // Debug with PC
	sprintf(PrintBuf, "Hello");
	USART1_wr_print(PrintBuf, sizeof(PrintBuf));
	systick_app_timer_module_init();
	/*Important information: which IMU interfaced to which SPI */
//IMU1-2 SPI1 .
//IMU3   SPI2
//IMU4-5 SPI3
	/*Sensor Initialization starts here */

// P_IMU4_SPI3_Initialization_at_reset();   //IMU4-5_SPI3 //step1
	//P_IMU1_SPI1_Initialization_at_reset(); //IMU1-2__SPI1  (only IMU1 configured)
	mpu9255_init(10);
	readTimer_event_handler();
	P_ADC_Sensor_GPIO_Init(); //ADC GPIOs //here we initialized the chip select pins as well

	/*CAN Bus SPI Initialization*/
//	MCP_SPI2_Initialization_at_reset();

	//Configure the mcp25625, CAN bus and SPI2
	CAN_configure();

	//Sets can mode currently set to normal mode
	//CAN_mode();

	//Transmits a message over can
//clear state

	EPOS4_enable(CAN_ID);
	//EPOS4_set_operation_mode(CAN_ID, 3); // this for velocity mode

	EPOS4_set_operation_mode(CAN_ID, 0x0A); // torque mode
	EPOS4_clear_errors(CAN_ID);
	delay_us(1500);

	EPOS4_enable(CAN_ID);
//EPOS4_enable2(CAN_ID);

	/* remove spikes from the beginning part*/
	for (int jj = 1; jj < 1000; ++jj);


	USB_PA9_EXTI_conf(); // USB connectivity pin detect Interrupt // Data_Pause_Resume_PC0_EXTI_conf();

	Configure_LPTIM2_Int(); // Configured LPTIM2 but not started. To be started before going to Loop
	Configure_Interrupt();       // Re-arrange NVIC interrupt priority

	//Power_on_reset();            // Following reset is found by troubleshooting

#ifdef AIM_Start_Data_Collection_on_Reset
	AIM_DataStart_at_Reset();
	GREEN_LED_ONLY();

#else
	Start_LPTIMCounter2(0x1F);           // Start 1ms LPTIM interrupt for
	SD_cnt_limit=100;// After 100 write, File will Sync
	Prepare_Goto_Dormant_Mode();
	Pros_state = Dormant_Idle_Stop;
#endif

//	while (1) {
//		mpu9255_process();
//	}
	// USB Default mode is USB VCP
	// Note: Data collection is stopped in Power on Reset. Send the command from PC LabVIEW software in USB VCP Mode to start data collection.
	// Data collection will resume after USB disconnect.
	// Only way to stop Data collection is by accessing SD card from PC LabVIEW program

	while (1) {

		if (isProcessKneeRequired) {
			processKnee();
			isProcessKneeRequired = 0;
		}
		mpu9255_process();
		switch (Pros_state) {
		case LP_STOP:      // Default mode for data collection
			EnterStop();   // Enter Stop Mode
//						angle_now=knee_angle();

//						  RED_LED_ONLY();
//						  delay_us(500000);
//						  GREEN_LED_ONLY();
//						  delay_us(500000);
//						EPOS4_CST_apply_torque(CAN_ID,100);
//						EPOS4_CST_apply_torque(CAN_ID,100);

			// Wake Up after STOP Mode by LPTIM2_interrupt or USB EXTI Int

			//		Non_critical_Sensor_Error_Handling();  // Handling errors occured in Data log
			/*						if (DataLog_Pause_ == 1)                     // After detecting DataLogPause Command via EXTI_0 int
			 {
			 Pros_state = Data_Log_Pause_Mode;               // Next State Data_Log_Pause_Mode Mode
			 DataLog_Pause_ = 0;                           // Reset Flag to avoid looping
			 }*/

			if (USB_Present_ == 1) // After detecting USB attachment via EXTI_5 int
					{
				Pros_state = USB_MSC_VCP_Mode;   // Next State USB_MSC_Mode Mode
				USB_Present_ = 0;                 // Reset Flag to avoid looping
			} else if (FATAL_Error == 1)         // Triggered from Error_Handler
					{
				FATAL_Error = 0;                  // Reset Flag to avoid looping
				Pros_state = Fatal_Error_State;  // Next State Fatal_Error_State
			} else if (SD_write_Flag == 1) // When BUffer full to store in SD card
					{
				Pros_state = Sensor_FATFS_Write; // Next State Sensor_FATFS_Write Mode
				SD_write_Flag = 0;                // Reset Flag to avoid looping
			}

			else {
				Pros_state = LP_STOP; // Return to Low Power Mode Data collection
			}
			break;

		case Sensor_FATFS_Write:

			Battery_ADC_Value = 2500;
			if (Battery_ADC_Value < Battery_3200mV) // Battery Cut off Value 3.0V
			{
				Pros_state = Low_Battery_Mode; // Prepare to enter Low_Battery_Mode mode
			} else {
				SD_Sensor_write();             // Write Sensor Buffer to SD card
				if (USB_Present_ == 1) // If USB attachment detected here via EXTI_5 int
						{
					Pros_state = USB_MSC_VCP_Mode; // Next State USB_MSC_Mode Mode
					USB_Present_ = 0;             // Reset Flag to avoid looping
				} else if (FATAL_Error == 1)     // Triggered from Error_Handler
						{
					FATAL_Error = 0;              // Reset Flag to avoid looping
					Pros_state = Fatal_Error_State; // Next State Fatal_Error_State
				} else {
					Pros_state = LP_STOP; // Return to Low Power Mode Data collection
				}
			}
			break;

		case Low_Battery_Mode:
			NVIC_DisableIRQ(LPTIM2_IRQn); // Disabling LPTIM2 Timer Interrupt to stop Data collection
			ALL_LED_OFF();
			Shut_Down_SD();

			Reset_Variables_for_LowBattery();
			Prepare_Goto_Dormant_Mode();
			Pros_state = Dormant_Idle_Stop;       // Idle Lowest Power STOP Mode

			break;

		case Fatal_Error_State:
			RED_LED_ONLY();

			FATFS_Logstart_Delete();
			Data_log_Start_Resume = 0;

			SD_write_Flag = 0;
			NVIC_DisableIRQ(LPTIM2_IRQn);

			LL_LPTIM_DisableIT_ARRM(LPTIM2);
			SD_POWER_OFF();
			LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC);

			EnterStop();   // Enter Stop Mode
			// Wake Up after STOP Mode only by USB EXTI Int

			if (USB_Present_ == 1) // After detecting USB attachment via EXTI_5 int
					{
				Pros_state = USB_MSC_VCP_Mode;   // Next State USB_MSC_Mode Mode
				USB_Present_ = 0;                 // Reset Flag to avoid looping
				SD_POWER_ON();               				// Power on SD CARD
			} else {
				Pros_state = Dormant_Idle_Stop; // Stay Dormant Mode if no external events
			}
			break;

		case USB_MSC_VCP_Mode:

			USB_Init_Start(); // Initialize USB and Stay USB mode as long as USB cable connected

			break;

		case Dormant_Idle_Stop: // Idle Lowest Power Stop Mode with no Data log (only wait for USB connectivity)

			EnterStop();   // Enter Stop Mode
			// Wake Up after STOP Mode only by USB EXTI Int

			if (USB_Present_ == 1) // After detecting USB attachment via EXTI_5 int
					{
				Pros_state = USB_MSC_VCP_Mode;   // Next State USB_MSC_Mode Mode
				USB_Present_ = 0;                 // Reset Flag to avoid looping
				SD_POWER_ON();               				// Power on SD CARD
			} else {
				Pros_state = Dormant_Idle_Stop; // Stay Dormant Mode if no external events
			}
			break;

		default:
			break;

		}
	}
}


void LPTIM2_IRQHandler(void)   // Response of 10ms LPTIM interrupt
{
	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2) == 1)    // auto reload match interrupt has occured
	{
		isProcessKneeRequired = 1;
		LL_LPTIM_ClearFLAG_ARRM(LPTIM2);    // Clear ARR interrupt flag
	}
}


void EXTI9_5_IRQHandler(void)    // Interrupt from USB connectivity PIN PA9
{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
	{
		USB_Present_=1;                            // USB present
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);    // Clear interrupt
	}
}
