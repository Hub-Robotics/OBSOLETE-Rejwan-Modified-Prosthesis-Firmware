#include "sensor.h"
#include "fatfs.h"
#include "main.h"
#include <math.h>

#include "gpio_functions.h"
#include "time_functions.h"
#include "peripherals.h"
#include <inttypes.h>

// Greg start
static uint8_t count = 1;
static float last_gyro;
// Greg end

//new
uint8_t FATAL_Error, ACC_Reset_Ongoing, Skip_FATFS, File_Sensor_write_issue,
		File_Sensor_close_issue;
int16_t xyzValB[6] = { 0, 0, 0, 0, 0, 0 };  // temporary register
#define SPI_TimeOut_Limit 4800000
#define SPI_TimeOut_Limit_Buf 4800000
#define ADC_TimeOut_Limit 4800000
#define ADC_TimeOut_Limit_Buf 4800000

#define LP_Ram_Key_Address ((uint32_t*) ((uint32_t) 0x20017CF0))

//new ends here
#define AK8963FASTMODE

extern int SD_cnt_limit;
uint32_t SPI_TimeOut_Count, ADC_TimeOut_Count;
uint8_t SPI_Error, ADC_Error, Error_indx, ADC_ErrorCnt, SPI_ErrorCnt;

volatile uint32_t SPI_TimeOut_Count_Buf;
volatile uint8_t SPI_Error_Buf, NonCritical_Sensor_Error;

volatile uint32_t ADC_TimeOut_Count_Buf;
volatile uint8_t ADC_Error_Buf;

int last_file;

extern int Logstart_Create, Logstart_Delete, EnterDFU_Create, EnterReset_Create;

extern uint32_t IMG_sz;

/*File Error Variables*/

#define f_open_E 1
#define f_read_E 2
#define f_mount_E 3
#define f_sync_E 4
#define f_write_E 5
#define f_writeC_E 6
#define f_openC_E 7
#define SPI_Acc_Error 8
#define SPI_Gyro_Error 9
#define I2C_Camera_Error 10
#define I2C_Pres_Error 11
#define ADC_R_Error 12
#define Low_Bat_Error 13
#define I2C_Camera_Init_Error 14
#define FPGA_Response_Error 15
#define FPGA_SPI_R_Error 16
/*File Error Variables Ends here*/

int FATFS_attempt, Fat_mnt_fail, FATAL_err, Fat_read_mnt_fail,
		Fat_write_read_fail, Fat_sync_read_fail;

extern volatile uint8_t USB_Present_;
extern volatile uint32_t TimeOut_For_Capture;
volatile uint16_t Strain_ADC_Value, Strain_ADC_Value_mV;
volatile uint8_t PressureBuffer[2];

uint16_t Battery_ADC_Value;
uint8_t USB_Mode, Acc_Error_ZeroValue, Enter_Into_DFU, MSC_continue, VCP_Bypass,
		VCP_continue, MSC_Bypass;
int last_file;
extern int First_Imaging_after_Camera_ON, last_file_camera, Logstart_Create,
		Logstart_Delete, EnterDFU_Create, EnterReset_Create;
extern uint8_t Skip_I2C, Data_log_Start_Resume, ADC_Busy_with_Battery_Monitor,
		ErrorCodeBuf;
extern uint32_t ErrorTimeStampBuf;

//volatile uint16_t Interval_Between_Image_Capture;

extern volatile uint8_t USB_Present_;
extern uint8_t ADC_Busy_with_Battery_Monitor;
extern volatile uint32_t TimeOut_For_Capture;
extern uint8_t USB_Mode;
extern uint8_t Data_log_Start_Resume;
volatile uint16_t Strain_ADC_Value, Strain_ADC_Value_mV;
uint16_t Battery_ADC_Value;
volatile uint8_t PressureBuffer[2];

extern volatile uint32_t Sub_cnt; // Storage Buffer Index Needed in LPTIM2 Interrupt
extern volatile uint8_t s_flag; 	// flag 1 for Buffer one, 2 for Buffer two
extern volatile uint8_t w_flag; 	// flag 1 for Buffer one, 2 for Buffer two
extern volatile uint8_t SD_write_Flag;		// SD Card write Flag
extern volatile uint8_t SD_Write_Count;  // File Write Count

FRESULT res;
FATFS SDFatFs;                   // File system object for SD card logical drive
FILINFO fno;                            // FATFS file information
FIL MyFile;                             // File object
char SDPath[4];                         // SD card logical drive path

char FILE_NAME_STRING[16];                 //String to hold file name
unsigned int BytesWritten1, BytesWritten2;  // File write/read counts
char textString[32];                       // Character sting for text output
extern USBD_HandleTypeDef USBD_Device;    // USB Variables

char RTC_String[32];                      // Write RTC timestamps in strings
uint8_t Hour, Min, Sec, SubSec, Year, Mon, Day; // variables to save RTC time values

void Non_critical_Sensor_Error_Handling(void) {
	if (NonCritical_Sensor_Error == 1) {
		NonCritical_Sensor_Error = 0;

		if (SPI_Error_Buf == 1) {
			AIM_Error_Handler(SPI_Acc_Error);
		}
		if (ADC_Error_Buf == 1 || ADC_Error == 1) {
			AIM_Error_Handler(ADC_R_Error);
		}

	}

}

void Power_on_reset(void) {
	if (*Reset_Ram_Key_Address != Reset_Key_Value) {
		*Reset_Ram_Key_Address = Reset_Key_Value;
		NVIC_SystemReset();
	}
}

void Configure_Interrupt(void) {
	NVIC_SetPriorityGrouping(0x00); // If Group 00, only priority number Matters: Low Number high priority

	//  NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_SetPriority(EXTI9_5_IRQn, 0); // USB Detect, Low priority
	NVIC_SetPriority(SDMMC1_IRQn, 2);
	NVIC_SetPriority(LPTIM2_IRQn, 1);
}

void FATFS_Init(void) {

	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
			{
		if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
				{
			Fat_mnt_fail = 1;
		}
	} else {
		Fat_mnt_fail = 1;
	}

	if (Fat_mnt_fail == 1)    // Could not Link driver in 2nd Attempt
			{
		AIM_Error_Handler(f_mount_E);
	}

}

void Check_SD_Command_File(void) {
	Data_log_Start_Resume = 0;
	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
			{
		if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
				{
			Fat_mnt_fail = 1;
		} else {
			if (f_open(&MyFile, "EnterDFU.txt", FA_READ) != FR_OK) // Check the presence of EnterDFU.txt file
					{
				if (f_open(&MyFile, "EnterVCP.txt", FA_READ) != FR_OK) // Check the presence of EnterVCP.txt file
						{
					if (Logstart_Delete == 1) {
						Logstart_Delete = 0;
						if (f_unlink("LogStart.txt") == FR_OK) { // If LogStart.txt file present, delete the file
						}
					}

					else {

						if (f_open(&MyFile, "LogStart.txt", FA_READ) != FR_OK) // Check the presence of LogStart.txt file
								{
							Data_log_Start_Resume = 0;         // Data Log Stops
						} else {
							Data_log_Start_Resume = 1; // Data Log Starts/Resume
							f_close(&MyFile);
						}
					}
				} else {
					Data_log_Start_Resume = 0;          // Data Log Stops

					USB_Mode = 0;       // USB VCP mode in Next USB Connectivity
					f_close(&MyFile);
					if (f_unlink("EnterVCP.txt") == FR_OK) { // If EnterVCP.txt file present, delete the file
					}
				}
			} else {

				f_close(&MyFile);
				if (f_unlink("EnterDFU.txt") == FR_OK) { // If EnterDFU.txt file present, delete the file
				}
				f_mount(0, "", 0); 		     //  unmount FATFS file if necessary
				FATFS_UnLinkDriver(SDPath); //  unlink SD card driver if necessary

				*Bootloader_Ram_Key_Address = Bootloader_Key_Value; // Write a key to a RAM location to check at next reset
				NVIC_SystemReset();        // System reset
			}
			f_mount(0, "", 0); 		     //  unmount FATFS file if necessary
			FATFS_UnLinkDriver(SDPath);  //  unlink SD card driver if necessary
		}
	} else {
		Fat_mnt_fail = 1;
	}

	if (Fat_mnt_fail == 1)    // Could not Link driver in 2nd Attempt
			{
		AIM_Error_Handler(f_mount_E);
	}
}

void Shut_Down_SD(void) {
	f_close(&MyFile);                       // Close previous running file
	update_FATFS_time(); // Write RTC time-stamp on File header/properties of last FATFS file to visualize in PC
	f_mount(0, "", 0); 	//  unmount FATFS file (of running sensor storage file)
	FATFS_UnLinkDriver(SDPath); //  unlink SD card driver (of running sensor storage file)
}

void Shut_Down_USB(void) {

	LL_PWR_DisableVddUSB();                     // Disable VDDUSB supply for USB
	USB_Present_ = 0;  // reset the USB present flag to be set by EXTI interrupt

	USBD_Stop(&USBD_Device);                 // Stop the USB Device Core.
	USBD_DeInit(&USBD_Device);               // De-Initialize the device library
	__HAL_RCC_USB_OTG_FS_CLK_DISABLE()
	;      // Disable USB Clock
}

void File_Close_Update_Unlink(void) {
	f_close(&MyFile);                       // Close previous running file
	update_FATFS_time(); // Write RTC time-stamp on File header/properties of last FATFS file to visualize in PC
	f_mount(0, "", 0); 	//  unmount FATFS file (of running sensor storage file)
	FATFS_UnLinkDriver(SDPath); //  unlink SD card driver (of running sensor storage file)
}

void USB_Clock_Ready(void) {
	LL_PWR_EnableVddUSB();                       // Enable VDDUSB supply for USB
	while (!LL_PWR_IsEnabledVddUSB())
		;                       // Wait for VDDUSB supply to activate
	__HAL_RCC_USB_OTG_FS_CLK_ENABLE()
	;     // Enable USB Clock
}

void Datalog_Sensor_Initialization(void) {

	FATFS_Init();
	Open_File_For_Sensor_Write();
	// Re-Initializing FATFS (UnixTimestamp .BIN file) and Left open for sensor write
	if (Skip_FATFS == 0) {
		Reset_All();         // Reset All Buffer parameters for FATFS SDIO write
	}
}

void Prepare_Data_Log_State(void) {
	if (Skip_FATFS == 0)   // If there's no FATFS SD error
			{
//	Battery_Monitor_Voltage_check();
		Battery_ADC_Value = 2500;
		if (Battery_ADC_Value < 2450)           // Battery high charge 3.8V 2400
				{

			Pros_state = Low_Battery_Mode; // Prepare to enter Low_Battery_Mode mode
		} else {
			/*
			 if (Data_log_Start_Resume == 1)       // Continue Data Log after USB disconnect
			 {

			 if (__RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC))!=18)  // If Date is not updated
			 {
			 Data_log_Start_Resume = 0;
			 //    Red_LED_Blinking_2s();
			 Pros_state = Low_Battery_Mode;
			 if (LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))  Pros_state = USB_MSC_VCP_Mode;

			 }

			 }
			 */
			if (Data_log_Start_Resume == 1) // Continue Data Log after USB disconnect
					{

				Datalog_Sensor_Initialization();

				if (Skip_FATFS == 0)   // If there's no FATFS SD error
						{

					LL_LPTIM_EnableIT_ARRM(LPTIM2);
					NVIC_EnableIRQ(LPTIM2_IRQn); // Enabling LPTIM2 Timer 10ms Interrupt

				}
			}

			else if (VCP_Bypass == 1) {
				VCP_Bypass = 0;
				Pros_state = LP_STOP;
			} else   // Go to lowest power IDLE mode, no data log necessary
			{
				prepare_low_battery();
				//ALL_LED_OFF();

				Prepare_Goto_Dormant_Mode();
				Pros_state = Dormant_Idle_Stop;
			}

		}
	}
}

void Common_USB_Job(void) {
	Battery_Monitor_Voltage_check();
	//ALL_LED_OFF();
	if (Battery_ADC_Value < 2300)               // Battery high charge 3.8V 2600
			{
		//   Data_log_Start_Resume = 0;
		//    Yellow_LED_Blinking_2s();
		//     YELLOW_LED_ONLY();
		Pros_state = Low_Battery_Mode; // Prepare to enter Low_Battery_Mode mode
	} else {
#ifdef Bottle_Device
		Data_log_Start_Resume = 0;
		Pros_state = USB_MSC_VCP_Mode; // Prepare to enter Low_Battery_Mode mode
		while (!LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))
		{
			LL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			LL_mDelay(200);
		}

#else
		/*
		 if (Data_log_Start_Resume == 1)       // Continue Data Log after USB disconnect
		 {

		 if (__RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC))!=18)
		 {
		 Data_log_Start_Resume = 0;
		 // 	    Red_LED_Blinking_2s();

		 Pros_state = Low_Battery_Mode;
		 }
		 }
		 */
		if (Data_log_Start_Resume == 1) // Continue Data Log after USB disconnect
				{

//	 GREEN_LED_ON();// Green LED indicates Data log starts
			FATFS_Init();
			Open_File_For_Sensor_Write();
			// Re-Initializing FATFS (UnixTimestamp .BIN file) and Left open for sensor write
			Reset_All();     // Reset All Buffer parameters for FATFS SDIO write

			LL_LPTIM_EnableIT_ARRM(LPTIM2);
			NVIC_EnableIRQ(LPTIM2_IRQn); // Enabling LPTIM2 Timer 10ms Interrupt

		} else if (VCP_Bypass == 1) {
			VCP_Bypass = 0;
			Pros_state = LP_STOP;
		} else {
			prepare_low_battery();
			//ALL_LED_OFF();

			Prepare_Goto_Dormant_Mode();
			//ALL_LED_OFF();                 // No data log continues
			Pros_state = Dormant_Idle_Stop;
		}
#endif
	}
}
int check;
void DFU_Bypass(void) {

	uint32_t i = 0;

	check = LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY);
	// if(check == 1)
	//	 CYAN_LED_ONLY();
	while (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY))
		;  // Wait until USB cable disconnect from Charger
	if (*LP_Ram_Key_Address != Reset_Key_Value) {
		*LP_Ram_Key_Address = Reset_Key_Value;
		if (check == 1) {
			/* Toggle IO in during 2s (10*200ms) */
			for (i = 0; i < 5; i++) {
				//   LL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				LL_mDelay(200);
				if (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY)) {
					*Bootloader_Ram_Key_Address = Bootloader_Key_Value; // Write a key to a RAM location to check at next reset
					NVIC_SystemReset();        // System reset
				}

			}
		}
	}
}

void DFU_Bypass_Bottle_Sensor(void) {
	uint32_t i = 0;
	check = LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY);
	while (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY))
		;  // Wait until USB cable disconnect from Charger
	if (check == 1) {
		/* Toggle IO in during 2s (10*200ms) */
		for (i = 0; i < 5; i++) {
			LL_mDelay(200);
			if (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY)) {
				*Bootloader_Ram_Key_Address = Bootloader_Key_Value; // Write a key to a RAM location to check at next reset
				NVIC_SystemReset();        // System reset
			}

		}
		//  BLUE_LED_ONLY();
		*Bottle_Ram_Key_Address = Reset_Key_Value;
	}
}

void Bypass_Datalog_Bottle(void) {
	*Bottle_Ram_Key_Address = 0x00000000;
	Pros_state = USB_MSC_VCP_Mode;                // Active Data Collection Mode
	while (!LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY)) {
		LL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		LL_mDelay(200);
	}
}

void put_low_power_while_charging(void) {
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_9); // Interrupt configured for Falling edge to wake up from Sleep
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	EnterStop();   // Enter Stop Mode
	LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_9);

}

void Execute_VCP_Command(void) {
	Data_log_Start_Resume = 0;
	if (EnterReset_Create == 1) {
		EnterReset_Create = 0;
		NVIC_SystemReset();    // System reset
	}

	if (EnterDFU_Create == 1) {
		EnterDFU_Create = 0;
		*Bootloader_Ram_Key_Address = Bootloader_Key_Value; // Write a key to a RAM location to check at next reset
		NVIC_SystemReset();    // System reset
	}

	if (Logstart_Create == 1) {
		Logstart_Create = 0;
		FATFS_Logstart_Ready();
		Data_log_Start_Resume = 1;
		Check_SD_Command_File();
	}
}

void USB_Init_Start(void) {
	SD_POWER_ON();

	Pros_state = LP_STOP;                  // Next state
	NVIC_DisableIRQ(LPTIM2_IRQn);         // Disabling LPTIM2 Timer Interrupt
	USB_Clock_Ready();
	NVIC_DisableIRQ(EXTI9_5_IRQn); // Disabling External GPIO pin interrupt (EXTI9-5) for USB connectivity PA9 Pin

	if (Data_log_Start_Resume == 1)    // If Data log was in operation
			{
		File_Close_Update_Unlink();

	}
	Data_log_Start_Resume = 0;
	HAL_ResumeTick();      // Before entering USB mode, SYS_tick needs to enable

	if (USB_Mode == 1)    // USB MSC Mode  // Default: USB VCP mode (0), MSC (1)
			{
		VIOLET_LED_ONLY();
		USB_Mode = 0;               // Next mode default USB VCP
		USB_MSC_Init_Start();
		FATAL_Error = 0;

		while (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY))
			;       // Stay USB mode until USB connectivity available PA9
		ALL_LED_OFF();
		// After USB Cable Disconnected from device
		Shut_Down_USB();
		//	Wait_for_DFU();
		Enter_Into_DFU = 0;
		if (Enter_Into_DFU == 1) {
			Enter_Into_DFU = 0;
			*Bootloader_Ram_Key_Address = Bootloader_Key_Value; // Write a key to a RAM location to check at next reset
			NVIC_SystemReset();        // System reset
		}

		Prepare_Data_Log_State();
		NVIC_EnableIRQ(EXTI9_5_IRQn);

	} else                             // USB VCP Mode
	{

		//ALL_LED_OFF();
		CYAN_LED_ONLY();

		USB_VCP_Init_Start();
		if (Battey_Charge_Mode(&USBD_Device) == USBD_OK) // USB Battery charging mode
				{
			//  BLUE_LED_ONLY();
			FATAL_Error = 0;
			while (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY))
				;
			Shut_Down_USB();
			ALL_LED_OFF();

			Check_SD_Command_File();
			Prepare_Data_Log_State();
			NVIC_EnableIRQ(EXTI9_5_IRQn);
		}

		else                         // USB VCP Mode
		{

			//	 ALL_LED_ON();
			FATAL_Error = 0;
			while (LL_GPIO_IsInputPinSet(GPIOA, USB_CONNECTIVITY))
				;       // Stay USB mode until USB connectivity available PA9
			// After USB Cable Disconnected from device
			Shut_Down_USB();
			ALL_LED_OFF();
			VCP_continue = 1;
			//	Wait_for_MSC();
			if (VCP_continue == 1)   // Not USB MSC mode intended
					{
				VCP_continue = 0;               // Variable reset, aviod looping

				Execute_VCP_Command();

				Prepare_Data_Log_State();         // Prepare for Data collection
				NVIC_EnableIRQ(EXTI9_5_IRQn); // USB PIN interrupt enable
			} else   // Jump to USB MSC
			{

				ALL_LED_OFF();                           // Switch off USB Light
				USB_Mode = 0;                  // Next Mode USB MSC
				Data_log_Start_Resume = 0;    // Data log stop
				Pros_state = USB_MSC_VCP_Mode;  // Goto USB Modes immediately
			}
		}
	}
}

void Prepare_Goto_Dormant_Mode(void) {
	LL_LPTIM_DisableIT_ARRM(LPTIM2); // Disable auto reload match interrupt of LPTIM2
	SD_POWER_OFF();               // Power off SD CARD

}

void Reset_Variables_for_LowBattery(void) // Reset all variables needed to Sensor logging in Double Buffer System
{

	SD_write_Flag = 0;
#ifdef Bottle_Device
	//ALL_LED_OFF();
#else
	// NVIC_SystemReset();
#endif

}

void Reset_All(void) // Reset all variables needed to Sensor logging in Double Buffer System
{

	last_file = 0;
	Sub_cnt = 0;         // Array Index count in a single buffer to store values
	w_flag = 1; // Buffer write flag to be written in FATFS file (1=Buffer1, 0=Buffer0)
	s_flag = 0; // Buffer Store flag to be stored/filled by sensor values in interrupt (1=Buffer1, 0=Buffer0)
	SD_write_Flag = 0; // Flag to indicates whether a buffer a full and ready to write in SD card
	SD_Write_Count = 0; // Number of SD write. If reach a predefined value, f_sync() operation will update FATFS files.

	// Debug Msg clear
	Clear_ErrorTimestamp_Buffer();
	for (int indxE = 0; indxE < 10; indxE++) {
		Error_reg_log.Error[indxE] = 0;
		Error_reg_log.ErrorTime[indxE] = 0;
	}

}

int GetNextIndex(char *path) // Read SD card content to save next value in auto increment format
{
	DIR dir;
	FILINFO fno;
	int i, index = -1;

	if (f_opendir(&dir, path) == FR_OK) {

		while (1) {
			if ((f_readdir(&dir, &fno) != FR_OK) || (fno.fname[0] == 0))
				break;

			if ((strstr(fno.fname, ".BIN") != NULL)
					&& (sscanf(fno.fname, "%d", &i) == 1)) // Searcing for .BIN file and determining highest index
				if (i > index)
					index = i;
		}
	}
	return (index + 1);
}

uint8_t read_conf_File(char *path) // Read SD card content to save next value in auto increment format
{
	DIR dir;
	FILINFO fno;
	int i;
	uint8_t index = 0;

	if (f_opendir(&dir, path) == FR_OK) {

		while (1) {
			if ((f_readdir(&dir, &fno) != FR_OK) || (fno.fname[0] == 0))
				break;

			if ((strstr(fno.fname, ".AIM") != NULL)
					&& (sscanf(fno.fname, "%d", &i) == 1)) // Searcing for .BIN file and determining highest index
				if (i > index)
					index = i;
		}
	}
	return 0;
}

void Try_FATFS_Mount(void) {
	if (Fat_mnt_fail == 1) {
		Fat_mnt_fail = 0;
		SD_POWER_OFF();
		delay_us(10000); // wait 10 msec
		SD_POWER_ON();
		delay_us(1000000); // wait 1 sec
		//Retry FATFS link

		if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
				{
			if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
					{
				Skip_FATFS = 1;
			}
		} else {
			Skip_FATFS = 1;
		}
	}

}

void file_reopen_sync(void) {
	f_close(&MyFile);
	f_open(&MyFile, FILE_NAME_STRING, FA_OPEN_ALWAYS | FA_WRITE);
	if (res != FR_OK) {
		Fat_sync_read_fail = 1;
	}
}

void file_reopen(void) {
	f_close(&MyFile);
	f_open(&MyFile, FILE_NAME_STRING, FA_OPEN_ALWAYS | FA_WRITE);
	if (res != FR_OK) {
		Fat_write_read_fail = 1;
	}
}

void file_mnt_reopen(void) {
	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
			{
		if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
				{
			Fat_read_mnt_fail = 1;
		} else  // Good to Go
		{
			res = f_open(&MyFile, FILE_NAME_STRING, FA_OPEN_ALWAYS | FA_WRITE);
			if (res != FR_OK) {
				Fat_read_mnt_fail = 1;
			}

		}
	} else {
		Fat_read_mnt_fail = 1;
	}

}
#define File_name_Autoincrement
void Open_File_For_Sensor_Write(void) {
	if (Skip_FATFS == 0) {
#ifdef File_name_Autoincrement
		last_file = GetNextIndex("");
		sprintf(FILE_NAME_STRING, "%06d.BIN", last_file);

#else
		Convert_SD_FileName_Unix_Time(); // Read RTC time and store in Sensor Filename (UNIX format)
#endif

		res = f_open(&MyFile, FILE_NAME_STRING, FA_OPEN_ALWAYS | FA_WRITE);
		if (res != FR_OK) // Create and Open a new text file object with write access
				{
			AIM_Error_Handler(f_open_E);
		} else {
			f_lseek(&MyFile, MyFile.fsize); // Increase File Size when big file to write
			res = f_sync(&MyFile);
		}
		delay_us(1000);  // Arbitrary delay to complete FATFS SD write

	}
}

void FATFS_SD_Open_Ready(void) {
	Convert_SD_FileName_Unix_Time(); // Read RTC time and store in Sensor Filename (UNIX format)

	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
			{
		if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
				{     // Initialization Error Debugging
			//		 RED_LED_ON();
		}

		else {
			if (f_open(&MyFile, FILE_NAME_STRING, FA_OPEN_ALWAYS | FA_WRITE)
					!= FR_OK) // Create and Open a new .BIN file object with write access
					{
				//		 RED_LED_ON();
			} else {
				f_lseek(&MyFile, MyFile.fsize); // Increase File Size when big file to write
				f_sync(&MyFile);
			}
		}
		delay_us(100);
	}
}

void FATFS_Logstart_Delete(void) {

	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
			{
		if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
				{     // Initialization Error Debugging
			//		 RED_LED_ON();
		}

		else {
			if (f_unlink("LogStart.txt") == FR_OK) { // If LogStart.txt file present, delete the file
			}
		}
		delay_us(100);
	}
	f_mount(0, "", 0); 		     //  unmount FATFS file if necessary
	FATFS_UnLinkDriver(SDPath);  //  unlink SD card driver if necessary
}

void FATFS_Logstart_Ready(void) {

	if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)    // Link SD Driver
			{
		if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 1) != FR_OK) // Register the file system object to the FatFs module
				{     // Initialization Error Debugging

		}

		else {
			if (f_open(&MyFile, "LogStart.txt", FA_OPEN_ALWAYS | FA_WRITE)
					!= FR_OK) // Create and Open a new text file object with write access
					{

			} else {

				f_close(&MyFile);
			}
		}
		delay_us(100);
	}
	f_mount(0, "", 0); 		     //  unmount FATFS file if necessary
	FATFS_UnLinkDriver(SDPath);  //  unlink SD card driver if necessary
}

void update_FATFS_time(void) // After File close, write RTC time-stamp on File header to visualize in PC
{
	Read_RTC_Timestamp();         // Read RTC time-stamps and store in variables
	fno.fdate = (WORD) (((Year + 20) << 9) | Mon << 5 | Day); // Update File time with the data log starting time
	fno.ftime = (WORD) (Hour << 11 | Min << 5 | Sec / 2); // Extracting values from RTC registers
	f_utime(FILE_NAME_STRING, &fno);     // Update RTC time on FATFS file header
}

void SD_Sensor_write(void)         // Storing Sensor Buffer values on FATFS file
{
#ifdef Bottle_Device
	//ALL_LED_OFF();
#else
#endif

	res = f_write(&MyFile, &BSbuffer[w_flag], (8192 * 2), &BytesWritten2);
	if (res != FR_OK)   // Write the sensor Buffer content to .BIN file
			{
		AIM_Error_Handler(f_write_E);
	} else {
		GREEN_LED_OFF();               // Sensor write Completed
	}
#ifdef Bottle_Device
	f_sync(&MyFile);
#else

	if (File_Sensor_write_issue == 0) {

		if (SD_Write_Count == SD_cnt_limit) // After 50 SD card write, File Sync (176 write*20.45 sec = 60 minute
				{
			res = f_sync(&MyFile);
			if (res != FR_OK) {
				AIM_Error_Handler(f_sync_E);
			}
			// Sync file without closing
			SD_Write_Count = 0;                       // Reset SD write counters
		} else {
			SD_Write_Count++;                      // Increment SD write counter
		}
	}
#endif
}

void USB_MSC_Init_Start(void)                         // Initialize USB MSC mode
{
	USBD_Init(&USBD_Device, &MSC_Desc, 0);             // USB MSC initialization
	USBD_RegisterClass(&USBD_Device, USBD_MSC_CLASS);  // USB class registration
	USBD_MSC_RegisterStorage(&USBD_Device, &USBD_DISK_fops); // Storage call backs
	USBD_Start(&USBD_Device);                                // Start USB MSC

	delay_us(1000000);

}

void USB_VCP_Init_Start(void) {
	USBD_Init(&USBD_Device, &VCP_Desc, 0);             // USB VCP initialization
	USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);  // USB class registration
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops); // VCP CDC call backs
	USBD_Start(&USBD_Device);                                 // Start USB VCP
	delay_us(1000000);
}

void Convert_SD_FileName_Unix_Time(void)  // Update file name with RTC timestamp
{
	struct tm t;                          // Time structure to convert UNIX time
	t.tm_year = __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)) + 100; // To make it 20xx say 2017
	t.tm_mon = __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)) - 1; // Month, 0 - jan in time structure
	t.tm_mday = __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)); // Day of the month
	t.tm_hour = __RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
	t.tm_min = __RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	t.tm_sec = __RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));
	t.tm_isdst = -1;                 // Is DST on? 1 = yes, 0 = no, -1 = unknown

	// Releasing RTC registers; necessary for RTC read/write operation
	(void) RTC->DR;
	(void) RTC->TR;

	sprintf(FILE_NAME_STRING, "%lx.BIN", mktime(&t)); // Update File name as Hex Unix Value, Change file format when necessary

}

void Convert_SD_FileName_RTC_Timestamp(void) // Update file name with RTC timestamp (UNIX format)
{
	Read_RTC_Timestamp();        //    Read RTC timestamp and store in variables
								 //    Update file name with RTC timestamp   // Change the file format when necessary

	sprintf(FILE_NAME_STRING, "%.2d_%.2d_%.2d.BIN", Hour, Min, Sec); // Only limited to 8characters
}

void Store_RTC_Time(void) // Read RTC timestamp and write in a string as hh:mm:ss format
{
	//  time Format : hh:mm:ss
	sprintf((char*) RTC_String, "%.2d:%.2d:%.2d",
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));
}
void Store_RTC_Date(void) // Read RTC calender and write in a string as mm-dd-yy format
{
	// date Format : mm-dd-yy
	sprintf((char*) RTC_String, "%.2d-%.2d-%.2d",
			__RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
			__RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
			2000 + __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)));
}

void Read_RTC_Timestamp(void)       // Read RTC timestamp and store in variables
{
// read RTC time in BCD format from registers and save in .BIN/.DEC format

	Hour = __RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
	Min = __RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	Sec = __RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));

	Mon = __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC));
	Day = __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC));
	Year = __RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC));

// Releasing RTC registers (Time and Date registers); necessary for RTC read/write operation
	(void) RTC->DR;
	(void) RTC->TR;
}

void Enter_RTC_InitMode(void) // Function needed before writing on RTC registers
{
	RTC->ISR = RTC_INIT_MASK;                                // Enter Init Mode
	while (((RTC->ISR) & RTC_ISR_INITF) != (RTC_ISR_INITF))
		; // Wait to enter Init mode
}

void Exit_RTC_InitMode(void)   // Function needed after writing on RTC registers
{
	RTC->ISR = (uint32_t) ~RTC_ISR_INIT; // Disable RTC init mode

	// Wait for synchro. Needed only if Shadow registers is enabled
	WRITE_REG(RTC->ISR,
			(~((RTC_ISR_RSF | RTC_ISR_INIT) & 0x0000FFFFU) | (RTC->ISR & RTC_ISR_INIT))); // Clear RTC sync flag
	while (((RTC->ISR) & RTC_ISR_RSF) != (RTC_ISR_RSF))
		;  // Wait the registers to be synchronised
}

void MX_RTC_Init(void)                // RTC initialization after Power on reset
{
	LL_RTC_InitTypeDef RTC_InitStruct;

	LL_RCC_ForceBackupDomainReset();            // Reset Backup domain registers
	LL_RCC_ReleaseBackupDomainReset();
	LL_RCC_LSE_Enable();                    // Configure LSE as RTC source clock
	while (LL_RCC_LSE_IsReady() != 1)
		;
	LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	LL_RCC_EnableRTC();                           // RTC clock

	RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
	LL_RTC_Init(RTC, &RTC_InitStruct); // Initialize RTC with default configurations

	LL_RTC_DisableWriteProtection(RTC);
	Enter_RTC_InitMode();     // Enter RTC initialization mode to set parameters

	// Calender set up
	MODIFY_REG(RTC->PRER, RTC_PRER_PREDIV_A,
			0x7F << RTC_POSITION_PRER_PREDIV_A);  // ASYNC prediv= 0x7F
	MODIFY_REG(RTC->PRER, RTC_PRER_PREDIV_S, 0xFF);         // SYNC prediv= 0xFF

	Exit_RTC_InitMode();
	LL_RTC_EnableWriteProtection(RTC);

	RTC_BAK_SetRegister(RTC, 0x01, RTC_BKP_DATE_TIME_UPDTATED); // A dummy value is necessary to store to indicate RTC is configured
}

void Set_RTC_Calendar(uint8_t Year, uint8_t Month, uint8_t Day, uint8_t Hour,
		uint8_t Min, uint8_t Sec) {
	LL_RTC_TimeTypeDef RTC_TimeStruct;
	LL_RTC_DateTypeDef RTC_DateStruct;

	// Enter RTC initialization mode to set parameters
	LL_RTC_DisableWriteProtection(RTC);
	Enter_RTC_InitMode();

	RTC_TimeStruct.Hours = Hour;
	RTC_TimeStruct.Minutes = Min;
	RTC_TimeStruct.Seconds = Sec;
	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);

	RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;          // Default value
	RTC_DateStruct.Month = Month;
	RTC_DateStruct.Year = Year;
	RTC_DateStruct.Day = Day;
	LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_DateStruct);

	Exit_RTC_InitMode();
	LL_RTC_EnableWriteProtection(RTC);
	RTC_BAK_SetRegister(RTC, 0x01, RTC_BKP_DATE_TIME_UPDTATED); // A dummy value is necessary to store to indicate RTC is configured
}

// Valid register value indicates RTC is enabled and running
void RTC_BAK_SetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister,
		uint32_t Data) {
	register uint32_t tmp = 0U;

	tmp = (uint32_t) (&(RTCx->BKP0R));
	tmp += (BackupRegister * 4U);

	/* Write the specified register */
	*(__IO uint32_t *) tmp = (uint32_t) Data;
}

// Valid register value indicates RTC is enabled and running
uint32_t RTC_BAK_GetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister) {
	register uint32_t tmp = 0U;

	tmp = (uint32_t) (&(RTCx->BKP0R));
	tmp += (BackupRegister * 4U);

	/* Read the specified register */
	return (*(__IO uint32_t *) tmp);
}

uint8_t Bcd2ToByte(uint8_t Value)          // Converting BCD value BIN value
{
	uint8_t tmp = 0;
	tmp = ((uint8_t) (Value & (uint8_t) 0xF0) >> (uint8_t) 0x4) * 10;
	return (tmp + (Value & (uint8_t) 0x0F));
}

void BTN_PA10_EXTI_conf(void) // PA10 EXTI pin interrupt configuration for Push Button Sense
{
	NVIC_EnableIRQ(EXTI15_10_IRQn); // Enabling External GPIO pin interrupt (EXTI15-10) for USB connectivity PA10 Pin
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG); // Enable Sysconfig clock to enable EXTI pin interrupts
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE10); // Set PA10 pin EXTI interrupt in EXTI15-10 line
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);    // Enable PA10 pin EXTI interrupt
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_10); // Interrupt configured for falling edge
}

void USB_PA9_EXTI_conf(void) // PA9 EXTI pin interrupt configuration for USB connectivity Sense
{
	NVIC_EnableIRQ(EXTI9_5_IRQn); // Enabling External GPIO pin interrupt (EXTI9-5) for USB connectivity PA10 Pin
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG); // Enable Sysconfig clock to enable EXTI pin interrupts
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE9); // Set PA9 pin EXTI interrupt in EXTI9-5 line
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_9);      // Enable PA9 pin EXTI interrupt
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_9); // Interrupt configured for Rising edge
}

//Toe: ADC1_Ch3_PC2=ADC1_Flex1  Heel: ADC2_Ch4_PC3=ADC2_Flex2
void P_ADC_Sensor_GPIO_Init(void) {
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
	LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_2);

	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_ANALOG);
	LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_3);

// LoadCell1

	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
	LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_1);

	// LoadCell2

	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
	LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_0);

	P_ADC1_conf_strain();

}

void Battery_Monitor_GPIO_init(void) // ADC1 input channel 6 (PA1) for battery monitor
{

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
	LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_1);

}

void ADC_conf_battery(void) {
	LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC);

	LL_GPIO_DisablePinAnalogControl(GPIOA, LL_GPIO_PIN_0); // Set Battery as ADC input
	LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_1);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOWAIT);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6,
			LL_ADC_SAMPLINGTIME_247CYCLES_5);
	Activate_ADC();
}

void P_ADC_conf_strain(void) {
	LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOWAIT);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_5);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_6);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4,
			LL_ADC_SAMPLINGTIME_47CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5,
			LL_ADC_SAMPLINGTIME_47CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6,
			LL_ADC_SAMPLINGTIME_47CYCLES_5);

	Activate_ADC();

}

void ADC_conf_strain(void) {

	LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC);

	LL_GPIO_DisablePinAnalogControl(GPIOA, LL_GPIO_PIN_1); // Set Strain_sensor as ADC input
	LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_0);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOWAIT);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5,
			LL_ADC_SAMPLINGTIME_247CYCLES_5);
	Activate_ADC();
}

//Toe: ADC1_Ch3_PC2=ADC1_Flex1  Heel: ADC2_Ch4_PC3=ADC2_Flex2

void P_ADC1_conf_strain(void) {

	LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

	//ADC1
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOWAIT);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3); // FSR1=PC2=Ch3=ADC1
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3,
			LL_ADC_SAMPLINGTIME_640CYCLES_5);

	//ADC2

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC2),
			LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	LL_ADC_SetLowPowerMode(ADC2, LL_ADC_LP_AUTOWAIT);
	LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetOverrun(ADC2, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

	LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4,
			LL_ADC_SAMPLINGTIME_640CYCLES_5); // FSR2=PC3=Ch4=ADC2

	//ADC3

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC3),
			LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	LL_ADC_SetLowPowerMode(ADC3, LL_ADC_LP_AUTOWAIT);
	LL_ADC_REG_SetTriggerSource(ADC3, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC3, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetOverrun(ADC3, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

	LL_ADC_REG_SetSequencerLength(ADC3, LL_ADC_REG_SEQ_SCAN_DISABLE);

	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_2,
			LL_ADC_SAMPLINGTIME_640CYCLES_5); // Load1=PC1=Ch2

	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_1,
			LL_ADC_SAMPLINGTIME_640CYCLES_5); // Load2=PC0=Ch1

	P_Activate_ADC3();

	P_Activate_ADC2();

	P_Activate_ADC1();
}

void P_ADC2_conf_strain(void) {

}

void ADC_Power_Down(void) {
	LL_ADC_Disable(ADC1);
	LL_ADC_EnableDeepPowerDown(ADC1);
}

void Activate_ADC(void) {
	__IO uint32_t wait_loop_index = 0;
	LL_ADC_DisableDeepPowerDown(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);      // Enable internal ADC regulators

	wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
			* (SystemCoreClock / 100000) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED); // ADC calibration single ended
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsCalibrationOnGoing(ADC1) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}

	wait_loop_index = ADC_DELAY_CALIB_ENABLE_CPU_CYCLES; // ADC Delay calculations
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_Enable(ADC1);                             // ADC Enable
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}
}

void P_Activate_ADC3(void) {
	__IO uint32_t wait_loop_index = 0;
	LL_ADC_DisableDeepPowerDown(ADC3);
	LL_ADC_EnableInternalRegulator(ADC3);      // Enable internal ADC regulators

	wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
			* (SystemCoreClock / 100000) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_StartCalibration(ADC3, LL_ADC_SINGLE_ENDED); // ADC calibration single ended
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsCalibrationOnGoing(ADC3) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}

	wait_loop_index = ADC_DELAY_CALIB_ENABLE_CPU_CYCLES; // ADC Delay calculations
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_Enable(ADC3);                             // ADC Enable
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsActiveFlag_ADRDY(ADC3) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}
}

void P_Activate_ADC2(void) {
	__IO uint32_t wait_loop_index = 0;
	LL_ADC_DisableDeepPowerDown(ADC2);
	LL_ADC_EnableInternalRegulator(ADC2);      // Enable internal ADC regulators

	wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
			* (SystemCoreClock / 100000) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED); // ADC calibration single ended
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsCalibrationOnGoing(ADC2) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}

	wait_loop_index = ADC_DELAY_CALIB_ENABLE_CPU_CYCLES; // ADC Delay calculations
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_Enable(ADC2);                             // ADC Enable
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}
}

void P_Activate_ADC1(void) {
	__IO uint32_t wait_loop_index = 0;
	LL_ADC_DisableDeepPowerDown(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);      // Enable internal ADC regulators

	wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
			* (SystemCoreClock / 100000) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED); // ADC calibration single ended
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsCalibrationOnGoing(ADC1) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}

	wait_loop_index = ADC_DELAY_CALIB_ENABLE_CPU_CYCLES; // ADC Delay calculations
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_Enable(ADC1);                             // ADC Enable
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while ((LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}
}

void ConversionStartPoll_ADC_GrpRegular(void) // Start ADC conversion (single conversion in this project)
{
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count = 0;
	ADC_Error = 0;
	while (!LL_DMA_IsActiveFlag_TC1(DMA1))
		;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0) && !Error_preceding
			&& (ADC_Error == 0)) {
		Wait_for_ADC_TimeOut_();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	if (ADC_Error == 1) {
		NonCritical_Sensor_Error = 1;
	}
}

void ConversionStartPoll_ADC_GrpRegular_Buf(void) // Start ADC conversion (single conversion in this project)
{
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	//while(!LL_DMA_IsActiveFlag_TC1(DMA1));
	// LL_DMA_ClearFlag_TC1(DMA1);
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) // && !Error_preceding&& (ADC_Error_Buf==0))
	{
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);

	//if (ADC_Error_Buf==1)
	//{
	//	NonCritical_Sensor_Error=1;
	//}
}

void P_IMU4_SPI3_Init(void) // work for imu5
{

	LL_SPI_InitTypeDef SPI_InitStruct;

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* Peripheral clock enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

	GPIO_InitStruct.Pin = SPI3_SCK_IMU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(SPI3_SCK_IMU_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI3_MISO_IMU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(SPI3_MISO_IMU_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI3_MOSI_IMU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(SPI3_MOSI_IMU_GPIO_PORT, &GPIO_InitStruct);

	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;

	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(SPI3, &SPI_InitStruct);

	LL_SPI_DisableNSSPulseMgt(SPI3);

	LL_SPI_SetRxFIFOThreshold(SPI3, LL_SPI_RX_FIFO_TH_QUARTER);
	LL_SPI_DisableIT_RXNE(SPI3);
	LL_SPI_Enable(SPI3);

	delay_us(10000);

}

//void P_IMU1_SPI1_Initialization_at_reset(void)       // IMU configurations
//{
//	spi1_init();
//	//P_IMU1_SPI1_Init();         // Accelerometer Chip Initialization
//	delay_us(7000);         // Arbitrary delay after SPI initialization
//
//	MPU1_SPI1_init();
//	MPU2_SPI1_init();
//
//}
//void MCP_SPI2_Initialization_at_reset(void) {
//	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS SET Active Low
//
//	MCP_setup();
////	delay_us(7000);
//	MCP_reset();
//	delay_us(70000);
//}
//
//void MCP_reset() {
//	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);
//
//	while (!(SPI2->SR & SPI_SR_TXE))
//		; //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, 0xC0);
//	while (!(SPI2->SR & SPI_SR_RXNE))
//		; //data received?
//	LL_SPI_ReceiveData8(SPI2);
//	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS SET Active Low
//
//}

void P_IMU3_SPI2_Initialization_at_reset(void)       // IMU configurations
{
	P_IMU3_SPI2_Init();         // Accelerometer Chip Initialization
	delay_us(7000);         // Arbitrary delay after SPI initialization

	MPU3_SPI2_init();
}

void P_IMU4_SPI3_Initialization_at_reset(void)       // IMU configurations
{
	P_IMU4_SPI3_Init(); // peripheral initialization  //work for imu5   too
	delay_us(7000);         // Arbitrary delay after SPI initialization

	MPU4_SPI3_init(); // device initialization // for IMU5 need to create something like "WriteReg3"
	MPU5_SPI3_init(); // later created for IMU5

}

void Wait_for_SPI_timeout_While_Datalog(void) // During Timer_Interrupt for Data logging, Involved Volatile variables
{
	SPI_TimeOut_Count_Buf++;
	if (SPI_TimeOut_Count_Buf > SPI_TimeOut_Limit_Buf) // 100ms= 48000*SPI_TimeOut_MS_value)
	{
		SPI_TimeOut_Count_Buf = 0;
		SPI_Error_Buf = 1;
	}
}

void Wait_for_ADC_timeout_While_Datalog(void) // During Timer_Interrupt for Data logging, Involved Volatile variables
{
	ADC_TimeOut_Count_Buf++;
	if (ADC_TimeOut_Count_Buf > ADC_TimeOut_Limit_Buf) // 100ms= 48000*ADC_TimeOut_MS_value)
	{
		ADC_TimeOut_Count_Buf = 0;
		ADC_Error_Buf = 1;
	}
}

void Wait_for_SPI_TimeOut_(void) {
	SPI_TimeOut_Count++;
	if (SPI_TimeOut_Count > SPI_TimeOut_Limit) // 100ms= 48000*SPI_TimeOut_MS_value)
	{
		SPI_TimeOut_Count = 0;
		SPI_Error = 1;
	}
}

void Wait_for_ADC_TimeOut_(void) {
	ADC_TimeOut_Count++;
	if (ADC_TimeOut_Count > ADC_TimeOut_Limit) // 100ms= 48000*SPI_TimeOut_MS_value)
	{
		ADC_TimeOut_Count = 0;
		ADC_Error = 1;
	}
}

void MPU_SPI_SendData(uint8_t adress, uint8_t data) {

	LL_GPIO_ResetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, adress);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, data);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS SET Active Low
}

unsigned int WriteReg(uint8_t adress, uint8_t data) {
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, adress);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, data);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	temp_val = LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS SET Active Low
	return temp_val;
}

unsigned int WriteReg3(uint8_t adress, uint8_t data) {
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(SPI3_CS_GPIO_PORT, SPI3_CS_PIN); // PA4 CS RESET Active Low // for IMU5 change here
	delay_us(10);
	while (!(SPI3->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI3, adress);
	while (!(SPI3->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI3);

	while (!(SPI3->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI3, data);
	while (!(SPI3->SR & SPI_SR_RXNE))
		; //data received?
	temp_val = LL_SPI_ReceiveData8(SPI3);
	delay_us(5);
	LL_GPIO_SetOutputPin(SPI3_CS_GPIO_PORT, SPI3_CS_PIN); // PA4 CS SET Active Low // for IMU5 change here
	return temp_val;
}

unsigned int WriteReg3_imu5(uint8_t adress, uint8_t data) {
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(SPI3_CS_GPIO_PORT, SPI3_IMU5_CS_Pin); // PA4 CS RESET Active Low // for IMU5 changed here
	delay_us(10);
	while (!(SPI3->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI3, adress);
	while (!(SPI3->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI3);

	while (!(SPI3->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI3, data);
	while (!(SPI3->SR & SPI_SR_RXNE))
		; //data received?
	temp_val = LL_SPI_ReceiveData8(SPI3);
	delay_us(5);
	LL_GPIO_SetOutputPin(SPI3_IMU5_CS_GPIO_Port, SPI3_IMU5_CS_Pin); // PA4 CS SET Active Low // for IMU5 changed here
	return temp_val;
}

unsigned int WriteReg1(uint8_t adress, uint8_t data) {
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, adress);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, data);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	temp_val = LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN);
	return temp_val;
}

unsigned int WriteReg1_imu2(uint8_t adress, uint8_t data) {
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(SPI1_IMU2_CS_GPIO_Port, SPI1_IMU2_CS_Pin);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, adress);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, data);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	temp_val = LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_PORT, SPI1_IMU2_CS_Pin);
	return temp_val;
}

////void MCP_write(uint8_t adress, uint8_t data)
//void MCP_write(int adress, int data) {
//
//	// Activate chip select
//	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);
//
//	while (!(SPI2->SR & SPI_SR_TXE)); //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, 0x02);
//
//	while (!(SPI2->SR & SPI_SR_TXE)); //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, adress);
//
//	while (!(SPI2->SR & SPI_SR_TXE)); //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, data);
//
//	while ((SPI2->SR & SPI_SR_FTLVL)){}; //transmit fifo empty?
//	while (!(SPI2->SR & SPI_SR_BSY)){}; // no longer busy
//	while ((SPI2->SR & SPI_SR_FRLVL))
//	{
//		uint8_t dummy = SPI2->DR; // clear rx fifo from the receives.
//	};
//
//	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS SET Active Low
//
//}
//
////unsigned int MCP_read(uint8_t adress)
//int MCP_read(int adress) {
//	unsigned int temp_val;
//	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);
//
//	while (!(SPI2->SR & SPI_SR_TXE))
//		; //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, 0x03);
//	while (!(SPI2->SR & SPI_SR_RXNE))
//		; //data received?
//	LL_SPI_ReceiveData8(SPI2);
//
//	while (!(SPI2->SR & SPI_SR_TXE))
//		; //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, adress);
//	while (!(SPI2->SR & SPI_SR_RXNE))
//		; //data received?
//	LL_SPI_ReceiveData8(SPI2);
//
//	while (!(SPI2->SR & SPI_SR_TXE))
//		; //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI2, 0x00);
//	while (!(SPI2->SR & SPI_SR_RXNE))
//		; //data received?
//	temp_val = LL_SPI_ReceiveData8(SPI2);
//
//	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS SET Active Low
//	return temp_val;
//}

unsigned int WriteReg2(uint8_t adress, uint8_t data) {
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, adress);
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI2);

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, data);
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	temp_val = LL_SPI_ReceiveData8(SPI2);

	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS SET Active Low
	return temp_val;
}

void MPU2_SPI2_GetData(uint8_t adress) {

	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, (adress | 0x80)); // (Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI2);

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, 0x00);
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI2);

	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PC4 CS SET Active Low

}

void MPU_SPI_GetData(uint8_t adress) {

	LL_GPIO_ResetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, (adress | 0x80)); // (Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, 0x00);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PC4 CS SET Active Low

}

void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	for (i = 0; i < Bytes; i++) {
		while (!(SPI1->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI1, 0x00);

		while (!(SPI1->SR & SPI_SR_RXNE))
			; //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI1);

	}

	LL_GPIO_SetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PC4 CS SET Active Low

}

//void MPU_SPI_GetData1(uint8_t adress) {
//
//	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN); // PA4 CS RESET Active Low
//
//	while (!(SPI1->SR & SPI_SR_TXE))
//		; //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI1, (adress | 0x80)); // (Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
//	while (!(SPI1->SR & SPI_SR_RXNE))
//		; //data received?
//	LL_SPI_ReceiveData8(SPI1);
//
//	while (!(SPI1->SR & SPI_SR_TXE))
//		; //transmit buffer empty?
//	LL_SPI_TransmitData8(SPI1, 0x00);
//	while (!(SPI1->SR & SPI_SR_RXNE))
//		; //data received?
//	LL_SPI_ReceiveData8(SPI1);
//
//	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN); // PC4 CS SET Active Low
//
//}

void ReadRegs1(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	for (i = 0; i < Bytes; i++) {
		while (!(SPI1->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI1, 0x00);

		while (!(SPI1->SR & SPI_SR_RXNE))
			; //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI1);

	}

	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN); // PC4 CS SET Active Low

}

void ReadRegs1_imu2(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(SPI1_IMU2_CS_GPIO_Port, SPI1_IMU2_CS_Pin); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	for (i = 0; i < Bytes; i++) {
		while (!(SPI1->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI1, 0x00);

		while (!(SPI1->SR & SPI_SR_RXNE))
			; //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI1);

	}

	LL_GPIO_SetOutputPin(SPI1_IMU2_CS_GPIO_Port, SPI1_IMU2_CS_Pin); // PC4 CS SET Active Low

}

void ReadRegs3(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(SPI3_CS_GPIO_PORT, SPI3_CS_PIN); // PA4 CS RESET Active Low // change here for IMU5
	delay_us(10);
	while (!(SPI3->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI3, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI3->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI3);

	for (i = 0; i < Bytes; i++) {
		while (!(SPI3->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI3, 0x00);

		while (!(SPI3->SR & SPI_SR_RXNE))
			; //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI3);

	}
	delay_us(5);
	LL_GPIO_SetOutputPin(SPI3_CS_GPIO_PORT, SPI3_CS_PIN); // PC4 CS SET Active Low // change here for IMU5

}

void ReadRegs3_imu5(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(SPI3_IMU5_CS_GPIO_Port, SPI3_IMU5_CS_Pin); // PA4 CS RESET Active Low // change here for IMU5
	delay_us(10);
	while (!(SPI3->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI3, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI3->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI3);

	for (i = 0; i < Bytes; i++) {
		while (!(SPI3->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI3, 0x00);

		while (!(SPI3->SR & SPI_SR_RXNE))
			; //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI3);

	}
	delay_us(5);
	LL_GPIO_SetOutputPin(SPI3_IMU5_CS_GPIO_Port, SPI3_IMU5_CS_Pin); // PC4 CS SET Active Low // change here for IMU5

}

void ReadRegs2(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI2);

	for (i = 0; i < Bytes; i++) {
		while (!(SPI2->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI2, 0x00);

		while (!(SPI2->SR & SPI_SR_RXNE))
			; //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI2);

	}

	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PC4 CS SET Active Low

}

void MPU9250_Read_Buf(void) {
	uint8_t indexB = 0; // temporary register
	int16_t sensorG_data_read[18];

	LL_GPIO_ResetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, 0xA2); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	for (indexB = 0; indexB < 12; indexB++) {
		while (!(SPI1->SR & SPI_SR_TXE))
			; //transmit buffer empty?
		LL_SPI_TransmitData8(SPI1, 0x00);

		while (!(SPI1->SR & SPI_SR_RXNE))
			; //data received?
		sensorG_data_read[indexB] = LL_SPI_ReceiveData8(SPI1);
		//	LL_SPI_ReceiveData8(SPI1);
	}

	LL_GPIO_SetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PC4 CS SET Active Low

//BSbuffer[s_flag].GX[Sub_cnt] = (sensorG_data_read[1]<<8) | sensorG_data_read[0];// GX
//BSbuffer[s_flag].GY[Sub_cnt]  = (sensorG_data_read[3]<<8) | sensorG_data_read[2]; //GY
//BSbuffer[s_flag].GZ[Sub_cnt]  = (sensorG_data_read[5]<<8) | sensorG_data_read[4]; //GZ
//BSbuffer[s_flag].X[Sub_cnt]  = (sensorG_data_read[7]<<8) | sensorG_data_read[6]; // X
//BSbuffer[s_flag].Y[Sub_cnt] = (sensorG_data_read[9]<<8) | sensorG_data_read[8]; // Y
//BSbuffer[s_flag].Z[Sub_cnt]  = (sensorG_data_read[11]<<8) | sensorG_data_read[10]; // Z
//BSbuffer[s_flag].MX[Sub_cnt]  = (sensorG_data_read[13]<<8) | sensorG_data_read[12]; // MX
//BSbuffer[s_flag].MY[Sub_cnt] = (sensorG_data_read[15]<<8) | sensorG_data_read[14]; // MY
//BSbuffer[s_flag].MZ[Sub_cnt]  = (sensorG_data_read[17]<<8) | sensorG_data_read[16]; // MZ

}

void Strain_Sensor_ADC_store(void) {

#ifdef Strain_Sensor_En

	if (ADC_Busy_with_Battery_Monitor == 0) // If ADC not busy with Battery Monitor
	{
		ConversionStartPoll_ADC_GrpRegular_Buf();   // Start a Single conversion
		Strain_ADC_Value = LL_ADC_REG_ReadConversionData12(ADC1);// Strain Sensor 12 bit ADC Value
	}
	BSbuffer[s_flag].Chew_Sensor_value[Sub_cnt] = Strain_ADC_Value;
#endif

}
__IO uint16_t aADCxConvertedData[3];
volatile uint16_t Test0, Test1, Test2, Test1_mV, Test2_mV, Test3, Test4,
		Test3_mV, Test4_mV, Test5, Test6, Test5_mV, Test6_mV;

void all_ADC_read_test1(void) {
	//LL_GPIO_ResetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);            // Battery Monitor On
	ConversionStartPoll_ADC_GrpRegular_Buf();       // Start a Single conversion
	//LL_GPIO_SetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);         // Battery Monitor Off
	Test0 = aADCxConvertedData[0];
	Test1 = aADCxConvertedData[1];

	delay_us(100000);

	//LL_GPIO_ResetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);            // Battery Monitor On
	ConversionStartPoll_ADC_GrpRegular_Buf();       // Start a Single conversion
	//LL_GPIO_SetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);         // Battery Monitor Off
	Test2 = aADCxConvertedData[0];
	Test3 = aADCxConvertedData[1];

	delay_us(100000);
	/*
	 LL_GPIO_ResetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);            // Battery Monitor On
	 ConversionStartPoll_ADC_GrpRegular_Buf();                          // Start a Single conversion
	 LL_GPIO_SetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);         // Battery Monitor Off
	 Test4= aADCxConvertedData[0];
	 Test5= aADCxConvertedData[1];*/
}

void battery_ADC_store_Buff(void) {
	//LL_GPIO_ResetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);            // Battery Monitor On

	//LL_GPIO_SetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);         // Battery Monitor Off
}

//int Read_Loadcell(void)
//{ int Load1, Load2;
//	// ADC3 Ch1 PC0 read
//	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
//
//	  LL_ADC_REG_StartConversion(ADC2);
//	  ADC_TimeOut_Count_Buf=0;
//	  ADC_Error_Buf=0;
//	  while ((LL_ADC_IsActiveFlag_EOC(ADC2) == 0))
//	  {
//		  Wait_for_ADC_timeout_While_Datalog();
//	  }
//	  LL_ADC_ClearFlag_EOC(ADC2);
//	  Load1= __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC2), LL_ADC_RESOLUTION_12B);  // Toe
//
//
//	  LL_ADC_REG_StartConversion(ADC1);
//	  ADC_TimeOut_Count_Buf=0;
//	  ADC_Error_Buf=0;
//	  while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0))
//	  {
//		  Wait_for_ADC_timeout_While_Datalog();
//	  }
//	  LL_ADC_ClearFlag_EOC(ADC1);
//	  Load2= __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC1), LL_ADC_RESOLUTION_12B);  //Heel
//
//return Load1;
//	  // ADC3 Ch2 PC1 read
//}

int Read_Loadcell1(void) {
	int Load1;
	// ADC3 Ch1 PC0 read
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);

	LL_ADC_REG_StartConversion(ADC2);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC2) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC2);
	Load1 = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI,
			LL_ADC_REG_ReadConversionData12(ADC2), LL_ADC_RESOLUTION_12B); // Toe

	return Load1;
	// ADC3 Ch2 PC1 read
}

int Read_Loadcell2(void) {
	int Load2;
	// ADC3 Ch1 PC0 read
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);

	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Load2 = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI,
			LL_ADC_REG_ReadConversionData12(ADC1), LL_ADC_RESOLUTION_12B); //Heel

	return Load2;
	// ADC3 Ch2 PC1 read
}

void F_Sensor_ADC_Store(void) {
	// ADC3 Ch1 PC0 read
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);

	LL_ADC_REG_StartConversion(ADC2);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC2) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC2);
	BSbuffer[s_flag].Loadcel1[Sub_cnt] = __LL_ADC_CALC_DATA_TO_VOLTAGE(
			VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC2),
			LL_ADC_RESOLUTION_12B);  // Toe

	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	BSbuffer[s_flag].Loadcel2[Sub_cnt] = __LL_ADC_CALC_DATA_TO_VOLTAGE(
			VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC1),
			LL_ADC_RESOLUTION_12B);  //Heel

	// ADC3 Ch2 PC1 read
}

////Toe: ADC1_Ch3_PC2=ADC1  Heel: ADC2_Ch4_PC3=ADC2
//void F_Sensor_ADC_Store(void)
//{
//	// ADC3 Ch1 PC0 read
//	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
//
//	  LL_ADC_REG_StartConversion(ADC3);
//	  ADC_TimeOut_Count_Buf=0;
//	  ADC_Error_Buf=0;
//	  while ((LL_ADC_IsActiveFlag_EOC(ADC3) == 0))
//	  {
//		  Wait_for_ADC_timeout_While_Datalog();
//	  }
//	  LL_ADC_ClearFlag_EOC(ADC3);
//	  // toe uninstrumented side
////	  BSbuffer[s_flag].other_fsr1[Sub_cnt]= __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC3), LL_ADC_RESOLUTION_12B);  // Toe
//
//	  LL_ADC_REG_StartConversion(ADC2);
//	  ADC_TimeOut_Count_Buf=0;
//	  ADC_Error_Buf=0;
//	  while ((LL_ADC_IsActiveFlag_EOC(ADC2) == 0))
//	  {
//		  Wait_for_ADC_timeout_While_Datalog();
//	  }
//	  LL_ADC_ClearFlag_EOC(ADC2);
//	  BSbuffer[s_flag].Loadcel1[Sub_cnt]= __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC2), LL_ADC_RESOLUTION_12B);  // Toe
//
//
//	  LL_ADC_REG_StartConversion(ADC1);
//	  ADC_TimeOut_Count_Buf=0;
//	  ADC_Error_Buf=0;
//	  while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0))
//	  {
//		  Wait_for_ADC_timeout_While_Datalog();
//	  }
//	  LL_ADC_ClearFlag_EOC(ADC1);
//	  BSbuffer[s_flag].Loadcel2[Sub_cnt]= __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC1), LL_ADC_RESOLUTION_12B);  //Heel
//
//
//	  // ADC3 Ch2 PC1 read
//
//		LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
//
//		  LL_ADC_REG_StartConversion(ADC3);
//		  ADC_TimeOut_Count_Buf=0;
//		  ADC_Error_Buf=0;
//		  while ((LL_ADC_IsActiveFlag_EOC(ADC3) == 0))
//		  {
//			  Wait_for_ADC_timeout_While_Datalog();
//		  }
//		  LL_ADC_ClearFlag_EOC(ADC3);
//		  // heel uninstrumented side
////	  BSbuffer[s_flag].Loadcel1[Sub_cnt]= __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_REG_ReadConversionData12(ADC3), LL_ADC_RESOLUTION_12B);  // Toe
//
//}

void read_Toe_ADC(void) {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3,
			LL_ADC_SAMPLINGTIME_247CYCLES_5); // FSR2=PC2=Ch3--Toe
//Toe
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Test1 = LL_ADC_REG_ReadConversionData12(ADC1);

	Test1_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, Test1,
			LL_ADC_RESOLUTION_12B);  // Toe
}

void read_Heel_ADC(void) {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4,
			LL_ADC_SAMPLINGTIME_247CYCLES_5); // FSR1=PC3=Ch4--Heel
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Test2 = LL_ADC_REG_ReadConversionData12(ADC1);

	Test2_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, Test2,
			LL_ADC_RESOLUTION_12B);  //Heel

}

void read_Load1_ADC(void) {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2,
			LL_ADC_SAMPLINGTIME_247CYCLES_5);
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Test3 = LL_ADC_REG_ReadConversionData12(ADC1);

	Test3_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, Test3,
			LL_ADC_RESOLUTION_12B);

}

void read_Load2_ADC(void) {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,
			LL_ADC_SAMPLINGTIME_247CYCLES_5);
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Test4 = LL_ADC_REG_ReadConversionData12(ADC1);

	Test4_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, Test4,
			LL_ADC_RESOLUTION_12B);

}

void read_add1_ADC(void) {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_10);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_10,
			LL_ADC_SAMPLINGTIME_247CYCLES_5);
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Test5 = LL_ADC_REG_ReadConversionData12(ADC1);

	Test5_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, Test5,
			LL_ADC_RESOLUTION_12B);

}

void read_add2_ADC(void) {
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_11);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11,
			LL_ADC_SAMPLINGTIME_247CYCLES_5);
	LL_ADC_REG_StartConversion(ADC1);
	ADC_TimeOut_Count_Buf = 0;
	ADC_Error_Buf = 0;
	while ((LL_ADC_IsActiveFlag_EOC(ADC1) == 0)) {
		Wait_for_ADC_timeout_While_Datalog();
	}
	LL_ADC_ClearFlag_EOC(ADC1);
	Test6 = LL_ADC_REG_ReadConversionData12(ADC1);

	Test6_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, Test6,
			LL_ADC_RESOLUTION_12B);

}

void all_ADC_read_test(void) {
	//Toe: ADC1_Ch3_PC2  Heel: ADC1_Ch4_PC3

	read_Toe_ADC();

//	  Heel
	read_Heel_ADC();

	read_Load1_ADC();
	read_Load2_ADC();
	read_add1_ADC();
	read_add2_ADC();

}

void all_ADC_read(void) {
	//LL_GPIO_ResetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);            // Battery Monitor On
	ConversionStartPoll_ADC_GrpRegular_Buf();       // Start a Single conversion
	//LL_GPIO_SetOutputPin(GPIOC, BATTERY_TEST_OD_PIN);         // Battery Monitor Off
//		BSbuffer[s_flag].Flex1[Sub_cnt]= aADCxConvertedData[0];
//		BSbuffer[s_flag].Flex2[Sub_cnt]= aADCxConvertedData[1];
}

void conf_ADC_DMA_poll(void) {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_DMA_ConfigTransfer(DMA1,
	LL_DMA_CHANNEL_1,
	LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
	LL_DMA_MODE_CIRCULAR |
	LL_DMA_PERIPH_NOINCREMENT |
	LL_DMA_MEMORY_INCREMENT |
	LL_DMA_PDATAALIGN_HALFWORD |
	LL_DMA_MDATAALIGN_HALFWORD |
	LL_DMA_PRIORITY_HIGH);

	/* Select ADC as DMA transfer request */
	LL_DMA_SetPeriphRequest(DMA1,
	LL_DMA_CHANNEL_1,
	LL_DMA_REQUEST_0);

	/* Set DMA transfer addresses of source and destination */
	LL_DMA_ConfigAddresses(DMA1,
	LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
			(uint32_t) &aADCxConvertedData,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	/* Set DMA transfer size */
	LL_DMA_SetDataLength(DMA1,
	LL_DMA_CHANNEL_1, 3);

	LL_DMA_EnableChannel(DMA1,
	LL_DMA_CHANNEL_1);

}

void Battery_Monitor_Voltage_check(void) {

}

void ADC_Reset(void) {

}

void Data_Pause_Resume_PC0_EXTI_conf(void) {
	NVIC_EnableIRQ(EXTI0_IRQn); // Enabling External GPIO pin interrupt (EXTI9-5) for DataLog Pause-Continue
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG); // Enable Sysconfig clock to enable EXTI pin interrupts
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE0); // Set PC0 pin EXTI interrupt in EXTI9-5 line
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);      // Enable PC0 pin EXTI interrupt
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0); // Interrupt configured for Falling edge
}

//void ACC_GPIO_INIT(void) {
//	// Configure SCK Pin connected to PA5, MISO PA6, MOSI PA7
//	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
//	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
//	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
//	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
//
//	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);
//	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
//	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
//	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);
//
//	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
//	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
//	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
//	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
//}

void ACC2_GPIO_INIT(void) {
	// Configure SCK Pin connected to PB13, MISO PC2, MOSI PB15
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);

	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_2, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_15, LL_GPIO_AF_5);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_15, LL_GPIO_PULL_NO);
}

//void P_IMU1_SPI1_Init(void)  //MPU9250
//{
//
//	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
//
//	ACC_GPIO_INIT();
//
//	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV4); // Must need 8MHz for MPU 9250, Lets try with 6MHz
//	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
//	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
//	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
//	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
//	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
//	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
//	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
//	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
//	LL_SPI_Enable(SPI1);
//	LL_SPI_EnableNSSPulseMgt(SPI1);
//
//}

//void MCP_setup(void) {
//	LL_SPI_InitTypeDef SPI_InitStruct;
//
//	LL_GPIO_InitTypeDef GPIO_InitStruct;
//
//	/* Peripheral clock enable */
//	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
//	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
//	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
//
//	GPIO_InitStruct.Pin = SPI2_SCK_IMU_PIN;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//	LL_GPIO_Init(SPI2_SCK_IMU_GPIO_PORT, &GPIO_InitStruct);
//
//	GPIO_InitStruct.Pin = SPI2_MISO_IMU_PIN;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//	LL_GPIO_Init(SPI2_MISO_IMU_GPIO_PORT, &GPIO_InitStruct);
//
//	GPIO_InitStruct.Pin = SPI2_MOSI_IMU_PIN;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//	LL_GPIO_Init(SPI2_MOSI_IMU_GPIO_PORT, &GPIO_InitStruct);
//
//	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
//	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
//	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
//	/*Mode 3 (Mode 1,1) */
//	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH; /*Clock 1 when idle and 0 when active */
//	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE; //Second clock transition is the first data capture edge
//	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
//
//	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
//	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
//	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
//	SPI_InitStruct.CRCPoly = 7; //?
//	LL_SPI_Init(SPI2, &SPI_InitStruct);
//
//	LL_SPI_DisableNSSPulseMgt(SPI2);
//
//	LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);
//	LL_SPI_DisableIT_RXNE(SPI2);
//	LL_SPI_Enable(SPI2);
//
//	delay_us(10000);
//
//}

/* SPI2 init function */
void P_IMU3_SPI2_Init(void) {

	LL_SPI_InitTypeDef SPI_InitStruct;

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* Peripheral clock enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

	GPIO_InitStruct.Pin = SPI2_SCK_IMU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(SPI2_SCK_IMU_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI2_MISO_IMU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(SPI2_MISO_IMU_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI2_MOSI_IMU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(SPI2_MOSI_IMU_GPIO_PORT, &GPIO_InitStruct);

	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;

	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(SPI2, &SPI_InitStruct);

	LL_SPI_DisableNSSPulseMgt(SPI2);

	LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);
	LL_SPI_DisableIT_RXNE(SPI2);
	LL_SPI_Enable(SPI2);

	delay_us(10000);

}

void Configure_USART_1(void)   // USART1: PB6-Tx, (PA10-Rx disabled)
{

// Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1 clock enable, Clock source Default peripheral clock PCLK2, APB2

// Default parameter: 8 data bit, 1 start bit, 1 stop bit, no parity, no hardware flow control

	MODIFY_REG(USART1->CR1, USART_CR1_RE | USART_CR1_TE,
			(USART_CR1_TE |USART_CR1_RE)); // TX/RX direction BidirecttionalLL_USART_DIRECTION_TX_RX

	USART1->BRR =
			(uint16_t) (__LL_USART_DIV_SAMPLING16(SystemCoreClock, 115200)); // Baud rate 115200

	USART1->CR1 |= USART_CR1_UE;  // Unable USART1
}

uint8_t USART1_read(void)    // USART Rx PA10 pin is connected with Push Button
{
	while (!(USART1->ISR & USART_ISR_RXNE))
		;
	return ((uint8_t) (USART1->RDR & 0xFF));

}

void USART1_wr_print(uint8_t *buffer, uint32_t nBytes) {
	for (int i = 0; i < nBytes; i++) {
		while (!(USART1->ISR & USART_ISR_TXE))
			;
		USART1->TDR = buffer[i] & 0xFF;

		while (!(USART1->ISR & USART_ISR_TC))
			;
		USART1->ISR &= ~USART_ISR_TC;
	}
}

void Write_Timestamp_In_SD(void) // Demo RTC timestamp write in FATFS SD card for Debug
{
	// Read RTC timestamp and write in a string as hh:mm:ss format
	sprintf((char*) textString, "%.2d_%.2d_%.2d..\n",
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));

	if (f_write(&MyFile, textString, strlen(textString), &BytesWritten1)
			!= FR_OK)   // Write the RTC timestamp string to the file
			{
		// debugging
	}

	if (f_sync(&MyFile) != FR_OK) // F_Sync will update SD FATFS file without closing it
			{    // debugging
	}

}

void FPGA_Programming_loop(void) {
	//  FPGA_POWER_ON();	 	 //Code for enabling FPGA programming
	//	 while(1);
}

void prepare_low_battery(void) {
#ifdef Bottle_Device
	//ALL_LED_OFF();
#else
//  NVIC_SystemReset();
#endif
}

void Start_Data_Collection_at_Reset(void) {
	Battery_Monitor_Voltage_check();

	if (Battery_ADC_Value < 2100)             // Battery Cut off Value 3V 2294 ?
			{
		Data_log_Start_Resume = 0;
		//       Gr_LED_Blinking_2s();
		Pros_state = Low_Battery_Mode; // Prepare to enter Low_Battery_Mode mode
	} else {
		//				 GREEN_LED_ON();// Green LED indicates Data log starts
		FATFS_Init();
		Open_File_For_Sensor_Write();
		// Re-Initializing FATFS (UnixTimestamp .BIN file) and Left open for sensor write
		Reset_All();         // Reset All Buffer parameters for FATFS SDIO write
		// 	Camera_Init_Sleep();

		delay_us(5000000);
		// 	ACC_GPIO_INIT();
		Start_LPTIMCounter2(0xFF); // Start 10ms LPTIM interrupt
		LL_LPTIM_EnableIT_ARRM(LPTIM2);
		NVIC_EnableIRQ(LPTIM2_IRQn);     // Enabling LPTIM2 Timer 10ms Interrupt
		NVIC_EnableIRQ(EXTI9_5_IRQn);
		Data_log_Start_Resume = 1;
	}

}

void retry_FATFS_mnt_twice(void) {
	// Try twice if FATFS fail to mount
	if (Fat_mnt_fail == 1) {
		Fat_mnt_fail = 0;
		delay_us(5000000);  //5s delay before retry
		FATFS_Init();
	}
	if (Fat_mnt_fail == 1) {
		Fat_mnt_fail = 0;
		delay_us(5000000);  //5s delay before retry
		FATFS_Init();
	}
	if (Fat_mnt_fail == 1) //3rd time failed Fatal error, close and goto lowBatery
			{
		Pros_state = Low_Battery_Mode;
		//	 	    RED_LED_ON();
		FATAL_err = 1;
	}
}

int FPGA_Loop_Error_cnt;

void Clear_ErrorTimestamp_Buffer(void) {
	ErrorCodeBuf = 0;
	ErrorTimeStampBuf = 0;
}

void Save_ErrorTimestamp_Buffer(uint8_t ErrorCode) {
	ErrorCodeBuf = ErrorCode;
	ErrorTimeStampBuf =
			__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC))
					* 10000+__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC))*100+__RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));

	Error_reg_log.Error[Error_indx] = ErrorCode;
	Error_reg_log.ErrorTime[Error_indx] = ErrorTimeStampBuf;
	Error_indx++;
}

void AIM_Error_Handler(int AIM_Error_Code) {
	switch (AIM_Error_Code) {

	case FPGA_SPI_R_Error:
		Save_ErrorTimestamp_Buffer(FPGA_SPI_R_Error);
		FATAL_Error = 1;

		break;

	case f_write_E:
		file_reopen();
		if (Fat_write_read_fail == 1) // Could not solve write issue by reopening file
				{
			Fat_read_mnt_fail = 0;
			Save_ErrorTimestamp_Buffer(f_write_E);
			File_Sensor_write_issue = 1;
			FATAL_Error = 1;
		}

		break;

	case f_open_E:

		file_mnt_reopen();
		if (Fat_read_mnt_fail == 1) // Could not solve read issue by FATFS mount
				{
			Fat_read_mnt_fail = 0;
			Save_ErrorTimestamp_Buffer(f_open_E);
			FATAL_Error = 1;
			Skip_FATFS = 1;
		}

		break;

	case f_mount_E:
		Try_FATFS_Mount();
		if (Skip_FATFS == 1)  // Could not solve FATFS mount
				{
			Save_ErrorTimestamp_Buffer(f_mount_E);
			FATAL_Error = 1;
		}
		break;

	case f_sync_E:
		file_reopen_sync();
		if (Fat_sync_read_fail == 1) // Could not solve write issue by reopening file
				{
			Fat_sync_read_fail = 0;
			Save_ErrorTimestamp_Buffer(f_sync_E);
			File_Sensor_close_issue = 1;
			FATAL_Error = 1;
		}

		break;

	case SPI_Acc_Error:

		if (SPI_ErrorCnt > 3) {
			SPI_ErrorCnt = 0;
			FATAL_Error = 1;
			Save_ErrorTimestamp_Buffer(SPI_Acc_Error);
		} else {
			SPI_ErrorCnt++;
		}
		break;

	case ADC_R_Error:
		ADC_Reset();
		if (ADC_ErrorCnt > 3) {
			ADC_ErrorCnt = 0;
			FATAL_Error = 1;
			Save_ErrorTimestamp_Buffer(ADC_R_Error);
		} else {
			ADC_ErrorCnt++;
		}

		break;

	default:

		break;

	}
}

void AIM_DataStart_at_Reset(void) {
	SD_POWER_ON();
	SD_cnt_limit = 3;                    // After 50 write, File will Sync
	Data_log_Start_Resume = 1;
	FATFS_Init();
	Open_File_For_Sensor_Write();
	Reset_All();

	// Greg start comment
//	  Start_LPTIMCounter2(0x10); //0xFF //0x4032768
	// Greg end comment

	// Greg start
	// Start interrupt (speed = 32,768 Hz / hex_value, i.e. 0x40 = 64 so speed = 32,768/64 = 512 Hz)
	Start_LPTIMCounter2(0x40);
	// Greg end

	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	NVIC_EnableIRQ(LPTIM2_IRQn);
	Pros_state = LP_STOP;
	USB_Mode = 1;
	NVIC_EnableIRQ(EXTI9_5_IRQn);

}

float acc_divider;
float gyro_divider;

int calib_data[3];
float Magnetometer_ASA[3];

float accel_data[3];
float temperature;
float gyro_data[3];
float mag_data[3];
int16_t mag_data_raw[3];

// mpu9250 registers
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_SELF_TEST_X 0x0D
#define MPUREG_SELF_TEST_Y 0x0E
#define MPUREG_SELF_TEST_Z 0x0F
#define MPUREG_SELF_TEST_A 0x10
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define MPUREG_SMPLRT_DIV 0x19
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_TEMP_OUT_H 0x41
#define MPUREG_TEMP_OUT_L 0x42
#define MPUREG_GYRO_XOUT_H 0x43
#define MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define MPUREG_GYRO_ZOUT_L 0x48
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_EXT_SENS_DATA_01    0x4A
#define MPUREG_EXT_SENS_DATA_02    0x4B
#define MPUREG_EXT_SENS_DATA_03    0x4C
#define MPUREG_EXT_SENS_DATA_04    0x4D
#define MPUREG_EXT_SENS_DATA_05    0x4E
#define MPUREG_EXT_SENS_DATA_06    0x4F
#define MPUREG_EXT_SENS_DATA_07    0x50
#define MPUREG_EXT_SENS_DATA_08    0x51
#define MPUREG_EXT_SENS_DATA_09    0x52
#define MPUREG_EXT_SENS_DATA_10    0x53
#define MPUREG_EXT_SENS_DATA_11    0x54
#define MPUREG_EXT_SENS_DATA_12    0x55
#define MPUREG_EXT_SENS_DATA_13    0x56
#define MPUREG_EXT_SENS_DATA_14    0x57
#define MPUREG_EXT_SENS_DATA_15    0x58
#define MPUREG_EXT_SENS_DATA_16    0x59
#define MPUREG_EXT_SENS_DATA_17    0x5A
#define MPUREG_EXT_SENS_DATA_18    0x5B
#define MPUREG_EXT_SENS_DATA_19    0x5C
#define MPUREG_EXT_SENS_DATA_20    0x5D
#define MPUREG_EXT_SENS_DATA_21    0x5E
#define MPUREG_EXT_SENS_DATA_22    0x5F
#define MPUREG_EXT_SENS_DATA_23    0x60
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL 0x6A
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_BANK_SEL 0x6D
#define MPUREG_MEM_START_ADDR 0x6E
#define MPUREG_MEM_R_W 0x6F
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI 0x75
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E
/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             0x0c//0x18
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

// Configuration bits mpu9250
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG   0x80

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB

uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ;
uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ;

unsigned int set_acc_scale(int scale) {
	unsigned int temp_scale;
	WriteReg(MPUREG_ACCEL_CONFIG, scale);

	switch (scale) {
	case BITS_FS_2G:
		acc_divider = 16384;
		break;
	case BITS_FS_4G:
		acc_divider = 8192;
		break;
	case BITS_FS_8G:
		acc_divider = 4096;
		break;
	case BITS_FS_16G:
		acc_divider = 2048;
		break;
	}
	temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);

	switch (temp_scale) {
	case BITS_FS_2G:
		temp_scale = 2;
		break;
	case BITS_FS_4G:
		temp_scale = 4;
		break;
	case BITS_FS_8G:
		temp_scale = 8;
		break;
	case BITS_FS_16G:
		temp_scale = 16;
		break;
	}
	return temp_scale;
}

unsigned int set_gyro_scale(int scale) {
	unsigned int temp_scale;
	WriteReg(MPUREG_GYRO_CONFIG, scale);

	switch (scale) {
	case BITS_FS_250DPS:
		gyro_divider = 131;
		break;
	case BITS_FS_500DPS:
		gyro_divider = 65.5;
		break;
	case BITS_FS_1000DPS:
		gyro_divider = 32.8;
		break;
	case BITS_FS_2000DPS:
		gyro_divider = 16.4;
		break;
	}

	temp_scale = WriteReg(MPUREG_GYRO_CONFIG | READ_FLAG, 0x00);

	switch (temp_scale) {
	case BITS_FS_250DPS:
		temp_scale = 250;
		break;
	case BITS_FS_500DPS:
		temp_scale = 500;
		break;
	case BITS_FS_1000DPS:
		temp_scale = 1000;
		break;
	case BITS_FS_2000DPS:
		temp_scale = 2000;
		break;
	}
	return temp_scale;
}

#define MPU_InitRegNum 17

void calib_acc() {
	uint8_t response[4];
	int temp_scale;
	//READ CURRENT ACC SCALE
	temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
	set_acc_scale(BITS_FS_8G);
	//ENABLE SELF TEST need modify
	//temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

	ReadRegs(MPUREG_SELF_TEST_X, response, 4);
	calib_data[0] = ((response[0] & 11100000) >> 3)
			| ((response[3] & 00110000) >> 4);
	calib_data[1] = ((response[1] & 11100000) >> 3)
			| ((response[3] & 00001100) >> 2);
	calib_data[2] = ((response[2] & 11100000) >> 3)
			| ((response[3] & 00000011));

	set_acc_scale(temp_scale);
}
float g_bias[3];
float a_bias[3];      // Bias corrections for gyro and accelerometer
#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)

void calib_mag() {
	uint8_t response[3];
	float data;
	int i;
	// Choose either 14-bit or 16-bit magnetometer resolution
	//uint8_t MFS_14BITS = 0; // 0.6 mG per LSB
	uint8_t MFS_16BITS = 1; // 0.15 mG per LSB
	// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	uint8_t M_8HZ = 0x02; // 8 Hz update
	//uint8_t M_100HZ = 0x06; // 100 Hz continuous magnetometer

	/* get the magnetometer calibration */

	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave    addres of AK8963 and set for read.
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); // I2C slave 0 register address from where to begin data transfer
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);  // Read 3 bytes from the magnetometer

	//WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                     // Enable I2C and set bytes
	delay_us(100000);
	//response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00); //Read I2C

	WriteReg(AK8963_CNTL1, 0x00);                    // set AK8963 to Power Down
	delay_us(50000);                    // long wait between AK8963 mode changes
	WriteReg(AK8963_CNTL1, 0x0F);               // set AK8963 to FUSE ROM access
	delay_us(50000);                    // long wait between AK8963 mode changes

	ReadRegs(MPUREG_EXT_SENS_DATA_00, response, 3);
	//response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);              // Read I2C
	for (i = 0; i < 3; i++) {
		data = response[i];
		Magnetometer_ASA[i] = ((data - 128) / 256 + 1)
				* Magnetometer_Sensitivity_Scale_Factor;
	}
	WriteReg(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
	delay_us(50000);
	// Configure the magnetometer for continuous read and highest resolution.
	// Set bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	// register, and enable continuous mode data acquisition (bits [3:0]),
	// 0010 for 8 Hz and 0110 for 100 Hz sample rates.
	WriteReg(AK8963_CNTL1, MFS_16BITS << 4 | M_8HZ); // Set magnetometer data resolution and sample ODR
	delay_us(50000);
}

int16_t bit_data[10], bit_data1[10], bit_data2[10], bit_data3[10];
void read_acc() {
	uint8_t response[6];
	float data;
	int i;
	ReadRegs(MPUREG_ACCEL_XOUT_H, response, 6);
	for (i = 0; i < 3; i++) {
		bit_data[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
		//   accel_data[i] = (float)bit_data;
		// accel_data[i] = data/acc_divider - a_bias[i];
	}

}

void read_acc_gyro(void) {
	uint8_t response[21];
	//  float data;
	int i, j, k;
	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);  // Read 7 bytes from the magnetometer

	ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);
	for (i = 0; i < 3; i++) {
		bit_data[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
	}
	for (j = 4; j < 7; j++) {
		bit_data[j] = ((int16_t) response[j * 2] << 8) | response[j * 2 + 1];
	}

	for (k = 7; k < 10; k++) {
		bit_data[k] = ((int16_t) response[k * 2 + 1] << 8) | response[k * 2];
	}

}

void read_acc_gyro1(void) {
	uint8_t response[21];
	//  float data;
	int i, j, k;
	WriteReg1(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg1(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg1(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs1(MPUREG_ACCEL_XOUT_H, response, 21);
	for (i = 0; i < 3; i++) {
		bit_data1[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
	}
	for (j = 4; j < 7; j++) {
		bit_data1[j] = ((int16_t) response[j * 2] << 8) | response[j * 2 + 1];
	}

	for (k = 7; k < 10; k++) {
		bit_data1[k] = ((int16_t) response[k * 2 + 1] << 8) | response[k * 2];
	}

}

void read_acc_gyro3(void) {
	uint8_t response[21];
	//  float data;
	int i, j, k;
	WriteReg3(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg3(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg3(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs3(MPUREG_ACCEL_XOUT_H, response, 21);
	for (i = 0; i < 3; i++) {
		bit_data3[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
	}
	for (j = 4; j < 7; j++) {
		bit_data3[j] = ((int16_t) response[j * 2] << 8) | response[j * 2 + 1];
	}

	for (k = 7; k < 10; k++) {
		bit_data3[k] = ((int16_t) response[k * 2 + 1] << 8) | response[k * 2];
	}

}

void read_acc_gyro2(void) {
	uint8_t response[21];
	//  float data;
	int i, j, k;
	WriteReg2(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg2(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg2(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs2(MPUREG_ACCEL_XOUT_H, response, 21);
	for (i = 0; i < 3; i++) {
		bit_data2[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
	}
	for (j = 4; j < 7; j++) {
		bit_data2[j] = ((int16_t) response[j * 2] << 8) | response[j * 2 + 1];
	}

	for (k = 7; k < 10; k++) {
		bit_data2[k] = ((int16_t) response[k * 2 + 1] << 8) | response[k * 2];
	}

}

void MPU_9D_store(void) {
	uint8_t response[21];
	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);  // Read 7 bytes from the magnetometer

	ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);

//    BSbuffer[s_flag].X[Sub_cnt]=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].Y[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
//    BSbuffer[s_flag].Z[Sub_cnt]=((int16_t)response[4]<<8)|response[5];
//    BSbuffer[s_flag].GX[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ[Sub_cnt]=((int16_t)response[12]<<8)|response[13];

//    BSbuffer[s_flag].MX[Sub_cnt]=((int16_t)response[15]<<8)|response[14];
//    BSbuffer[s_flag].MY[Sub_cnt]=((int16_t)response[17]<<8)|response[16];
//    BSbuffer[s_flag].MZ[Sub_cnt]=((int16_t)response[19]<<8)|response[18];
}

int IMU1_SPI1_acc_read(void) {
	int16_t data_az;
	uint8_t response[21];
	WriteReg1(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg1(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg1(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs1(MPUREG_ACCEL_XOUT_H, response, 21);

//    data_ax=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].AY1[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
	data_az = ((int16_t) response[4] << 8) | response[5];
//    BSbuffer[s_flag].GX1[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY1[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ1[Sub_cnt]=((int16_t)response[12]<<8)|response[13];
//    BSbuffer[s_flag].MX1[Sub_cnt]=((int16_t)response[15]<<8)|response[14];
//    BSbuffer[s_flag].MY1[Sub_cnt]=((int16_t)response[17]<<8)|response[16];
//    BSbuffer[s_flag].MZ1[Sub_cnt]=((int16_t)response[19]<<8)|response[18];

	return data_az;
}

struct imu_data IMU1_read(void) {
	struct imu_data IMU1;
	uint8_t response[21];
	WriteReg1(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg1(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg1(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs1(MPUREG_ACCEL_XOUT_H, response, 21);

	IMU1.AX = ((int16_t) response[0] << 8) | response[1];
	IMU1.AY = ((int16_t) response[2] << 8) | response[3];
	IMU1.AZ = ((int16_t) response[4] << 8) | response[5];
	IMU1.GX = ((int16_t) response[8] << 8) | response[9];
	IMU1.GY = ((int16_t) response[10] << 8) | response[11];
	IMU1.GZ = ((int16_t) response[12] << 8) | response[13];

	return IMU1;
}

void Knee_data_storeIMU(int value1, int value2, int value3, int value4,
		int value5, int value6) {

	BSbuffer[s_flag].AX1[Sub_cnt] = value1;
	BSbuffer[s_flag].AY1[Sub_cnt] = value2;
	BSbuffer[s_flag].AZ1[Sub_cnt] = value3;
	BSbuffer[s_flag].GX1[Sub_cnt] = value4;
	BSbuffer[s_flag].GY1[Sub_cnt] = value5;
	BSbuffer[s_flag].GZ1[Sub_cnt] = value6;

}

//void MPU_9D_store_IMU1_SPI1(void)
//{
//    uint8_t response[21];
//    WriteReg1(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
//    WriteReg1(MPUREG_I2C_SLV0_REG, AK8963_HXL);                // I2C slave 0 register address from where to begin data transfer
//    WriteReg1(MPUREG_I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
//
//    ReadRegs1(MPUREG_ACCEL_XOUT_H,response,21);
//
//    BSbuffer[s_flag].AX1[Sub_cnt]=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].AY1[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
//    BSbuffer[s_flag].AZ1[Sub_cnt]=((int16_t)response[4]<<8)|response[5];
//    BSbuffer[s_flag].GX1[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY1[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ1[Sub_cnt]=((int16_t)response[12]<<8)|response[13];
////    BSbuffer[s_flag].MX1[Sub_cnt]=((int16_t)response[15]<<8)|response[14];
////    BSbuffer[s_flag].MY1[Sub_cnt]=((int16_t)response[17]<<8)|response[16];
////    BSbuffer[s_flag].MZ1[Sub_cnt]=((int16_t)response[19]<<8)|response[18];
//}

void MPU_9D_store_IMU2_SPI1(void) {
	uint8_t response[21];
	WriteReg1_imu2(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg1_imu2(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg1_imu2(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs1_imu2(MPUREG_ACCEL_XOUT_H, response, 21);

//    BSbuffer[s_flag].AX2[Sub_cnt]=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].AY2[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
//    BSbuffer[s_flag].AZ2[Sub_cnt]=((int16_t)response[4]<<8)|response[5];
//    BSbuffer[s_flag].GX2[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY2[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ2[Sub_cnt]=((int16_t)response[12]<<8)|response[13];

}

void MPU_9D_store_IMU4_SPI3(void) {
	uint8_t response[21];
	//change all writereg3 for imu5
	WriteReg3(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg3(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg3(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs3(MPUREG_ACCEL_XOUT_H, response, 21); //create and change here for IMU5

	//change these to store for IMU5
//    BSbuffer[s_flag].AX4[Sub_cnt]=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].AY4[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
//    BSbuffer[s_flag].AZ4[Sub_cnt]=((int16_t)response[4]<<8)|response[5];
//    BSbuffer[s_flag].GX4[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY4[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ4[Sub_cnt]=((int16_t)response[12]<<8)|response[13];

	//    BSbuffer[s_flag].MX[Sub_cnt]=((int16_t)response[15]<<8)|response[14];
//    BSbuffer[s_flag].MY[Sub_cnt]=((int16_t)response[17]<<8)|response[16];
	//   BSbuffer[s_flag].MZ[Sub_cnt]=((int16_t)response[19]<<8)|response[18];
//    BSbuffer[s_flag].marking_sw[Sub_cnt]=(int16_t) (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_10));
}

void MPU_9D_store_IMU5_SPI3(void) {
	uint8_t response[21];
	//change all writereg3 for imu5
	WriteReg3_imu5(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg3_imu5(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg3_imu5(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs3_imu5(MPUREG_ACCEL_XOUT_H, response, 21); //create and change here for IMU5

	//change these to store for IMU5
//    BSbuffer[s_flag].AX5[Sub_cnt]=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].AY5[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
//    BSbuffer[s_flag].AZ5[Sub_cnt]=((int16_t)response[4]<<8)|response[5];
//    BSbuffer[s_flag].GX5[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY5[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ5[Sub_cnt]=((int16_t)response[12]<<8)|response[13];

}

//void MPU_9D_store_IMU3_SPI2(void)
void MPU_9D_store_IMU3_SPI2(int value1, int value2, int value3, int value4,
		int value5, int value6) {
//    uint8_t response[21];
//    WriteReg2(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
//    WriteReg2(MPUREG_I2C_SLV0_REG, AK8963_HXL);                // I2C slave 0 register address from where to begin data transfer
//    WriteReg2(MPUREG_I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
//
//    ReadRegs2(MPUREG_ACCEL_XOUT_H,response,21);

//    BSbuffer[s_flag].AX3[Sub_cnt]=((int16_t)response[0]<<8)|response[1];
//    BSbuffer[s_flag].AY3[Sub_cnt]=((int16_t)response[2]<<8)|response[3];
//    BSbuffer[s_flag].AZ3[Sub_cnt]=((int16_t)response[4]<<8)|response[5];
//    BSbuffer[s_flag].GX3[Sub_cnt]=((int16_t)response[8]<<8)|response[9];
//    BSbuffer[s_flag].GY3[Sub_cnt]=((int16_t)response[10]<<8)|response[11];
//    BSbuffer[s_flag].GZ3[Sub_cnt]=((int16_t)response[12]<<8)|response[13];
//	mcpwrite(0x0F,0x60);

//    BSbuffer[s_flag].AX3[Sub_cnt]=value1;
//    BSbuffer[s_flag].AY3[Sub_cnt]=value2;
//    BSbuffer[s_flag].AZ3[Sub_cnt]=value3;
//    BSbuffer[s_flag].GX3[Sub_cnt]=value4;
//    BSbuffer[s_flag].GY3[Sub_cnt]=value5;
//    BSbuffer[s_flag].GZ3[Sub_cnt]=value6;

//    BSbuffer[s_flag].MX2[Sub_cnt]=((int16_t)response[15]<<8)|response[14];
//    BSbuffer[s_flag].MY2[Sub_cnt]=((int16_t)response[17]<<8)|response[16];
//    BSbuffer[s_flag].MZ2[Sub_cnt]=((int16_t)response[19]<<8)|response[18];
}

//void Knee_data_store(int val1, int val2, int val3, int val4, int val5, int val6, int val7, int val8)
//{
//	BSbuffer[s_flag].data1[Sub_cnt]=val1;
//	BSbuffer[s_flag].data2[Sub_cnt]=val2;
//	BSbuffer[s_flag].data3[Sub_cnt]=val3;
//	BSbuffer[s_flag].data4[Sub_cnt]=val4;
//	BSbuffer[s_flag].data5[Sub_cnt]=val5;
//	BSbuffer[s_flag].data6[Sub_cnt]=val6;
//	BSbuffer[s_flag].data7[Sub_cnt]=val7;
//	BSbuffer[s_flag].data8[Sub_cnt]=val8;
//
//
//}

void Knee_data_store(int val1, int val2, int val3, int val4) {
	BSbuffer[s_flag].data1[Sub_cnt] = val1;
	BSbuffer[s_flag].data2[Sub_cnt] = val2;
	BSbuffer[s_flag].data3[Sub_cnt] = val3;
	BSbuffer[s_flag].data4[Sub_cnt] = val4;
//	BSbuffer[s_flag].data5[Sub_cnt]=val5;
//	BSbuffer[s_flag].data6[Sub_cnt]=val6;
//	BSbuffer[s_flag].data7[Sub_cnt]=val7;
//	BSbuffer[s_flag].data8[Sub_cnt]=val8;
}

void Knee_data_store1(int val5, int val6, int val7, int val8) {
//	BSbuffer[s_flag].data1[Sub_cnt]=val1;
//	BSbuffer[s_flag].data2[Sub_cnt]=val2;
//	BSbuffer[s_flag].data3[Sub_cnt]=val3;
//	BSbuffer[s_flag].data4[Sub_cnt]=val4;
	BSbuffer[s_flag].data5[Sub_cnt] = val5;
	BSbuffer[s_flag].data6[Sub_cnt] = val6;
	BSbuffer[s_flag].data7[Sub_cnt] = val7;
	BSbuffer[s_flag].data8[Sub_cnt] = val8;
}

//void Knee_data_store1( int val5, int val6)
//{
////	BSbuffer[s_flag].data1[Sub_cnt]=val1;
////	BSbuffer[s_flag].data2[Sub_cnt]=val2;
////	BSbuffer[s_flag].data3[Sub_cnt]=val3;
////	BSbuffer[s_flag].data4[Sub_cnt]=val4;
//	BSbuffer[s_flag].data5[Sub_cnt]=val5;
//	BSbuffer[s_flag].data6[Sub_cnt]=val6;
////	BSbuffer[s_flag].data7[Sub_cnt]=val7;
////	BSbuffer[s_flag].data8[Sub_cnt]=val8;
//}
//
//void Knee_data_store2(int val7, int val8)
//{
////	BSbuffer[s_flag].data1[Sub_cnt]=val1;
////	BSbuffer[s_flag].data2[Sub_cnt]=val2;
////	BSbuffer[s_flag].data3[Sub_cnt]=val3;
////	BSbuffer[s_flag].data4[Sub_cnt]=val4;
////	BSbuffer[s_flag].data5[Sub_cnt]=val5;
////	BSbuffer[s_flag].data6[Sub_cnt]=val6;
//	BSbuffer[s_flag].data7[Sub_cnt]=val7;
//	BSbuffer[s_flag].data8[Sub_cnt]=val8;
//}

void read_all() {
	uint8_t response[21];
	int16_t bit_data;
	float data;
	int i;

	// Send I2C command at first
	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);  // Read 7 bytes from the magnetometer
	// must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

	ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);
	// Get accelerometer value
	for (i = 0; i < 3; i++) {
		bit_data = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
		//      data = (float)bit_data;
		//      accel_data[i] = data/acc_divider - a_bias[i];
	}
	// Get temperature
	//  bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
	//  data = (float)bit_data;
	//  temperature = ((data-21)/333.87)+21;
	// Get gyroscope value
	for (i = 4; i < 7; i++) {
		bit_data = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
		//   data = (float)bit_data;
		//   gyro_data[i-4] = data/gyro_divider - g_bias[i-4];
	}
	// Get Magnetometer value
	for (i = 7; i < 10; i++) {
		mag_data_raw[i - 7] = ((int16_t) response[i * 2 + 1] << 8)
				| response[i * 2];
		data = (float) mag_data_raw[i - 7];
		mag_data[i - 7] = data * Magnetometer_ASA[i - 7];
	}
}

void MPU1_SPI1_init(void) {
	float temp[3];
	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { { BIT_H_RESET,
			MPUREG_PWR_MGMT_1 },        // Reset Device
			{ 0x01, MPUREG_PWR_MGMT_1 },               // Clock Source
			{ 0x00, MPUREG_PWR_MGMT_2 },               // Enable Acc & Gyro
			{ low_pass_filter, MPUREG_CONFIG }, // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{ BITS_FS_1000DPS, MPUREG_GYRO_CONFIG },    // +-250dps
			{ BITS_FS_8G, MPUREG_ACCEL_CONFIG },       // +-2G
			{ low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2 }, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{ 0x12, MPUREG_INT_PIN_CFG },      //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{ 0x30, MPUREG_USER_CTRL }, // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
			{ 0x0D, MPUREG_I2C_MST_CTRL }, // I2C configuration multi-master  IIC 400KHz

			{ AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR }, // Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

			{ AK8963_CNTL2, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
			{ 0x01, MPUREG_I2C_SLV0_DO },   // Reset AK8963
			{ 0x81, MPUREG_I2C_SLV0_CTRL }, // Enable I2C and set 1 byte

			{ AK8963_CNTL1, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
			{ 0x16, MPUREG_I2C_SLV0_DO }, // Register value to 100Hz continuous measurement in 16bit
#else
			{	0x12, MPUREG_I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
			{ 0x81, MPUREG_I2C_SLV0_CTRL }  //Enable I2C and set 1 byte

	};

	for (i = 0; i < MPU_InitRegNum; i++) {
		WriteReg1(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		delay_us(1000); // I2C must slow down the write speed, otherwise it won't work
	}
	//  set_acc_scale(BITS_FS_2G);
	//   set_gyro_scale(BITS_FS_250DPS);

	//   calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	//   return 0;

}

void MPU2_SPI1_init(void) {
	float temp[3];
	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { { BIT_H_RESET,
			MPUREG_PWR_MGMT_1 },        // Reset Device
			{ 0x01, MPUREG_PWR_MGMT_1 },               // Clock Source
			{ 0x00, MPUREG_PWR_MGMT_2 },               // Enable Acc & Gyro
			{ low_pass_filter, MPUREG_CONFIG }, // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{ BITS_FS_1000DPS, MPUREG_GYRO_CONFIG },    // +-250dps
			{ BITS_FS_8G, MPUREG_ACCEL_CONFIG },       // +-2G
			{ low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2 }, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{ 0x12, MPUREG_INT_PIN_CFG },      //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{ 0x30, MPUREG_USER_CTRL }, // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
			{ 0x0D, MPUREG_I2C_MST_CTRL }, // I2C configuration multi-master  IIC 400KHz

			{ AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR }, // Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

			{ AK8963_CNTL2, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
			{ 0x01, MPUREG_I2C_SLV0_DO },   // Reset AK8963
			{ 0x81, MPUREG_I2C_SLV0_CTRL }, // Enable I2C and set 1 byte

			{ AK8963_CNTL1, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
			{ 0x16, MPUREG_I2C_SLV0_DO }, // Register value to 100Hz continuous measurement in 16bit
#else
			{	0x12, MPUREG_I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
			{ 0x81, MPUREG_I2C_SLV0_CTRL }  //Enable I2C and set 1 byte

	};

	for (i = 0; i < MPU_InitRegNum; i++) {
		WriteReg1_imu2(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		delay_us(1000); // I2C must slow down the write speed, otherwise it won't work
	}
	//  set_acc_scale(BITS_FS_2G);
	//   set_gyro_scale(BITS_FS_250DPS);

	//   calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	//   return 0;

}

void MPU4_SPI3_init(void) {
//	float temp[3];
	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { { BIT_H_RESET,
			MPUREG_PWR_MGMT_1 },        // Reset Device
			{ 0x01, MPUREG_PWR_MGMT_1 },               // Clock Source
			{ 0x00, MPUREG_PWR_MGMT_2 },               // Enable Acc & Gyro
			{ low_pass_filter, MPUREG_CONFIG }, // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{ BITS_FS_1000DPS, MPUREG_GYRO_CONFIG },    // +-250dps
			{ BITS_FS_8G, MPUREG_ACCEL_CONFIG },       // +-2G
			{ low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2 }, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{ 0x12, MPUREG_INT_PIN_CFG },      //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{ 0x30, MPUREG_USER_CTRL }, // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
			{ 0x0D, MPUREG_I2C_MST_CTRL }, // I2C configuration multi-master  IIC 400KHz

			{ AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR }, // Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

			{ AK8963_CNTL2, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
			{ 0x01, MPUREG_I2C_SLV0_DO },   // Reset AK8963
			{ 0x81, MPUREG_I2C_SLV0_CTRL }, // Enable I2C and set 1 byte

			{ AK8963_CNTL1, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
			{ 0x16, MPUREG_I2C_SLV0_DO }, // Register value to 100Hz continuous measurement in 16bit
#else
			{	0x12, MPUREG_I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
			{ 0x81, MPUREG_I2C_SLV0_CTRL }  //Enable I2C and set 1 byte

	};

	//	 LED_Nucleo_OFF();
	for (i = 0; i < MPU_InitRegNum; i++) {
		// change here
		WriteReg3(MPU_Init_Data[i][1], MPU_Init_Data[i][0]); // change here
		delay_us(1000); // I2C must slow down the write speed, otherwise it won't work
	}
	//  set_acc_scale(BITS_FS_2G);
	//   set_gyro_scale(BITS_FS_250DPS);

	//   calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	//   return 0;

}

void MPU5_SPI3_init(void) // created later for IMU5
{
//	float temp[3];
	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { { BIT_H_RESET,
			MPUREG_PWR_MGMT_1 },        // Reset Device
			{ 0x01, MPUREG_PWR_MGMT_1 },               // Clock Source
			{ 0x00, MPUREG_PWR_MGMT_2 },               // Enable Acc & Gyro
			{ low_pass_filter, MPUREG_CONFIG }, // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{ BITS_FS_1000DPS, MPUREG_GYRO_CONFIG },    // +-250dps
			{ BITS_FS_8G, MPUREG_ACCEL_CONFIG },       // +-2G
			{ low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2 }, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{ 0x12, MPUREG_INT_PIN_CFG },      //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{ 0x30, MPUREG_USER_CTRL }, // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
			{ 0x0D, MPUREG_I2C_MST_CTRL }, // I2C configuration multi-master  IIC 400KHz

			{ AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR }, // Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

			{ AK8963_CNTL2, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
			{ 0x01, MPUREG_I2C_SLV0_DO },   // Reset AK8963
			{ 0x81, MPUREG_I2C_SLV0_CTRL }, // Enable I2C and set 1 byte

			{ AK8963_CNTL1, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
			{ 0x16, MPUREG_I2C_SLV0_DO }, // Register value to 100Hz continuous measurement in 16bit
#else
			{	0x12, MPUREG_I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
			{ 0x81, MPUREG_I2C_SLV0_CTRL }  //Enable I2C and set 1 byte

	};

	//	 LED_Nucleo_OFF();
	for (i = 0; i < MPU_InitRegNum; i++) {
		// change here
		WriteReg3_imu5(MPU_Init_Data[i][1], MPU_Init_Data[i][0]); // change here
		delay_us(1000); // I2C must slow down the write speed, otherwise it won't work
	}
	//  set_acc_scale(BITS_FS_2G);
	//   set_gyro_scale(BITS_FS_250DPS);

	//   calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	//   return 0;

}

void MPU3_SPI2_init(void) {
	float temp[3];
	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { { BIT_H_RESET,
			MPUREG_PWR_MGMT_1 },        // Reset Device
			{ 0x01, MPUREG_PWR_MGMT_1 },               // Clock Source
			{ 0x00, MPUREG_PWR_MGMT_2 },               // Enable Acc & Gyro
			{ low_pass_filter, MPUREG_CONFIG }, // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{ BITS_FS_1000DPS, MPUREG_GYRO_CONFIG },    // +-250dps
			{ BITS_FS_8G, MPUREG_ACCEL_CONFIG },       // +-2G
			{ low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2 }, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{ 0x12, MPUREG_INT_PIN_CFG },      //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{ 0x30, MPUREG_USER_CTRL }, // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
			{ 0x0D, MPUREG_I2C_MST_CTRL }, // I2C configuration multi-master  IIC 400KHz

			{ AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR }, // Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

			{ AK8963_CNTL2, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
			{ 0x01, MPUREG_I2C_SLV0_DO },   // Reset AK8963
			{ 0x81, MPUREG_I2C_SLV0_CTRL }, // Enable I2C and set 1 byte

			{ AK8963_CNTL1, MPUREG_I2C_SLV0_REG }, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
			{ 0x16, MPUREG_I2C_SLV0_DO }, // Register value to 100Hz continuous measurement in 16bit
#else
			{	0x12, MPUREG_I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
			{ 0x81, MPUREG_I2C_SLV0_CTRL }  //Enable I2C and set 1 byte

	};

	for (i = 0; i < MPU_InitRegNum; i++) {
		WriteReg2(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		delay_us(1000); // I2C must slow down the write speed, otherwise it won't work
	}
	//  set_acc_scale(BITS_FS_2G);
	//   set_gyro_scale(BITS_FS_250DPS);

	//   calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	//   return 0;

}

void calibrate(float *dest1, float *dest2) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	WriteReg(MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay_us(100000);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	WriteReg(MPUREG_PWR_MGMT_1, 0x01);
	WriteReg(MPUREG_PWR_MGMT_2, 0x00);
	delay_us(200000);

	// Configure device for bias calculation
	WriteReg(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
	WriteReg(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
	WriteReg(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	WriteReg(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
	WriteReg(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay_us(15000);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	WriteReg(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	WriteReg(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	WriteReg(MPUREG_GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	WriteReg(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	WriteReg(MPUREG_USER_CTRL, 0x40);   // Enable FIFO
	WriteReg(MPUREG_FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay_us(40000); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	WriteReg(MPUREG_FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
	fifo_count = ((uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		ReadRegs(MPUREG_FIFO_R_W, data, 12); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0] += (int32_t) gyro_temp[0];
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0] /= (int32_t) packet_count;
	gyro_bias[1] /= (int32_t) packet_count;
	gyro_bias[2] /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	WriteReg(MPUREG_XG_OFFS_USRH, data[0]);
	WriteReg(MPUREG_XG_OFFS_USRL, data[1]);
	WriteReg(MPUREG_YG_OFFS_USRH, data[2]);
	WriteReg(MPUREG_YG_OFFS_USRL, data[3]);
	WriteReg(MPUREG_ZG_OFFS_USRH, data[4]);
	WriteReg(MPUREG_ZG_OFFS_USRL, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	ReadRegs(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
	ReadRegs(MPUREG_YA_OFFSET_H, data, 2);
	accel_bias_reg[1] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
	ReadRegs(MPUREG_ZA_OFFSET_H, data, 2);
	accel_bias_reg[2] = (int32_t) (((int16_t) data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask))
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
	WriteReg(MPUREG_XA_OFFSET_H, data[0]);
	WriteReg(MPUREG_XA_OFFSET_L, data[1]);
	WriteReg(MPUREG_YA_OFFSET_H, data[2]);
	WriteReg(MPUREG_YA_OFFSET_L, data[3]);
	WriteReg(MPUREG_ZA_OFFSET_H, data[4]);
	WriteReg(MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
	dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
	dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

uint8_t AK8963_whoami() {
	uint8_t response;
	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

	//WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
	delay_us(100);
	response = WriteReg(MPUREG_EXT_SENS_DATA_00 | READ_FLAG, 0x00);   //Read I2C
	//ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
	//response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C

	return response;
}

unsigned int whoami(void) {
	unsigned int response;
	LL_GPIO_ResetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, (MPUREG_WHOAMI | 0x80)); // (Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, 0x00);
	while (!(SPI1->SR & SPI_SR_RXNE))
		; //data received?
	response = LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(MPU_CS_GPIO_PORT, MPU_CS_PIN); // PC4 CS SET Active Low
	return response;
}

unsigned int whoami2(void) {
	unsigned int response;
	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PA4 CS RESET Active Low

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, (MPUREG_WHOAMI | 0x80)); // (Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	LL_SPI_ReceiveData8(SPI2);

	while (!(SPI2->SR & SPI_SR_TXE))
		; //transmit buffer empty?
	LL_SPI_TransmitData8(SPI2, 0x00);
	while (!(SPI2->SR & SPI_SR_RXNE))
		; //data received?
	response = LL_SPI_ReceiveData8(SPI2);

	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN); // PC4 CS SET Active Low
	return response;
}

int Enc1_GetPosition(void) {
	// Greg start comment out
//	int DataPrecision = 12;
//	int tempPosition = 0;
////	int Position = 55;
//	int i = 0;
////	uint8_t XOR = 0;
//	uint8_t tempRead = 0;
//	uint8_t Flags[6];
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, ENC1_CS_PIN);
//
//	// Sensor feeds out position MSB first
//	for(i = DataPrecision-1; i >= 0; i--)
//	{
//		LL_GPIO_ResetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		LL_GPIO_SetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		tempRead = LL_GPIO_IsInputPinSet(ENC1_DATA_GPIO_PORT,ENC1_DATA_PIN) &0x01;
//	//	XOR ^= tempRead;
//		tempPosition |= (tempRead)<<i;
//	}
//
//	for(i = 0; i < 6; i++)
//	{
//		LL_GPIO_ResetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		LL_GPIO_SetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		tempRead = LL_GPIO_IsInputPinSet(ENC1_DATA_GPIO_PORT,ENC1_DATA_PIN) &0x01;
//	//	XOR ^= tempRead;
//		Flags[i] |= (tempRead)<<i;
//	}
//
//	LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, ENC1_CS_PIN);
//
//	// ValidityCheck
////	if(XOR == 0
////		&& Flags[0] == 1
////		&& Flags[1] == 0
////		&& Flags[2] == 0)
////	{
//		// if valid, return flags and update position
////		Position=tempPosition;
////	}
//	return tempPosition;
	// Greg end comment out
}

void Mag_Enc1_Store(void) {
	// Greg start comment out
//	int tempPosition = 0;
//	int i = 0;
//	uint8_t tempRead = 0;
//	uint8_t Flags[6];
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, ENC1_CS_PIN);
//
//	// Sensor feeds out position MSB first
//	for(i = 11; i >= 0; i--)
//	{
//		LL_GPIO_ResetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		LL_GPIO_SetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		tempRead = LL_GPIO_IsInputPinSet(ENC1_DATA_GPIO_PORT,ENC1_DATA_PIN) &0x01;
//		tempPosition |= (tempRead)<<i;
//	}
//
//	for(i = 0; i < 6; i++)
//	{
//		LL_GPIO_ResetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		LL_GPIO_SetOutputPin(ENC1_SCLK_GPIO_PORT, ENC1_SCLK_PIN);
//		delay_us(10);
//
//		tempRead = LL_GPIO_IsInputPinSet(ENC1_DATA_GPIO_PORT,ENC1_DATA_PIN) &0x01;
//		Flags[i] |= (tempRead)<<i;
//	}
//
//	LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, ENC1_CS_PIN);
//
////	BSbuffer[s_flag].Enc1[Sub_cnt]=tempPosition;
	// Greg end comment out
}

void Mag_Enc2_Store(void) {

	int tempPosition = 0;
	int i = 0;
	uint8_t tempRead = 0;
	uint8_t Flags[6];
	LL_GPIO_ResetOutputPin(ENC2_CS_GPIO_PORT, ENC2_CS_PIN);

	// Sensor feeds out position MSB first
	for (i = 11; i >= 0; i--) {
		LL_GPIO_ResetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(10);

		LL_GPIO_SetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(10);

		tempRead = LL_GPIO_IsInputPinSet(ENC2_DATA_GPIO_PORT, ENC2_DATA_PIN)
				& 0x01;
		tempPosition |= (tempRead) << i;
	}

	for (i = 0; i < 6; i++) {
		LL_GPIO_ResetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(10);

		LL_GPIO_SetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(10);

		tempRead = LL_GPIO_IsInputPinSet(ENC2_DATA_GPIO_PORT, ENC2_DATA_PIN)
				& 0x01;
		Flags[i] |= (tempRead) << i;
	}

	LL_GPIO_SetOutputPin(ENC2_CS_GPIO_PORT, ENC2_CS_PIN);

	BSbuffer[s_flag].Enc2[Sub_cnt] = tempPosition;
}

int Enc2_GetPosition(void) {
	int DataPrecision = 12;
	int tempPosition = 0;
	int i = 0;
	uint8_t tempRead = 0;
	uint8_t Flags[6];

	LL_GPIO_ResetOutputPin(ENC2_CS_GPIO_PORT, ENC2_CS_PIN);
	delay_us(1);
	// Sensor feeds out position MSB first
	for (i = DataPrecision - 1; i >= 0; i--) {
		LL_GPIO_ResetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(1);

		LL_GPIO_SetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(1);

		tempRead = LL_GPIO_IsInputPinSet(ENC2_DATA_GPIO_PORT, ENC2_DATA_PIN)
				& 0x01;
		tempPosition |= (tempRead) << i;
	}

	for (i = 0; i < 6; i++) {
		LL_GPIO_ResetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(1);

		LL_GPIO_SetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(1);

		tempRead = LL_GPIO_IsInputPinSet(ENC2_DATA_GPIO_PORT, ENC2_DATA_PIN)
				& 0x01;
		Flags[i] |= (tempRead) << i;
	}

	LL_GPIO_SetOutputPin(ENC2_CS_GPIO_PORT, ENC2_CS_PIN);

	return tempPosition;
}

float knee_angle(void) {
	// Greg start comment
//	float knee_position;
//knee_position= -(Enc2_GetPosition()*0.088)+125; //-(Enc2_GetPosition()*0.088-306); //-((Prosthesis_Data(:,36)*0.088)-306)
// Greg end comment

	// Greg end comment
	float knee_position = (Enc2_GetPosition() - 2263.5) * 0.088;
	// Greg start

	return knee_position;
}

float IMU_orientation(struct imu_data imuMyData, float last_angle, float dt_s)
{
	struct imu_angle gyro, accel, accel_angle, gyro_angle;
	float DEGREES_TO_RADIANS = 3.14159 / 180;

	// User defined constant for complementary filter
	float alpha = 0.002;

	// Convert IMU to gs and rad/sec
	accel.x = (float) imuMyData.AX / 4096;
	accel.y = (float) imuMyData.AY / 4096;
	accel.z = (float) imuMyData.AZ / 4096;
	gyro.x  = (float) imuMyData.GX / 32.8 * DEGREES_TO_RADIANS;
	gyro.y  = (float) imuMyData.GY / 32.8 * DEGREES_TO_RADIANS;
	gyro.z  = (float) imuMyData.GZ / 32.8 * DEGREES_TO_RADIANS;

	// Compute angle from accel
	accel_angle.z   = atan(accel.x / sqrt(pow(accel.y,2) + pow(accel.z,2)));

	// Compute change in angle from gyro (trapezoidal used)
	if (count == 1)
	{
		gyro_angle.z = 0;
		count++;
	}
	else
	{
		gyro_angle.z = dt_s/2 * (gyro.z + last_gyro);
	}

	// Save gyro for next iteration
	last_gyro = gyro.z;

	// Compute angle using complementary filter
	float imu_angle = accel_angle.z*alpha + (1-alpha) * (gyro_angle.z + last_angle);

	// Return angle
	return (imu_angle);
}

