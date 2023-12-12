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
float i_deg=1,i_count=1;
float degree_value=0;
float hip_angle=0;
float KneeError=0, SetAngle=0, Con_d=0,KneeError_old=0,integral=0;
float diff_part=0, angular_velocity_d=0, old_angular_velocity_d=0;
uint16_t volatile phase=0;
uint16_t Loadcell_back=0,Loadcell_back_old=0,Loadcell_front=0,Loadcell_back_filtered_old=0,Loadcell_back_filtered=0;
uint16_t Loadcell_front_filtered_old=0,Loadcell_front_filtered=0,Loadcell_front_old=0;
//float Loadcell_back_filtered_old=0,Loadcell_back_filtered=0;
float IMU_acc=0;
struct imu_data imu_data_now;
struct imu_angle imu_angle_past,imu_angle_now;

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

int Enc1,Enc2,Enc11,Enc22;
float angle_now=0,angle_eq=0;
int torque_calc=0,CST_CMD_EPOS=0;

/**/

struct st_impedance my_st_impedance;

float angle_old = 0;
float angular_velocity=0;
float old_angular_velocity=0;
#define fc 10
float tau= (float) 1/(2*3.14*10);//0.0159;
float T= (float) 1/640; // interrupt duration
float part_1, part_2, part_3,part_4;
unsigned int Sg;
extern volatile uint16_t Test1,Test2,Test1_mV,Test2_mV,Test3,Test4,Test3_mV,Test4_mV,Test5,Test6,Test5_mV,Test6_mV;
extern int16_t bit_data[10],bit_data1[10],bit_data2[10],bit_data3[10];
uint8_t PrintBuf[50],PrintBuf1[50];

// Greg start
uint16_t Loadcell_top_m2       = 0;
uint16_t Loadcell_top_m1       = 0;
float Loadcell_top_filtered_m2 = 0;
float Loadcell_top_filtered_m1 = 0;
// Greg end


int main(void){
	Pros_state =  LP_STOP;                                    // Default state after power ON reset
	SystemClock_Config_MSI_80MHz();	                        // Configure the system clock to 48 MHz from MSI, PLL disabled for power saving

	// If RTC is previously configured and running via backup battery i.e. BackUp Register Data is available, No need to update RTC time again
 if (RTC_BAK_GetRegister(RTC, 0x01) != RTC_BKP_DATE_TIME_UPDTATED) //    0x01 value was arbitrarily set from previous Configure_RTC_Calendar()
	  {
			MX_RTC_Init();   // Set RTC clock source and parameters
	  }

  MX_GPIO_Init();
  HAL_Init();                 // Reset of all peripherals, Initializes the Flash interface and the Systick.



    DFU_Bypass();
    Configure_USART_1();  // Debug with PC
    sprintf(PrintBuf,"Hello");
    USART1_wr_print(PrintBuf,sizeof(PrintBuf));

/*Important information: which IMU interfaced to which SPI */
//IMU1-2 SPI1 .
//IMU3   SPI2
//IMU4-5 SPI3

/*Sensor Initialization starts here */

// P_IMU4_SPI3_Initialization_at_reset();   //IMU4-5_SPI3 //step1
 P_IMU1_SPI1_Initialization_at_reset(); //IMU1-2__SPI1  (only IMU1 configured)
 P_ADC_Sensor_GPIO_Init(); //ADC GPIOs //here we initialized the chip select pins as well

/*CAN Bus SPI Initialization*/
 MCP_SPI2_Initialization_at_reset();






  //Configures Bit timing currently set to 125kbits/s
  CAN_configure();

  //Sets can mode currently set to normal mode
  CAN_mode();

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
int jj;
for(jj = 1; jj<1000; ++jj)
{
		angle_now=knee_angle();
//					phase=1;
		/* velocity calculation*/
		angular_velocity = (float) (2*(angle_now - angle_old)+(2*tau-T)*old_angular_velocity) /(T+2*tau);
		old_angular_velocity = (float) angular_velocity;
		angle_old = (float) angle_now;

}


imu_angle_past.x=0;
imu_angle_past.y=0;


//while(1)
//{
//	}


//*
 // CAN chip disable
// while(1)
// {RED_LED_ONLY();
// LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);
// }
 //*/



//  velocity mode
//  EPOS4_PVM_set_velocity(CAN_ID, -50);
//  EPOS4_PVM_start(CAN_ID);

//while(1)
//{
//	  EPOS4_PVM_set_velocity(CAN_ID, 250);
//	  EPOS4_PVM_start(CAN_ID);
//	  delay_us(500000);
//
//	  EPOS4_PVM_set_velocity(CAN_ID, -250);
//	  EPOS4_PVM_start(CAN_ID);
//	  delay_us(500000);
//
//}


//  while(1)
//  {
////	  EPOS4_enable(CAN_ID);
//  EPOS4_CST_apply_torque(CAN_ID,200); //100 means 10% = 0.8A current; 1000=100%=8A
//  delay_us(250000);
//  EPOS4_CST_stop(CAN_ID);
//  delay_us(1000000);
//
//  EPOS4_CST_apply_torque(CAN_ID,-100);
//  delay_us(250000);
//  EPOS4_CST_stop(CAN_ID);
//  delay_us(1000000);
//
//  }


////LED blinking
//  while(1)
//  {
//
//  RED_LED_ONLY();
//  delay_us(1000000);
//  GREEN_LED_ONLY();
//  delay_us(1000000);
//  }


  //Angle testing
//    while(1)
//    {
//    	angle_now=knee_angle();
//
//    		  	  if(angle_now>=10 && angle_now<=30 )
//    		  	  {	GREEN_LED_ONLY();
//
//    		  	  }
//    		  	  else
//    		  	  {
//    		  		RED_LED_ONLY();
//    		  	  }
//    }

// New board led test

//while(1)
//{
//
//	RED_LED();
//	GREEN_LED();
//	YELLOW_LED();
//	delay_us(500000);
//	RED_LED_OFF();
//	GREEN_LED_OFF();
//	YELLOW_LED_OFF();
//	delay_us(1000000);
//}



//while(1)
//	  {
//	LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_11); // RED LED
//	LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_10); // GREEN LED
//	LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_8);  // YELLOW LED
//
//	delay_us(500000);
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_11);
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_10);
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_8);
//
//	delay_us(500000);
//
//
//	  }


//while(1){
//	LL_GPIO_TogglePin(ENC1_CS_GPIO_PORT,ENC1_CS_PIN);
//
//						angle_eq=30;
//						Enc2_GetPosition();
////						angle_now=knee_angle();
//
//						/* velocity calculation*/
//						angular_velocity = (float) (2*(angle_now - angle_old)+(2*tau-T)*old_angular_velocity) /(T+2*tau);
//						old_angular_velocity = (float) angular_velocity;
//						angle_old = (float) angle_now;
//
//
////	EPOS4_CST_apply_torque(CAN_ID,100);
////angle_now=knee_angle();
//}




  USB_PA9_EXTI_conf();         // USB connectivity pin detect Interrupt // Data_Pause_Resume_PC0_EXTI_conf();

  Configure_LPTIM2_Int(); // Configured LPTIM2 but not started. To be started before going to Loop
  Configure_Interrupt();       // Re-arrange NVIC interrupt priority

  Power_on_reset();            // Following reset is found by troubleshooting





#ifdef AIM_Start_Data_Collection_on_Reset
  AIM_DataStart_at_Reset();
  GREEN_LED_ONLY();


#else
  Start_LPTIMCounter2(0x1F);           // Start 1ms LPTIM interrupt for
  SD_cnt_limit=100;                    // After 100 write, File will Sync
  Prepare_Goto_Dormant_Mode();
  Pros_state = Dormant_Idle_Stop;
#endif



  // USB Default mode is USB VCP
  // Note: Data collection is stopped in Power on Reset. Send the command from PC LabVIEW software in USB VCP Mode to start data collection.
  // Data collection will resume after USB disconnect.
  // Only way to stop Data collection is by accessing SD card from PC LabVIEW program



    while(1) {

				  switch(Pros_state) {
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

						if (USB_Present_ == 1)                     // After detecting USB attachment via EXTI_5 int
						 {
						 Pros_state = USB_MSC_VCP_Mode;               // Next State USB_MSC_Mode Mode
						 USB_Present_ = 0;                           // Reset Flag to avoid looping
						  }
						 else if (FATAL_Error == 1)                      // Triggered from Error_Handler
							{
								FATAL_Error = 0;                            // Reset Flag to avoid looping
								 Pros_state = Fatal_Error_State;             // Next State Fatal_Error_State
							 }
						 else if (SD_write_Flag == 1)                    // When BUffer full to store in SD card
						  {
							Pros_state = Sensor_FATFS_Write;              // Next State Sensor_FATFS_Write Mode
							SD_write_Flag = 0;                           // Reset Flag to avoid looping
						  }

						 else
						 {
							 Pros_state = LP_STOP;                       // Return to Low Power Mode Data collection
						 }
						break;

					case Sensor_FATFS_Write:


						Battery_ADC_Value=2500;
	                    if ( Battery_ADC_Value < Battery_3200mV)                            // Battery Cut off Value 3.0V
	   								{
	   							         Pros_state = Low_Battery_Mode;            // Prepare to enter Low_Battery_Mode mode
	   								}
	   								 else
	   								 {
	   									SD_Sensor_write();                   // Write Sensor Buffer to SD card
		                                     if (USB_Present_ == 1)          // If USB attachment detected here via EXTI_5 int
												 {
													 Pros_state = USB_MSC_VCP_Mode;                   // Next State USB_MSC_Mode Mode
													 USB_Present_ = 0;                           // Reset Flag to avoid looping
												 }
		            						 else if (FATAL_Error == 1)                      // Triggered from Error_Handler
		            							{
		            								FATAL_Error = 0;                            // Reset Flag to avoid looping
		            								 Pros_state = Fatal_Error_State;             // Next State Fatal_Error_State
		            							 }
												 else
												 {
													 Pros_state = LP_STOP;                        // Return to Low Power Mode Data collection
												 }
	   								 }
						break;

					case Low_Battery_Mode:
							NVIC_DisableIRQ(LPTIM2_IRQn); // Disabling LPTIM2 Timer Interrupt to stop Data collection
							ALL_LED_OFF();
							Shut_Down_SD();

							Reset_Variables_for_LowBattery();
							Prepare_Goto_Dormant_Mode();
						 Pros_state = Dormant_Idle_Stop;                       // Idle Lowest Power STOP Mode

						break;

					case Fatal_Error_State:
						   RED_LED_ONLY();

						   FATFS_Logstart_Delete();
						   Data_log_Start_Resume=0;

						   SD_write_Flag = 0;
						   NVIC_DisableIRQ(LPTIM2_IRQn);


						   LL_LPTIM_DisableIT_ARRM(LPTIM2);
						   SD_POWER_OFF();
						   LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC);

	                        EnterStop();   // Enter Stop Mode
						 // Wake Up after STOP Mode only by USB EXTI Int

						if (USB_Present_ == 1)                     // After detecting USB attachment via EXTI_5 int
						 {
						 Pros_state = USB_MSC_VCP_Mode;               // Next State USB_MSC_Mode Mode
						 USB_Present_ = 0;                           // Reset Flag to avoid looping
						 SD_POWER_ON();               				// Power on SD CARD
						 }
						 else
						 {
							 Pros_state = Dormant_Idle_Stop;                       // Stay Dormant Mode if no external events
						 }
						break;

					case USB_MSC_VCP_Mode:

						USB_Init_Start();            // Initialize USB and Stay USB mode as long as USB cable connected

  					    break;

					case Dormant_Idle_Stop:             // Idle Lowest Power Stop Mode with no Data log (only wait for USB connectivity)

					  	                        EnterStop();   // Enter Stop Mode
												 // Wake Up after STOP Mode only by USB EXTI Int

												if (USB_Present_ == 1)                     // After detecting USB attachment via EXTI_5 int
												 {
												 Pros_state = USB_MSC_VCP_Mode;               // Next State USB_MSC_Mode Mode
												 USB_Present_ = 0;                           // Reset Flag to avoid looping
												 SD_POWER_ON();               				// Power on SD CARD
												 }
												 else
												 {
													 Pros_state = Dormant_Idle_Stop;                       // Stay Dormant Mode if no external events
												 }
						break;


					default:
				            break;

				  }
				}

}

void LPTIM2_IRQHandler(void)   // Response of 10ms LPTIM interrupt
{

	// Greg start
	// Measure interrupt speed start
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
	// Greg end

	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2) == 1)	// auto reload match interrupt has occured
	{



		if (Data_log_Start_Resume == 1)   // Sensor Start Flag from VCP command / FATFS SD card file
		{

//					angle_eq=30;

					angle_now=knee_angle();
//					phase=1;
					/* velocity calculation*/
					angular_velocity = (float) (2*(angle_now - angle_old)+(2*tau-T)*old_angular_velocity) /(T+2*tau);
					old_angular_velocity = (float) angular_velocity;
					angle_old = (float) angle_now;

					Loadcell_front=Read_Loadcell1();
					Loadcell_back=Read_Loadcell2();

					/*Filter*/
					Loadcell_back_filtered=coeff_y*Loadcell_back_filtered_old+coeff_x*Loadcell_back+coeff_x*Loadcell_back_old;
					Loadcell_back_filtered_old=Loadcell_back_filtered;
					Loadcell_back_old=Loadcell_back;

					// Greg start comment
//					Loadcell_front_filtered=coeff_y*Loadcell_front_filtered_old+coeff_x*Loadcell_front+coeff_x*Loadcell_front_old;
//					Loadcell_front_filtered_old=Loadcell_front_filtered;
//					Loadcell_front_old=Loadcell_front;
					// Greg start end comment

					// Greg start
					uint16_t Loadcell_top       = Loadcell_front;
					float Loadcell_top_filtered = 1.6556*Loadcell_top_filtered_m1 - 0.7068*Loadcell_top_filtered_m2 + 0.0128*Loadcell_top + 0.0256*Loadcell_top_m1 + 0.0128*Loadcell_top_m2;
					Loadcell_top_m2             = Loadcell_top_m1;
					Loadcell_top_m1             = Loadcell_top;
					Loadcell_top_filtered_m2    = Loadcell_top_filtered_m1;
					Loadcell_top_filtered_m1    = Loadcell_top_filtered;
					// Greg end

					imu_data_now=IMU1_read();
					imu_angle_now=IMU_orientation(imu_data_now, imu_angle_past, 0.002);
					imu_angle_past=imu_angle_now;

					// Greg start comment
//					IMU_acc=imu_data_now.AZ;
					// Greg end comment

					// Greg start
					IMU_acc = -imu_data_now.AY;
					// Greg end

					// Greg start comment
					hip_angle=(imu_angle_now.x*rad2deg)-angle_now;
					// Greg end comment

					// Greg Starts
					// Compute hip angle using Madgwick filter (input args are rotated 90 deg in X)
//					hip_angle = -MadgwickAHRSupdateIMU(imu_data_now.GX/32.8*(3.1416/180),-imu_data_now.GZ/32.8*(3.1416/180),imu_data_now.GY/32.8*(3.1416/180),imu_data_now.AX,-imu_data_now.AZ,imu_data_now.AY);
//					hip_angle = hip_angle*180/3.1416 - angle_now;
					// Greg Ends

							// Greg start comment out
//						  	  if(angle_now>=-5 && angle_now<=70)
//						  	  {	GREEN_LED_ONLY();
//
//
//						  	  	  GREEN_LED();
						    // Greg end comment out

							  	  my_st_impedance=controller_impedance(angle_now, angular_velocity,Loadcell_back_filtered,Loadcell_front_filtered,IMU_acc,hip_angle);
//    					  	  phase=2;//my_st_impedance.st;

//							  	SetAngle=(float) 25+15*sin(i_deg*3.14/180/2);//(float) (25+10*sin((float) i*3.1416/180/2));
//
//									if(i_deg==360*2)
//									{
//										i_deg=0;
//									}
//									else
//									{
//										i_deg=i_deg+1;}
//
//
//
//								/*PID controller*/
//								KneeError=SetAngle-angle_now;
//
//								//derivative part of the controller
//								Con_d=(KneeError-KneeError_old)/512;
//								KneeError_old=KneeError;
//
//
//								integral=integral+KneeError;
//
//
//								 /*Practical diff*/
//
//								CST_CMD_EPOS= -(kp*KneeError+kI*integral-kd*angular_velocity);//PID controller (Kp*KneeError+Ki*integral+Kd*Con_d);
//
//
//						  		if (CST_CMD_EPOS>=max_CST_CMD_EPOS)
//						  		{
//						  			CST_CMD_EPOS=max_CST_CMD_EPOS;
//						  		}
//
//						  		else if (CST_CMD_EPOS<-max_CST_CMD_EPOS)
//						  		{
//						  			CST_CMD_EPOS=-max_CST_CMD_EPOS;
//						  		}
//						  		else
//						  			CST_CMD_EPOS=CST_CMD_EPOS;
//						  		EPOS4_CST_apply_torque(CAN_ID,CST_CMD_EPOS); //100 means 10% +ve is extension -ve is flextion

							  	// Greg start comment out
//						  	  }
//						  	  else
//						  	  {GREEN_LED_OFF();
//						  		RED_LED_ONLY();
//						  		CST_CMD_EPOS=0;
//						  		EPOS4_CST_apply_torque(CAN_ID,0); //100 means 10%
//						  	  }
							  	  // Greg end comment out

//			LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_10);

			// Greg start comment out
//			LL_GPIO_TogglePin(ENC1_CS_GPIO_PORT,ENC1_CS_PIN);
			// Greg end comment out

			F_Sensor_ADC_Store();
			Mag_Enc2_Store();
//			MPU_9D_store_IMU1_SPI1();

//			Knee_data_storeIMU(Loadcell_front_filtered,Loadcell_back_filtered,IMU_acc,1,1,1);

			// Greg Start Comment Out
//			Knee_data_storeIMU(100*hip_angle,1,IMU_acc,1,1,1); //previous 1-6: hip_angle=1, IMU_acc=3
			// Greg End Comment Out

			// Greg Start
			Knee_data_storeIMU(imu_data_now.AX,imu_data_now.AY,imu_data_now.AZ,imu_data_now.GX,imu_data_now.GY,imu_data_now.GZ);
			// Greg End

//			Knee_data_storeIMU(1,1,1,1,1,1);




//			Knee_data_store(100*angle_now,100*SetAngle,100*angular_velocity,my_st_impedance.CST_CMD_now,1,1,1,my_st_impedance.st);
//			Knee_data_store1(100*my_st_impedance.desired_torque,Loadcell_front,Loadcell_back,IMU_acc);
//			Knee_data_store(100*angle_now,angular_velocity,my_st_impedance.CST_CMD_now,my_st_impedance.st,1,1,1,1);

			// Greg Start Comment Out
//			Knee_data_store(100*angle_now,angular_velocity,my_st_impedance.CST_CMD_now,my_st_impedance.st);  //previous 7-10,
			// Greg End Comment Out

			// Greg Start
			Knee_data_store(angle_now/0.088, hip_angle, Loadcell_front, Loadcell_back);
			// Greg End

			// Greg start comment out
//			Knee_data_store1(100*my_st_impedance.desired_torque,Loadcell_front_filtered,Loadcell_back_filtered,IMU_acc);  //previous 11-14,
			// Greg end comment out

			// Greg start
			Knee_data_store1(Loadcell_top_filtered,12,13,14);
			// Greg end

//			Knee_data_store1(1,1,1,1);



			//			Knee_data_store1(100*hip_angle,100*my_st_impedance.desired_torque,1,1); //imu_angle_now.x


//			Knee_data_store1(100*hip_angle,100*my_st_impedance.desired_torque);
//			Knee_data_store2(1,1);

			if (Sub_cnt == 5)
			{
				//BSbuffer[s_flag].Blank1 = (uint8_t) (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_10));
				BSbuffer[s_flag].RTC_Time = (uint32_t)(RTC->TR & 0x007F7F7F);
				BSbuffer[s_flag].RTC_Date= (uint32_t)(RTC->DR & 0x00FFFF3F);

			}

// Switching Buffer code starts here
			if(Sub_cnt==Highest_sensor_count)   // Total samples to be stored in a 16KB buffer
			{
				Sub_cnt=0;           // Reset Counter of sensor element
				SD_write_Flag = 1;   // Flag set to write filled buffer content
				// Changing Buffer
				if (s_flag==0)       // if current storgae_buffer was 0
				{
					w_flag=0;            // write_buffer to be saved in SD card = 0
					s_flag=1;            // current storgae_buffer is set 1
				}
				else                    // if current storgae_buffer was 1
				{
					w_flag=1;           // write_buffer to be saved in SD card = 1
					s_flag=0;           // current storgae_buffer is set 0
				}

			}
			else
			{
				Sub_cnt++;              // Increment Counter of sensor element
			}                    /*Switching Buffer code Ends here */

		}  // Data log at timer interrupt ends here

		LL_LPTIM_ClearFLAG_ARRM(LPTIM2); // Clear ARR interrupt flag
	}

	// Greg start
	// Measure interrupt speed end
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);
	// Greg end

}

void EXTI9_5_IRQHandler(void)       // Interrupt from USB connectivity PIN PA9
{
if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
{
USB_Present_=1;         // USB present
LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);   // Clear interrupt
}
}


//int PID_position_controller(float set_position,float kp, float ki, float kd)
//{
//
////	angle = GetKneeAngle();
//
//	angle_now=knee_angle();
//
//
//	/* velocity calculation*/
//
//	angular_velocity = (float) ((float) 2*(angle_now - angle_old)+(float) (2*tau-T)*old_angular_velocity) /(float) (T+2*tau);
//	old_angular_velocity = (float) angular_velocity;
//	angle_old = (float) angle_now;
//
//
//
//
//
//
//	        KneeError=SetAngle-angle_now;
//
//	        //derivative part of the controller
//	        Con_d=(KneeError-KneeError_old)*500;
//	        KneeError_old=KneeError;
//
//	        integral=integral+KneeError*0.002;
//
//	        //anti-windup
//	        if(fabs(Ki*integral)>integral_max){
//	            integral=integral*integral_max/fabs(integral);
//	        }
//
//	        PWMPercent=(Kp*KneeError+Ki*integral+Kd*Con_d);//PID controller
//
//}


/************************ (C) COPYRIGHT CLAWS UA ***************/
