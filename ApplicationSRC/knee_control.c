/*
 * knee_control.c
 *
 *  Created on: Dec 18, 2023
 *      Author: brett
 */

#include "main.h"
#include "CAN.h"
#include "sensor.h"
#include "EPOS4.h"
#include "controller.h"
#include "knee_control.h"

extern int CAN_ID;

#define torque_const_kv100lite 0.095
#define gear_ratio_Chain_knee 40
#define peak_current 19 // Amp set in epos
#define nominal_current 8 // Amp
#define rad2deg 180/3.14
#define kp 65//65   //40
#define kd .68  //.68
#define kI 0
#define set_position 25

#define coeff_x 0.0689
#define coeff_y 0.8621



uint16_t Loadcell_top_m2;
uint16_t Loadcell_top_m1 ;
float Loadcell_top_filtered_m2;
float Loadcell_top_filtered_m1;

void processKnee() {
	if (Data_log_Start_Resume == 1) // Sensor Start Flag from VCP command / FATFS SD card file
			{

//					angle_eq=30;

		angle_now = knee_angle();
//					phase=1;
		/* velocity calculation*/
		angular_velocity = (float) (2 * (angle_now - angle_old)
				+ (2 * tau - T) * old_angular_velocity) / (T + 2 * tau);
		old_angular_velocity = (float) angular_velocity;
		angle_old = (float) angle_now;

		Loadcell_front = Read_Loadcell1();
		Loadcell_back = Read_Loadcell2();

		/*Filter*/
		Loadcell_back_filtered = coeff_y * Loadcell_back_filtered_old
				+ coeff_x * Loadcell_back + coeff_x * Loadcell_back_old;
		Loadcell_back_filtered_old = Loadcell_back_filtered;
		Loadcell_back_old = Loadcell_back;

		// Greg start comment
//					Loadcell_front_filtered=coeff_y*Loadcell_front_filtered_old+coeff_x*Loadcell_front+coeff_x*Loadcell_front_old;
//					Loadcell_front_filtered_old=Loadcell_front_filtered;
//					Loadcell_front_old=Loadcell_front;
		// Greg start end comment

		// Greg start
		uint16_t Loadcell_top = Loadcell_front;
		float Loadcell_top_filtered = 1.6556 * Loadcell_top_filtered_m1
				- 0.7068 * Loadcell_top_filtered_m2 + 0.0128 * Loadcell_top
				+ 0.0256 * Loadcell_top_m1 + 0.0128 * Loadcell_top_m2;
		Loadcell_top_m2 = Loadcell_top_m1;
		Loadcell_top_m1 = Loadcell_top;
		Loadcell_top_filtered_m2 = Loadcell_top_filtered_m1;
		Loadcell_top_filtered_m1 = Loadcell_top_filtered;
		// Greg end

		imu_data_now = IMU1_read();
		imu_angle_now = IMU_orientation(imu_data_now, imu_angle_past, 0.002);
		imu_angle_past = imu_angle_now;

		// Greg start comment
//					IMU_acc=imu_data_now.AZ;
		// Greg end comment

		// Greg start
		IMU_acc = -imu_data_now.AY;
		// Greg end

		// Greg start comment
		hip_angle = (imu_angle_now.x * rad2deg) - angle_now;
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

		my_st_impedance = controller_impedance(angle_now, angular_velocity,
				Loadcell_back_filtered, Loadcell_front_filtered, IMU_acc,
				hip_angle);
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
		Knee_data_storeIMU(imu_data_now.AX, imu_data_now.AY, imu_data_now.AZ,
				imu_data_now.GX, imu_data_now.GY, imu_data_now.GZ);
		// Greg End

//			Knee_data_storeIMU(1,1,1,1,1,1);

//			Knee_data_store(100*angle_now,100*SetAngle,100*angular_velocity,my_st_impedance.CST_CMD_now,1,1,1,my_st_impedance.st);
//			Knee_data_store1(100*my_st_impedance.desired_torque,Loadcell_front,Loadcell_back,IMU_acc);
//			Knee_data_store(100*angle_now,angular_velocity,my_st_impedance.CST_CMD_now,my_st_impedance.st,1,1,1,1);

		// Greg Start Comment Out
//			Knee_data_store(100*angle_now,angular_velocity,my_st_impedance.CST_CMD_now,my_st_impedance.st);  //previous 7-10,
		// Greg End Comment Out

		// Greg Start
		Knee_data_store(angle_now / 0.088, hip_angle, Loadcell_front,
				Loadcell_back);
		// Greg End

		// Greg start comment out
//			Knee_data_store1(100*my_st_impedance.desired_torque,Loadcell_front_filtered,Loadcell_back_filtered,IMU_acc);  //previous 11-14,
		// Greg end comment out

		// Greg start
		Knee_data_store1(Loadcell_top_filtered, 12, 13, 14);
		// Greg end

//			Knee_data_store1(1,1,1,1);

		//			Knee_data_store1(100*hip_angle,100*my_st_impedance.desired_torque,1,1); //imu_angle_now.x

//			Knee_data_store1(100*hip_angle,100*my_st_impedance.desired_torque);
//			Knee_data_store2(1,1);

		if (Sub_cnt == 5) {
			//BSbuffer[s_flag].Blank1 = (uint8_t) (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_10));
			BSbuffer[s_flag].RTC_Time = (uint32_t) (RTC->TR & 0x007F7F7F);
			BSbuffer[s_flag].RTC_Date = (uint32_t) (RTC->DR & 0x00FFFF3F);

		}

// Switching Buffer code starts here
		if (Sub_cnt == Highest_sensor_count) // Total samples to be stored in a 16KB buffer
				{
			Sub_cnt = 0;           // Reset Counter of sensor element
			SD_write_Flag = 1;   // Flag set to write filled buffer content
			// Changing Buffer
			if (s_flag == 0)       // if current storgae_buffer was 0
					{
				w_flag = 0;           // write_buffer to be saved in SD card = 0
				s_flag = 1;            // current storgae_buffer is set 1
			} else                    // if current storgae_buffer was 1
			{
				w_flag = 1;           // write_buffer to be saved in SD card = 1
				s_flag = 0;           // current storgae_buffer is set 0
			}

		} else {
			Sub_cnt++;              // Increment Counter of sensor element
		} /*Switching Buffer code Ends here */

	}  // Data log at timer interrupt ends here

}
