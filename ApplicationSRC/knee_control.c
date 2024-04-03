/*
 * knee_control.c
 *
 *  Created on: Dec 18, 2023
 *      Author: brett
 */

#include "main.h"
//#include "CAN.h"
#include "sensor.h"
#include "EPOS4.h"
#include "controller.h"
#include "knee_control.h"

// Greg start
#include "gpio_functions.h"
#include "mpu9255.h"
#include "math.h"
// Greg end

extern int CAN_ID;

#define torque_const_kv100lite 0.095
#define gear_ratio_Chain_knee 40
#define peak_current 19 // Amp set in epos
#define nominal_current 8 // Amp
#define rad2deg 180/3.1416
#define kp 65//65   //40
#define kd .68  //.68
#define kI 0
#define set_position 25
#define coeff_x 0.0689
#define coeff_y 0.8621

// Greg start
uint8_t count = 1;
uint16_t Loadcell_top_m2;			// m2 = minus 2, used in 2nd order filter
uint16_t Loadcell_top_m1;			// m1 = minus 1, used in 2nd order filter
uint16_t Loadcell_bot_m2;			// m2 = minus 2, used in 2nd order filter
uint16_t Loadcell_bot_m1;			// m1 = minus 1, used in 2nd order filter
float Loadcell_top_filtered;
float Loadcell_top_filtered_m2;		// m2 = minus 2, used in 2nd order filter
float Loadcell_top_filtered_m1;		// m1 = minus 1, used in 2nd order filter
float Loadcell_bot_filtered;
float Loadcell_bot_filtered_m2;		// m2 = minus 2, used in 2nd order filter
float Loadcell_bot_filtered_m1;		// m1 = minus 1, used in 2nd order filter
float const dt_s = 1.0/512;

float hip_angle = 0.0;
float angle_old = 0.0;
float angular_velocity = 0.0;
float old_angular_velocity = 0.0;
float IMU_acc = 0.0;
struct imu_data imu_data_now;
float imu_angle_now;
float imu_angle_past;

float tau              = 1/(2*3.1416*10);
float Ts               = 1/512.0;

struct imu_data AccelCal (struct imu_data imu_data_now);
// Greg end


void processKnee () {
	if (Data_log_Start_Resume == 1) // Sensor Start Flag from VCP command / FATFS SD card file
	{
		// Get knee angle in degrees
		float angle_now = knee_angle();

		// Initialize knee angle with first knee angle (used in hip  angle    computation)
		// Initialize IMU  angle with first knee angle (used in knee velocity computation)
		if (count == 1)
		{
			angle_old      = angle_now;				// Units in degrees
			imu_angle_past = angle_now / (rad2deg);	// Units in radians
		}

		// Compute knee velocity with practical differentiator (fc = 10 Hz, bilinear transformation used)
		// Units in degrees per second
		angular_velocity = (2*(angle_now - angle_old)+(2*tau-Ts)*angular_velocity) / (Ts+2*tau);
		angle_old        = angle_now;

		// Filter load cell with 2nd order low-pass Butterworth (fc = 20 Hz)
		// Units in ADC values
		uint16_t Loadcell_top = Read_Loadcell1();
		uint16_t Loadcell_bot = Read_Loadcell2();
		if (count == 1)
		{
			Loadcell_top_filtered    = Loadcell_top;
			Loadcell_top_m2          = Loadcell_top;
			Loadcell_top_filtered_m2 = Loadcell_top;

			Loadcell_bot_filtered    = Loadcell_bot;
			Loadcell_bot_m2          = Loadcell_bot;
			Loadcell_bot_filtered_m2 = Loadcell_bot;

			count++;
		}
		else if (count == 2)
		{
			Loadcell_top_filtered    = Loadcell_top;
			Loadcell_top_m1          = Loadcell_top;
			Loadcell_top_filtered_m1 = Loadcell_top;

			Loadcell_bot_filtered    = Loadcell_bot;
			Loadcell_bot_m1          = Loadcell_bot;
			Loadcell_bot_filtered_m1 = Loadcell_bot;

			count++;
		}
		else
		{
			Loadcell_top_filtered    =    1.6556f * Loadcell_top_filtered_m1
										- 0.7068 * Loadcell_top_filtered_m2 + 0.0128 * Loadcell_top
										+ 0.0256 * Loadcell_top_m1 + 0.0128 * Loadcell_top_m2;
			Loadcell_top_m2          = Loadcell_top_m1;
			Loadcell_top_m1          = Loadcell_top;
			Loadcell_top_filtered_m2 = Loadcell_top_filtered_m1;
			Loadcell_top_filtered_m1 = Loadcell_top_filtered;

			Loadcell_bot_filtered    =    1.6556 * Loadcell_bot_filtered_m1
										- 0.7068 * Loadcell_bot_filtered_m2 + 0.0128 * Loadcell_bot
										+ 0.0256 * Loadcell_bot_m1 + 0.0128 * Loadcell_bot_m2;
			Loadcell_bot_m2          = Loadcell_bot_m1;
			Loadcell_bot_m1          = Loadcell_bot;
			Loadcell_bot_filtered_m2 = Loadcell_bot_filtered_m1;
			Loadcell_bot_filtered_m1 = Loadcell_bot_filtered;
		}

		// Get raw IMU data in ADC values
		imu_data_now = IMU1_read();

		// Calibrate accelerometer in ADC values
		imu_data_now = AccelCal(imu_data_now);

		// Remove offsets from gyro in ADC values
		imu_data_now.GX -= -60;
		imu_data_now.GY -= -29;
		imu_data_now.GZ -=  16;

		// Get accel data for heel strike if needed in ADC values
		IMU_acc = -imu_data_now.AY;

		// Compute hip angle using complementary filter
		imu_angle_now  = IMU_orientation(imu_data_now, imu_angle_past, dt_s);	// Units in radians
		imu_angle_past = imu_angle_now;											// Units in radians
		hip_angle      = imu_angle_now*rad2deg - angle_now;						// Units in degrees

		// Measure speed with oscope start
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);

		// Command motor
		my_st_impedance = controller_impedance(angle_now, angular_velocity, Loadcell_bot_filtered, Loadcell_top_filtered, IMU_acc, hip_angle);

		// Measure speed with oscope end
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);

		// CAN WE DELETE THIS ??? CAUSES DIFFERENT GYRO READINGS FOR SOME REASON
//		F_Sensor_ADC_Store();
//		Mag_Enc2_Store();

		// Store data
		// Data can only be stored as integers, so some scaling is done to help resolution
		Knee_data_storeIMU(imu_data_now.AX, imu_data_now.AY, imu_data_now.AZ, imu_data_now.GX, imu_data_now.GY, imu_data_now.GZ);
		Knee_data_store(angle_now * 100, hip_angle * 100, Loadcell_top, Loadcell_top_filtered);
		Knee_data_store1(Loadcell_bot, Loadcell_bot_filtered, my_st_impedance.desired_torque * 100, angular_velocity * 100);
//        Knee_data_store2(15,16);    <-- this is commented out in sensor.c

		if (Sub_cnt == 5) {
			//BSbuffer[s_flag].Blank1 = (uint8_t) (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_10));
			BSbuffer[s_flag].RTC_Time = (uint32_t) (RTC->TR & 0x007F7F7F);
			BSbuffer[s_flag].RTC_Date = (uint32_t) (RTC->DR & 0x00FFFF3F);

		}

		// Switching Buffer
		if (Sub_cnt == Highest_sensor_count)    // Total samples to be stored in a 16KB buffer
		{
			Sub_cnt = 0;          // Reset Counter of sensor element
			SD_write_Flag = 1;    // Flag set to write filled buffer content
			// Changing Buffer
			if (s_flag == 0)    // if current storgae_buffer was 0
			{
				w_flag = 0;    // write_buffer to be saved in SD card = 0
				s_flag = 1;    // current storgae_buffer is set 1
			}
			else               // if current storgae_buffer was 1
			{
				w_flag = 1;    // write_buffer to be saved in SD card = 1
				s_flag = 0;    // current storgae_buffer is set 0
			}
		}
		else
		{
			Sub_cnt++;    // Increment Counter of sensor element
		}
	}
}

struct imu_data AccelCal (struct imu_data imu_data_now)
{
	// Separate data in ADC values
	float ax = imu_data_now.AX;
	float ay = imu_data_now.AY;
	float az = imu_data_now.AZ;

	// Accelerometer bias in ADV values
	float bx = 0.0198038816126119 * 4096;
	float by = 0.0223486211295295 * 4096;
	float bz = 0.0296143940007433 * 4096;

	// Sine/Cosine of Euler angles (1 = alpha, 2 = beta, 3 = gamma)
	float c1 = cos(-0.00991457347766541);
	float c2 = cos(-0.00385752322363421);
	float c3 = cos(-0.009920754204949  );
	float s1 = sin(-0.00991457347766541);
	float s2 = sin(-0.00385752322363421);
	float s3 = sin(-0.009920754204949  );

	// Scaling factor (helps with normalization)
	float n = 1.00509896445316;

	// Accelerometer calibration in ADC values
	imu_data_now.AX = n * ( ax*(c1*c3 - c2*s1*s3) + ay*(  -c3*s1 - c1*c2*s3) + az*( s2*s3) ) - bx;
	imu_data_now.AY = n * ( ax*(c1*s3 + c2*c3*s1) + ay*(c1*c2*c3 - s1*s3   ) + az*(-c3*s2) ) - by;
	imu_data_now.AZ = n * ( ax*(        s1*s2   ) + ay*(           c1*s2   ) + az*( c2   ) ) - bz;

	// Return in ADC values
	return imu_data_now;
}
