/*
 * knee_control.h
 *
 *  Created on: Dec 18, 2023
 *      Author: brett
 */

#ifndef INCLUDESSRC_KNEE_CONTROL_H_
#define INCLUDESSRC_KNEE_CONTROL_H_


extern float i_deg;
extern float degree_value;
extern float hip_angle;
extern float KneeError, SetAngle, Con_d,KneeError_old,integral;
extern float diff_part, angular_velocity_d, old_angular_velocity_d;
extern uint16_t volatile phase;



extern volatile uint32_t Sub_cnt; 		// Storage Buffer Index Needed in LPTIM2 Interrupt
extern volatile uint8_t s_flag; 		// flag 1 for Buffer one, 2 for Buffer two
extern volatile uint8_t w_flag; 		// flag 1 for Buffer one, 2 for Buffer two
extern volatile uint8_t SD_write_Flag; 	// SD Card write Flag
extern volatile uint8_t SD_Write_Count;  // File Write Count

extern int Enc1,Enc2,Enc11,Enc22;
extern float angle_now,angle_eq;
extern int torque_calc,CST_CMD_EPOS;

extern struct st_impedance my_st_impedance;

extern float angle_old;
extern float angular_velocity;
extern float old_angular_velocity;

extern float tau;//0.0159;
extern float T; // interrupt duration
extern float part_1, part_2, part_3,part_4;
extern unsigned int Sg;

extern uint8_t SD_cnt_limit, Data_log_Start_Resume,ADC_Busy_with_Battery_Monitor,ErrorCodeBuf;
extern uint16_t Loadcell_back,Loadcell_back_old,Loadcell_front,Loadcell_back_filtered_old,Loadcell_back_filtered;
extern uint16_t Loadcell_front_filtered_old,Loadcell_front_filtered,Loadcell_front_old;

extern float IMU_acc;
extern struct imu_data imu_data_now;
extern struct imu_angle imu_angle_past,imu_angle_now;

void processKnee();

#endif /* INCLUDESSRC_KNEE_CONTROL_H_ */
