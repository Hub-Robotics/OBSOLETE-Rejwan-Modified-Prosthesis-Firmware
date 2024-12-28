/*
 * knee_control.h
 *
 *  Created on: Dec 18, 2023
 *      Author: brett
 */

#ifndef INCLUDESSRC_KNEE_CONTROL_H_
#define INCLUDESSRC_KNEE_CONTROL_H_

// Declared in main.c, sensor.c, and usbd_storage.c | used in main.c, sensor.c, usbd_storage.c and knee_control.c
extern uint8_t Data_log_Start_Resume;

// Declared in main.c and sensor.c | used in sensor.c and knee_control.c
extern volatile uint32_t Sub_cnt; 		// Storage Buffer Index Needed in LPTIM2 Interrupt
extern volatile uint8_t s_flag; 		// flag 1 for Buffer one, 2 for Buffer two
extern volatile uint8_t w_flag; 		// flag 1 for Buffer one, 2 for Buffer two
extern volatile uint8_t SD_write_Flag; 	// SD Card write Flag
extern volatile uint8_t SD_Write_Count;  // File Write Count

// Declared in main.c and controller.c | structure definition in controller.c | used in knee_control.c
extern struct st_impedance my_st_impedance;

// Declared (maybe?) in core_cm*.h (\Drivers\CMSIS\Include)
extern float T; // interrupt duration


void processKnee();

#endif /* INCLUDESSRC_KNEE_CONTROL_H_ */
