/*
 * MPU9255.h
 *
 *  Created on: Feb 22, 2024
 *      Author: brett
 */

#ifndef DRIVERS_MPU9255_MPU9255_H_
#define DRIVERS_MPU9255_MPU9255_H_

#include "stdint.h"

int mpu9255_write(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char const *data);
int mpu9255_read(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char *data);
void mpu9255_delay_ms(unsigned long num_ms);
void mpu9255_get_ms(unsigned long *count);

/*
 * readPeriod is how often to report the data for the DMP.
 * After init, the driver will read the fifo on this period.
 */
void mpu9255_init(uint32_t readPeriod);
#endif /* DRIVERS_MPU9255_MPU9255_H_ */
