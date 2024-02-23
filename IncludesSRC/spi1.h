/*
 * spi1.h
 *
 *  Created on: Feb 19, 2024
 *      Author: brett
 */

#ifndef INCLUDESSRC_SPI1_H_
#define INCLUDESSRC_SPI1_H_

void spi1_init();
int spi1_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int spi1_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif /* INCLUDESSRC_SPI1_H_ */
