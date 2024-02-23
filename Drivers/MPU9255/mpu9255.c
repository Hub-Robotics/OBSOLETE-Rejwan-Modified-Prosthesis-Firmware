/*
 * mpu9255.c
 *
 *  Created on: Feb 22, 2024
 *      Author: brett
 */

#include "mpu9255.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l476xx.h"
#include "spi1.h"
#include "systick_app_timer.h"

/*
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
*/

static systick_app_timer_t timer;
static uint32_t g_ul_ms_ticks = 0;
static uint32_t ms_delay = 0;

// CS is active low. This disables the chip
__STATIC_INLINE void clearChipSelect() {
	LL_GPIO_SetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN);
}

// CS is active low. This enables the chip.
__STATIC_INLINE void setChipSelect() {
	LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN);
}

int mpu9255_write(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char const *data) {

	(void)slave_addr; // NOT NEEDED, I2C variable. Suppress unused variable warning

	setChipSelect();

	// 4 byte fifo. Respect that and write when free
	LL_SPI_TransmitData8(SPI1, reg_addr);
	for (uint8_t i = 0; i < length; i++) {
		while (!(SPI1->SR & SPI_SR_TXE)) {
		};
		LL_SPI_TransmitData8(SPI1, data[i]);
	}

	// wait for completion
	while ((SPI1->SR & SPI_SR_FTLVL)) {
	}; 	//transmit fifo empty?
	while ((SPI1->SR & SPI_SR_BSY)) {
	}; 	// no longer busy
	clearChipSelect();

	// clear receive fifo. Overrun data will be lost, but we are only sending.
	while ((SPI1->SR & SPI_SR_FRLVL)) {
		uint8_t dummy = SPI1->DR; 			// clear rx fifo from the receives.
		(void) dummy;						// suppress unused variable warning
	};
	return 0;
}

/*
 * Read assumes that calling function is providing a data location big enough
 * to handle the size of length. Be aware that this can overrun the calling
 * memory location if not sized appropriately!
 */
int mpu9255_read(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char *data) {

	(void) slave_addr; // not used, suppress unused variable warning.

	setChipSelect();

	// 4 byte fifo not used. Reads need to be captured.
	LL_SPI_TransmitData8(SPI1, reg_addr);
	while ((SPI2->SR & SPI_SR_FRLVL) == 0) {}; 	//wait for SR buffer to have data
	uint8_t dummy = SPI2->DR; 				// first byte is from cmd transfer. Ignore
	(void)dummy; // suppress unused variable warning

	// read the data sequentially
	for (uint8_t i = 0; i < length; i++) {
		LL_SPI_TransmitData8(SPI1, 0xFF);	// send clocking byte, not used.
		while ((SPI2->SR & SPI_SR_FRLVL) == 0) {}; // wait for SR buffer to have data
		data[i] = SPI1->DR;
	}

	while ((SPI1->SR & SPI_SR_BSY)) {
		}; 	// no longer busy
	clearChipSelect();

	return 0;
}

void mpu9255_delay_ms(unsigned long num_ms) {
	ms_delay = num_ms;
	while (ms_delay) {
		// do nothing while we wait.
	}
}

void mpu9255_get_ms(unsigned long *count) {
	*count = g_ul_ms_ticks;
}

void app_timer_event_handler() {
	g_ul_ms_ticks++;
	if (ms_delay) {
		ms_delay--;
	}
}

void mpu9255_init() {
	spi1_init();

	timer.mode = APP_TIMER_MODE_CONTINUOUS;
	timer.alarm = 10; // 10ms
	timer.timerAlarmCallback = app_timer_event_handler;
	systick_app_timer_channel_create(&timer);
}
