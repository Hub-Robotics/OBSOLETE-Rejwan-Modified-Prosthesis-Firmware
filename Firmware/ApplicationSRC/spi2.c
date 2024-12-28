/*
 * spi2.c
 *
 *  Created on: Jan 9, 2024
 *      Author: brett
 */

#include "main.h"

void spi2_init() {
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
		/*Mode 3 (Mode 1,1) */
		SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH; /*Clock 1 when idle and 0 when active */
		SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE; //Second clock transition is the first data capture edge
		SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;

		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
		SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
		SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
		SPI_InitStruct.CRCPoly = 7; //?
		LL_SPI_Init(SPI2, &SPI_InitStruct);

		LL_SPI_DisableNSSPulseMgt(SPI2);

		LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);
		LL_SPI_DisableIT_RXNE(SPI2);
		LL_SPI_Enable(SPI2);

		delay_us(10000);
}
