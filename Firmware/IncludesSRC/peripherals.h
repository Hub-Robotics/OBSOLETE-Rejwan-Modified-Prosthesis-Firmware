/**
  ******************************************************************************
  AIM peripherals
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PERIPHERALS_H
#define __PERIPHERALS_H

//FPGA Buffer SPI init
void MX_SPI2_Init(void);
void MX_LPUART1_UART_Init(void);

//I2C Init - Camera or Pressure sensor
void Configure_I2C1(void);
void Configure_I2C1_Pressure_Sensor(void);

void WIFI_UART3_Init(void);
void MX_ADC1_Init(void);
// IMU SPI1 Init
void MX_SPI1_Init(void);
void En2Configure_SPI2(void);
void En1Configure_SPI3(void);

#endif /* __PERIPHERALS_H */
