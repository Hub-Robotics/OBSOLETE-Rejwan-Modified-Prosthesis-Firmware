/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_FUNC_H
#define __GPIO_FUNC_H

void FPGA_clk_digital (void);
void FPGA_clk_SPI (void);

void FPGA_capture_read_input(void);
void FPGA_capture_read_output(void);

void LED_SPI_Mode(void);
void LED_Nucelo(void);
void MX_GPIO_Init(void);
void LP_SDIO_GPIO_Init(void);
void LP_STLink_GPIO_Init(void);
void LP_SPI_GPIO_Init(void);
void LP_I2C_GPIO_Init(void);
void LP_USB_GPIO_Init(void);
void LP_Common_GPIO_Init(void);
void LP_GPIO_Init(void);

/* LED functions */
void RED_LED_ON(void);
//void RED_LED_OFF(void);
void GREEN_LED_ON(void);
//void GREEN_LED_OFF(void);
void BLUE_LED_ON(void);
void BLUE_LED_OFF(void);
void ALL_LED_OFF(void);
void ALL_LED_ON(void);
void VIOLET_LED_ON(void);
void CYAN_LED_ONLY(void);
void RED_YELLOW_LED_ON(void);
void BLUE_LED_ONLY(void);
void GREEN_LED_ONLY(void);
void RED_LED_ONLY(void);
void YELLOW_LED_ONLY(void);
void VIOLET_LED_ONLY(void);
void Gr_LED_Blinking_5s(void);
void Gr_LED_Blinking_2s(void);
void Blue_LED_Blinking_2s(void);
void Blue_LED_Blinking_10s(void);

// New prosthesis LED
void RED_LED(void);
void GREEN_LED(void);
void YELLOW_LED(void);
void RED_LED_OFF(void);
void GREEN_LED_OFF(void);
void YELLOW_LED_OFF(void);


void LED_Nulceo_Blinking_5s(void);
void LED_Nulceo_Blinking_2s(void);
void LED_Nulceo_Blinking_1s(void);
void LED_Nucleo_OFF(void);
void LED_Nucleo_ON(void);

void Red_LED_Blinking_2s(void);
void Yellow_LED_Blinking_2s(void);
void Blue_LED_Blinking_10s(void);
/* SD CARD power pin */
void SD_POWER_ON(void);
void SD_POWER_OFF(void);

/* MCO Clock for I2C camera Device */
void MCO_PA8_Init(void);

/* Peripheral Clock Declaration */
void Enable_I2C_SPI_Peripheral_Clock(void);
void Disable_I2C_SPI_Peripheral_Clock(void);

#endif /* __GPIO_FUNC_H */
