
/* Includes ------------------------------------------------------------------*/
#include "gpio_functions.h"
#include "stm32l4xx_ll_gpio.h"
#include "time_functions.h"
#include "main.h"

LL_GPIO_InitTypeDef GPIO_InitStruct;



void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);

  GPIO_InitStruct.Pin = LED_G_PIN|LED_R_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOA, LED_G_PIN|LED_R_PIN);

  GPIO_InitStruct.Pin = LED_B_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOB, LED_B_PIN);

  GPIO_InitStruct.Pin = USB_CONNECTIVITY;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(USB_CONNECTIVITY_GPIO_PORT, &GPIO_InitStruct);

  /**/
  // Greg start comment out
//  GPIO_InitStruct.Pin = ENC1_CS_PIN;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ENC1_CS_GPIO_PORT, &GPIO_InitStruct);
  // Greg end comment out

//  GPIO_InitStruct.Pin = ENC1_SCLK_PIN;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ENC1_SCLK_GPIO_PORT, &GPIO_InitStruct);

  // Greg start comment out
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ENC1_SCLK_GPIO_PORT, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ENC1_SCLK_GPIO_PORT, &GPIO_InitStruct);
  // Greg end comment out

  // Greg start comment out
//  GPIO_InitStruct.Pin =  ENC1_DATA_PIN;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
//  LL_GPIO_Init(ENC1_DATA_GPIO_PORT, &GPIO_InitStruct);
  // Greg end comment out

  GPIO_InitStruct.Pin = ENC2_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ENC2_CS_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =  ENC2_DATA_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(ENC2_DATA_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENC2_SCLK_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ENC2_SCLK_GPIO_PORT, &GPIO_InitStruct);



  GPIO_InitStruct.Pin = SPI1_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI1_CS_GPIO_PORT, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN);

  GPIO_InitStruct.Pin = SPI2_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI2_CS_GPIO_PORT, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN);

  GPIO_InitStruct.Pin = SPI3_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI3_CS_GPIO_PORT, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(SPI3_CS_GPIO_PORT, SPI3_CS_PIN);

  GPIO_InitStruct.Pin = SPI1_IMU2_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI1_IMU2_CS_GPIO_Port, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(SPI1_IMU2_CS_GPIO_Port, SPI1_IMU2_CS_Pin);

  GPIO_InitStruct.Pin = SPI3_IMU5_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI3_IMU5_CS_GPIO_Port, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(SPI3_IMU5_CS_GPIO_Port, SPI3_IMU5_CS_Pin);


  /**/
  GPIO_InitStruct.Pin = SD_DETECT_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /**/
//
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;   // Portable IMU2 CS Pin
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);


  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;   // Portable IMU3 CS Pin
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);

  // Greg start
  // Enable pin for boost regulator
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);

  // Measure interrupt speed
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Greg end
}

void RED_LED_ON(void)
{
#ifdef LED_Signal_En
LL_GPIO_ResetOutputPin(GPIOA, LED_R_PIN);
#endif
}

//void RED_LED_OFF(void)
//{
//LL_GPIO_SetOutputPin(GPIOA, LED_R_PIN);
//}

void GREEN_LED_ON(void)
{
#ifdef LED_Signal_En
LL_GPIO_ResetOutputPin(GPIOA, LED_G_PIN);
#endif
}

//void GREEN_LED_OFF(void)
//{
//
//LL_GPIO_SetOutputPin(GPIOA, LED_G_PIN);
//}

void BLUE_LED_ON(void)
{
#ifdef LED_Signal_En
LL_GPIO_ResetOutputPin(GPIOB, LED_B_PIN);
#endif
}

void BLUE_LED_OFF(void)
{
LL_GPIO_SetOutputPin(GPIOB, LED_B_PIN);
}

void ALL_LED_OFF(void)
{
#ifdef LED_Signal_En
	LL_GPIO_SetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_R_PIN);
#endif
}

void ALL_LED_ON(void)
{
#ifdef LED_Signal_En
	LL_GPIO_ResetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_R_PIN);
#endif
}

void GREEN_LED_ONLY(void)
{
#ifdef LED_Signal_En
	LL_GPIO_ResetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_SetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_R_PIN);
#endif
}


// New knee prosthesis LED
void RED_LED(void){
	// Greg start comment out
//LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_11); // RED LED
	// Greg end comment out
}

void GREEN_LED(void){
	// Greg start comment out
//LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_10); // GREEN LED
	// Greg end comment out
}

void YELLOW_LED(void){
	// Greg start comment out
//LL_GPIO_SetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_8);  // YELLOW LED
	// Greg end comment out
}


void RED_LED_OFF(void){
	// Greg start comment out
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_11); // RED LED
	// Greg end comment out

}

void GREEN_LED_OFF(void){
	// Greg start comment out
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_10); // GREEN LED
	// Greg end comment out
}

void YELLOW_LED_OFF(void){
	// Greg start comment out
//	LL_GPIO_ResetOutputPin(ENC1_CS_GPIO_PORT, LL_GPIO_PIN_8);  // YELLOW LED
	// Greg end comment out
}



void RED_LED_ONLY(void)
{
#ifdef LED_Signal_En
	LL_GPIO_ResetOutputPin(GPIOA, LED_R_PIN);
	LL_GPIO_SetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_G_PIN);
#endif
}

void BLUE_LED_ONLY(void)
{
#ifdef LED_Signal_En
	LL_GPIO_ResetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_R_PIN);
#endif
}

void VIOLET_LED_ONLY(void)
{
#ifdef LED_Signal_En
	LL_GPIO_ResetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_R_PIN);
#endif
}

void YELLOW_LED_ONLY(void)
{
#ifdef LED_Signal_En
	LL_GPIO_SetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_R_PIN);
#endif
}

void CYAN_LED_ONLY(void)
{
#ifdef LED_Signal_En
	LL_GPIO_ResetOutputPin(GPIOB, LED_B_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, LED_R_PIN);
#endif
}

/* SD CARD power pin */
void SD_POWER_ON(void)
{

}
void SD_POWER_OFF(void)
{

}




void Gr_LED_Blinking_5s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;


  for(i =0; i<10; i++)
  {
    LL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
    LL_mDelay(500);
  }
#endif
}



void Blue_LED_Blinking_10s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;


  for(i =0; i<20; i++)
  {
    LL_GPIO_TogglePin(GPIOB, LED_B_PIN);
    LL_mDelay(200);
  }
#endif
}

void Gr_LED_Blinking_2s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;


  for(i =0; i<10; i++)
  {
    LL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
    LL_mDelay(500);
  }
#endif
}

void Red_LED_Blinking_2s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;
  ALL_LED_OFF();

  for(i =0; i<20; i++)
  {
	LL_GPIO_TogglePin(GPIOA, LED_R_PIN);
    LL_mDelay(500);

    if (LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))
    break;
  }
  ALL_LED_OFF();
#endif
}

void Yellow_LED_Blinking_2s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;
ALL_LED_OFF();


  for(i =0; i<20; i++)
  {
    LL_GPIO_TogglePin(GPIOA, LED_R_PIN);
    LL_GPIO_TogglePin(GPIOA, LED_G_PIN);
    LL_mDelay(500);

    if (LL_GPIO_IsInputPinSet(GPIOA,USB_CONNECTIVITY))
    break;
  }

	LL_GPIO_SetOutputPin(GPIOA, LED_G_PIN);
	LL_GPIO_SetOutputPin(GPIOA, LED_R_PIN);
#endif
}

void Blue_LED_Blinking_2s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;


  for(i =0; i<10; i++)
  {
    LL_GPIO_TogglePin(GPIOB, LED_B_PIN);
    LL_mDelay(500);
  }
  LL_GPIO_SetOutputPin(GPIOB, LED_B_PIN);

#endif
}

void Blue_LED_Blinking_5s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;


  for(i =0; i<25; i++)
  {
    LL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    LL_mDelay(200);
  }
#endif
}

void Red_LED_Blinking_5s(void)  // Calculate Delay On-Demand
{
#ifdef LED_Signal_En
  uint32_t i=0;


  for(i =0; i<10; i++)
  {
    LL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    LL_mDelay(1000);
  }
#endif
}
