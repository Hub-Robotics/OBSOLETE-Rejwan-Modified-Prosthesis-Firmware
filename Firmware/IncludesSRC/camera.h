/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DCMI_OV5642_H
#define __DCMI_OV5642_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"
#include "stm32l4xx_ll_i2c.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint8_t Manufacturer_ID1;
  uint8_t Manufacturer_ID2;
  uint8_t Version;
  uint8_t PID; 
} OV5642_IDTypeDef;

void Camera_Init_Sleep(void);
void CAPTURE_IMAGE_Loop_Wk_Sleep(void);
void Camera_Reset(void);
void Camera_ReInit(void);
void FPGA_Reset(void);

void OV5640_step_default(void);
void OV5640_step_manual(void);
void OV5640_adjust_camera(void);
void frame_delay(int frame_number);
void Read_Image_Size(void);
void FPGA_capture_image(void);
void FPGA_Reset_Power_On(void);

/* Exported constants --------------------------------------------------------*/

/* Use this define to set the maximum delay timeout for the I2C DCMI_SingleRandomWrite()
   and DCMI_SingleRandomRead() operations. Exeeding this timeout delay, the read/write 
   functions will be aborted and return error code (0xFF).
   The period of the delay will depend on the system operating frequency. The following
   value has been set for system running at 120MHz. */

#define DCMI_TIMEOUT_MAX  10000

#define OV5642_DEVICE_WRITE_ADDRESS    0x78  // the device address for ov5642 (write)
#define OV5642_DEVICE_READ_ADDRESS     0x79  // the device address for ov5642 (read)

/* OV5642 Registers definition */
#define OV5642_GAIN       0x00
#define OV5642_BLUE       0x01
#define OV5642_RED        0x02
#define OV5642_VREF       0x03
#define OV5642_COM1       0x04
#define OV5642_BAVE       0x05
#define OV5642_GbAVE      0x06
#define OV5642_GrAVE      0x07
#define OV5642_RAVE       0x08
#define OV5642_COM2       0x09
#define OV5642_PID        0x0A
#define OV5642_VER        0x0B
#define OV5642_COM3       0x0C
#define OV5642_COM4       0x0D
#define OV5642_COM5       0x0E
#define OV5642_COM6       0x0F
#define OV5642_AEC        0x10
#define OV5642_CLKRC      0x11
#define OV5642_COM7       0x12
#define OV5642_COM8       0x13
#define OV5642_COM9       0x14
#define OV5642_COM10      0x15
#define OV5642_REG16      0x16
#define OV5642_HSTART     0x17
#define OV5642_HSTOP      0x18
#define OV5642_VSTART     0x19
#define OV5642_VSTOP      0x1A
#define OV5642_PSHFT      0x1B
#define OV5642_MIDH       0x1C
#define OV5642_MIDL       0x1D
#define OV5642_MVFP       0x1E
#define OV5642_BOS        0x20
#define OV5642_GBOS       0x21
#define OV5642_GROS       0x22
#define OV5642_ROS        0x23
#define OV5642_AEW        0x24
#define OV5642_AEB        0x25
#define OV5642_VPT        0x26
#define OV5642_BBIAS      0x27
#define OV5642_GbBIAS     0x28
#define OV5642_PREGAIN    0x29
#define OV5642_EXHCH      0x2A
#define OV5642_EXHCL      0x2B
#define OV5642_RBIAS      0x2C
#define OV5642_ADVFL      0x2D
#define OV5642_ADVFH      0x2E
#define OV5642_YAVE       0x2F
#define OV5642_HSYST      0x30
#define OV5642_HSYEN      0x31
#define OV5642_HREF       0x32
#define OV5642_CHLF       0x33
#define OV5642_AREF1      0x34
#define OV5642_AREF2      0x35
#define OV5642_AREF3      0x36
#define OV5642_ADC1       0x37
#define OV5642_ADC2       0x38
#define OV5642_AREF4      0x39
#define OV5642_TSLB       0x3A
#define OV5642_COM11      0x3B
#define OV5642_COM12      0x3C
#define OV5642_COM13      0x3D
#define OV5642_COM14      0x3E
#define OV5642_EDGE       0x3F
#define OV5642_COM15      0x40
#define OV5642_COM16      0x41
#define OV5642_COM17      0x42
#define OV5642_MTX1       0x4F
#define OV5642_MTX2       0x50
#define OV5642_MTX3       0x51
#define OV5642_MTX4       0x52
#define OV5642_MTX5       0x53
#define OV5642_MTX6       0x54
#define OV5642_BRTN       0x55
#define OV5642_CNST1      0x56
#define OV5642_CNST2      0x57
#define OV5642_MTXS       0x58
#define OV5642_AWBOP1     0x59
#define OV5642_AWBOP2     0x5A
#define OV5642_AWBOP3     0x5B
#define OV5642_AWBOP4     0x5C
#define OV5642_AWBOP5     0x5D
#define OV5642_AWBOP6     0x5E
#define OV5642_BLMT       0x5F
#define OV5642_RLMT       0x60
#define OV5642_GLMT       0x61
#define OV5642_LCC1       0x62
#define OV5642_LCC2       0x63
#define OV5642_LCC3       0x64
#define OV5642_LCC4       0x65
#define OV5642_MANU       0x66
#define OV5642_MANV       0x67
#define OV5642_MANY       0x68
#define OV5642_VARO       0x69
#define OV5642_BD50MAX    0x6A
#define OV5642_DBLV       0x6B
#define OV5642_DNSTH      0x70
#define OV5642_POIDX      0x72
#define OV5642_PCKDV      0x73
#define OV5642_XINDX      0x74
#define OV5642_YINDX      0x75
#define OV5642_SLOP       0x7A
#define OV5642_GAM1       0x7B
#define OV5642_GAM2       0x7C
#define OV5642_GAM3       0x7D
#define OV5642_GAM4       0x7E
#define OV5642_GAM5       0x7F
#define OV5642_GAM6       0x80
#define OV5642_GAM7       0x81
#define OV5642_GAM8       0x82
#define OV5642_GAM9       0x83
#define OV5642_GAM10      0x84
#define OV5642_GAM11      0x85
#define OV5642_GAM12      0x86
#define OV5642_GAM13      0x87
#define OV5642_GAM14      0x88
#define OV5642_GAM15      0x89
#define OV5642_COM18      0x8B
#define OV5642_COM19      0x8C
#define OV5642_COM20      0x8D
#define OV5642_DMLNL      0x92
#define OV5642_DMLNH      0x93
#define OV5642_LCC6       0x9D
#define OV5642_LCC7       0x9E
#define OV5642_AECH       0xA1
#define OV5642_BD50       0xA2
#define OV5642_BD60       0xA3
#define OV5642_COM21      0xA4
#define OV5642_GREEN      0xA6
#define OV5642_VZST       0xA7
#define OV5642_REFA8      0xA8
#define OV5642_REFA9      0xA9
#define OV5642_BLC1       0xAC
#define OV5642_BLC2       0xAD
#define OV5642_BLC3       0xAE
#define OV5642_BLC4       0xAF
#define OV5642_BLC5       0xB0
#define OV5642_BLC6       0xB1
#define OV5642_BLC7       0xB2
#define OV5642_BLC8       0xB3
#define OV5642_CTRLB4     0xB4
#define OV5642_FRSTL      0xB7
#define OV5642_FRSTH      0xB8
#define OV5642_ADBOFF     0xBC
#define OV5642_ADROFF     0xBD
#define OV5642_ADGbOFF    0xBE
#define OV5642_ADGrOFF    0xBF
#define OV5642_COM23      0xC4
#define OV5642_BD60MAX    0xC5
#define OV5642_COM24      0xC7

/* Registers bit definition */
/* COM1 Register */
#define CCIR656_FORMAT  0x40
#define HREF_SKIP_0     0x00
#define HREF_SKIP_1     0x04
#define HREF_SKIP_3     0x08

/* COM2 Register */
#define SOFT_SLEEP_MODE  0x10	
#define ODCAP_1x         0x00	
#define ODCAP_2x         0x01	
#define ODCAP_3x         0x02	
#define ODCAP_4x         0x03
	
/* COM3 Register */
#define COLOR_BAR_OUTPUT         0x80
#define OUTPUT_MSB_LAS_SWAP      0x40
#define PIN_REMAP_RESETB_EXPST   0x08 
#define RGB565_FORMAT            0x00 
#define RGB_OUTPUT_AVERAGE       0x04 
#define SINGLE_FRAME             0x01

/* COM5 Register */
#define SLAM_MODE_ENABLE      0x40
#define EXPOSURE_NORMAL_MODE  0x01

/* COM7 Register */
#define SCCB_REG_RESET                       0x80
#define FORMAT_CTRL_15fpsVGA                 0x00
#define FORMAT_CTRL_30fpsVGA_NoVArioPixel    0x50
#define FORMAT_CTRL_30fpsVGA_VArioPixel      0x60
#define OUTPUT_FORMAT_RAWRGB                 0x00
#define OUTPUT_FORMAT_RAWRGB_DATA            0x00
#define OUTPUT_FORMAT_RAWRGB_INTERP          0x01
#define OUTPUT_FORMAT_YUV                    0x02
#define OUTPUT_FORMAT_RGB                    0x03

/* COM9 Register */
#define GAIN_2x         0x00	
#define GAIN_4x         0x10	
#define GAIN_8x         0x20	
#define GAIN_16x        0x30	
#define GAIN_32x        0x40	
#define GAIN_64x        0x50	
#define GAIN_128x       0x60	
#define DROP_VSYNC      0x04	
#define DROP_HREF       0x02
	
/* COM10 Register */
#define RESETb_REMAP_SLHS    0x80
#define HREF_CHANGE_HSYNC    0x40
#define PCLK_ON              0x00
#define PCLK_OFF             0x20
#define PCLK_POLARITY_REV    0x10
#define HREF_POLARITY_REV    0x08
#define RESET_ENDPOINT       0x04
#define VSYNC_NEG            0x02
#define HSYNC_NEG            0x01

/* TSLB Register */
#define PCLK_DELAY_0         0x00
#define PCLK_DELAY_2         0x40
#define PCLK_DELAY_4         0x80
#define PCLK_DELAY_6         0xC0
#define OUTPUT_BITWISE_REV   0x20
#define UV_NORMAL            0x00
#define UV_FIXED             0x10
#define YUV_SEQ_YUYV         0x00
#define YUV_SEQ_YVYU         0x02
#define YUV_SEQ_VYUY         0x04
#define YUV_SEQ_UYVY         0x06
#define BANDING_FREQ_50      0x02

#define RGB_NORMAL   0x00 
#define RGB_565      0x10 
#define RGB_555      0x30 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t DCMI_OV5642Config(void);
uint8_t DCMI_OV5640Config(void);
void DCMI_Control_IO_Init(void);
void DCMI_OV5642_Reset(void);
void DCMI_OV5642_QQVGASizeSetup(void);
void DCMI_OV5642_QVGASizeSetup(void);
void DCMI_OV5642_JPEG(void);
void OV2640_QVGA(void);
void OV2640_JPEG_Init(void);
void OV2640_yuv422(void);
void OV2640_JPEG(void);
void OV2640_320x240_JPEG(void);
void OV2640_160x120_JPEG(void);
void OV2640_1600x1200_JPEG(void);
void OV5642_dvp_fmt_global_init(void);
void OV5642_dvp_fmt_jpeg_5M(void);
void OV5642_dvp_fmt_jpeg_qvga(void);
void DCMI_OV5642_ReadID(OV5642_IDTypeDef* OV5642ID);
void DCMI_OV5642_SetPrescaler(uint8_t OV5642_Prescaler);
void DCMI_OV5642_SelectOutputFormat(uint8_t OV5642_OuputFormat);
void DCMI_OV5642_SelectFormatResolution(uint8_t OV5642_FormatResolution);
void DCMI_OV5642_SetRegister(uint8_t OV5642_Register, uint8_t Register_Val);
void DCMI_OV5642_HREFControl(uint8_t OV9665_HREFControl);
void DCMI_OV5642_SelectRGBOption(uint8_t OV9665_RGBOption);
void DCMI_SingleRandomWrite(uint8_t Device, uint16_t Addr, uint8_t Data);
uint8_t DCMI_SingleRandomRead(uint8_t Device, uint16_t Addr);
void OV5642_jpeg_5M_test(void);
void CAPTURE_IMAGE(void);
void update_Camera_FATFS_time(void);
void Configure_Camera(void);

/* FPGA functions */
void FPGA_GPIO_INIT(void);
void FPGA_RESET_ON(void);
void FPGA_RESET_OFF(void);
void FPGA_POWER_ON(void);
void FPGA_POWER_OFF(void);
void FPGA_SWITCH2CAPTURE(void);
void FPGA_SWITCH2READ(void);

/* Camera functions */
void STORE_IMAGE_SD(void);
void CAMERA_GPIO_INIT(void);
void CAMERA_POWER_ON(void);
void CAMERA_POWER_OFF(void);
void Demo_CAPTURE_IMAGE(void);
void CAPTURE_IMAGE_Loop(void);
void Convert_Camera_FileName_Unix_Time(void);
void update_Camera_FATFS_time(void);
void DCMI_OV5640_Config_Masudul(void);
void Set_ex_values(void);

/* Sturctures holding camera register values */
struct camera_registers {
	uint16_t cam_address;
	uint8_t cam_value;
};

static const struct camera_registers test[] =
{
	{0x3819, 0x81},
	{0x3503, 0x00}  //AWE Manual Mode Control //0x07
};

extern char SDPath[4];

#endif /* __DCMI_OV5642_H */



/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/

