#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage.h"
void P_Activate_ADC3(void);


int File_Write_Error,File_Open_Error,File_Sync_Error,File_Mount_Error, File_Error;

void Wifi_Init(void);

void put_low_power_while_charging (void);
void Execute_VCP_Command (void);
void USB_Init_Start (void);
void USB_VCP_Init_Start(void);
void USB_MSC_Init_Start(void);
void Shut_Down_USB(void);

void F_Sensor_ADC_Store(void);
int Read_Loadcell(void);
int Read_Loadcell1(void);
int Read_Loadcell2(void);

void read_Load1_ADC(void);
void read_Load2_ADC(void);
void battery_ADC_store_Buff(void);
void Mag_Enc1_Store(void);
void Mag_Enc2_Store(void);
void MPU_9D_store_IMU4_SPI3(void);
void MPU_9D_store_IMU5_SPI3(void);
void MPU_9D_store_IMU3_SPI2(int value1, int value2, int value3, int value4, int value5, int value6);
//void Knee_data_store(int val1, int val2, int val3, int val4, int val5, int val6, int val7, int val8);
void Knee_data_store(int val1, int val2, int val3, int val4);
void Knee_data_store1( int val5, int val6, int val7, int val8);
//void Knee_data_store1( int val5, int val6);
//void Knee_data_store2( int val7, int val8);


//void MPU_9D_store_IMU3_SPI2(void);


//void MPU_9D_store_IMU1_SPI1(void);
void Knee_data_storeIMU(int value1, int value2, int value3, int value4, int value5, int value6);

int IMU1_SPI1_acc_read(void);
void MPU_9D_store_IMU2_SPI1(void);
void MPU3_9D_store(void);
void read_acc_gyro3(void);
void FPGA_Response_Err_Timestamp(void);

void Datalog_Sensor_Initialization(void);

void Bypass_Datalog_Bottle (void);
void Prepare_Data_Log_State(void);


void USART3_UART_WIFI_Init(void);
void USART2_UART_Init(void);

void read_acc_gyro(void);
void read_acc_gyro1(void);
void read_acc_gyro2(void);
void Reset_Variables_for_LowBattery(void);
void AIM_Error_Handler(int AIM_Error_Code);

void Reset_DBG_buffer(void);
void Acc_Err_Timestamp(void);
void Gyro_Err_Timestamp(void);
void FPGA_Err_Timestamp(void);
void Camera_Err_Timestamp(void);
void Pres_Err_Timestamp(void);
void Bat_Err_Timestamp(void);
void ADC_Err_Timestamp(void);
void SD_Err_Timestamp(void);
void FPGA_loop_Err_Timestamp(void);
void Clear_ErrorTimestamp_Buffer(void);
void Save_ErrorTimestamp_Buffer(uint8_t ErrorCode);
void Non_critical_Sensor_Error_Handling(void);

void ADC_int_IN6_Battery(void);
void ADC_int_IN5_Strain(void);
void ADC_Power_Down(void);
void ADC_conf_battery(void);
void P_ADC_conf_strain(void);
void ADC_conf_strain(void);
void prepare_low_battery(void);
void FPGA_Programming_loop(void);

//Interrupt
void Deep_Sleep_Consumption_Test(void);
void Disable_Interrupts(void);
void Enable_Interrupts(void);
void Power_on_reset(void);
void Configure_Interrupt(void);
void ADC_Battery_Level_Test(void);
// Comparator
void Comp1_Inp1_Strain_Rec_Enable(void);
void Shut_Down_SD(void);
void File_Close_Update_Unlink(void);

void Battery_Indicator(void);

// FATFS SD buffer variables
void Reset_All(void);
void FATFS_Init(void);
void Open_File_For_Sensor_Write(void);
int GetNextIndex(char *path);
// Strain Sensor
void Strain_Sensor_GPIO_Init(void);
void Strain_Sensor_ADC_store(void);
void all_ADC_read(void);
// EXTI

void P_Activate_ADC1(void);
void P_Activate_ADC2(void);
void P_ADC1_conf_strain(void);
void P_ADC2_conf_strain(void);


void BTN_PA10_EXTI_conf(void);
void USB_PA9_EXTI_conf(void);
void P_ADC_Sensor_GPIO_Init(void);
void all_ADC_read_test(void);
void conf_ADC_DMA_poll(void);
void Start_Data_Collection_at_Reset(void);
void Power_on_reset (void);
void LP_ACC_GPIO_Init(void);
void Prepare_Goto_Dormant_Mode(void);
void ACC_GPIO_INIT(void);
void ACC2_GPIO_INIT(void);
void DFU_Bypass(void);
void DFU_Bypass_Bottle_Sensor(void);

void Pressure_Sensor_Rst(void);


void Data_Pause_Resume_PC0_EXTI_conf(void);
void Data_Collection_Paused(void);

// ADC
void ADC_Reset(void);
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);
void ConversionStartPoll_ADC_GrpRegular_Buf(void);

void Wait_for_ADC_TimeOut_ (void);
void Battery_Monitor_GPIO_init(void);
void Battery_Monitor_Voltage_check(void);
void AIM_DataStart_at_Reset(void);

void file_reopen_sync(void);
void file_reopen(void);

void Try_FATFS_Mount(void);

void Gyro_Reset(void);
void ACC_Reset(void);
void Wait_for_ADC_timeout_While_Datalog(void);
void Wait_for_SPI_timeout_While_Datalog(void);
void Wait_for_SPI_TimeOut_ (void);
void Acc_GetXYZ_Check(void);
void IMU_Store_Buffer(void);
void Capture_Frame_Decision(void);
void ADXL362_Init(void);
void ADXL362_Init_at_reset(void);
void MPU9250_Init_at_reset(void);
void LSM6DS3_Init(void);

// RTC
void MX_RTC_Init(void);
void Store_RTC_Time(void);
void Store_RTC_Date(void);
void Enter_RTC_InitMode(void);
void Exit_RTC_InitMode(void);
void Read_RTC_Timestamp(void);
void Convert_SD_FileName_Unix_Time(void);
void Convert_SD_FileName_RTC_Timestamp(void);
void RTC_BAK_SetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister, uint32_t Data);
uint32_t RTC_BAK_GetRegister(RTC_TypeDef *RTCx, uint32_t BackupRegister);
void Set_RTC_Calendar(uint8_t Year, uint8_t Month, uint8_t Day, uint8_t Hour, uint8_t Min, uint8_t Sec);

//FATFS SD
void Check_SD_Command_File (void);
void FATFS_SD_Open_Ready(void);
void FATFS_Logstart_Delete(void);
void SD_Sensor_write(void);
void Write_Timestamp_In_SD(void);
void update_FATFS_time(void);
int GetNeaxtIndex(char *path);
uint8_t read_conf_File(char *path);
//USB
void USB_MSC_Init_Start(void);
void VCP_mode_en(void);
void AIM_Error_Handler(int AIM_Error_Code);
void FATFS_Logstart_Ready(void);
//IMU
void ACC_SPI_SendData(uint8_t adress, uint8_t data);
void MPU_SPI_SendData(uint8_t adress, uint8_t data);
void MPU_SPI_GetData(uint8_t adress);
void MPU2_SPI2_GetData(uint8_t adress);
unsigned int WriteReg2(uint8_t adress, uint8_t data);
void MPU9250_Read_Buf(void);
void MPU9250_Read_Buf_2(void);

void ACC_SPI_GetData(uint8_t adress);
void LSM6DS3_SPI_Write(uint8_t adress, uint8_t data);
void LSM6DS3_Read_Buf(void);
void Acc_GetXYZ_Buf(void);
void IMU_Initialization(void);
void IMU_Initialization_at_reset(void);
void P_IMU1_SPI1_Initialization_at_reset(void);
void P_IMU3_SPI2_Initialization_at_reset(void);

//void MCP_SPI2_Initialization_at_reset(void);
//void MCP_setup(void);
//void MCP_reset();
//int MCP_read(int adress);
//void MCP_write(int adress, int data);


void P_IMU4_SPI3_Initialization_at_reset(void);
void MPU3_init(void);
void MPU_9D_store3(void);

void P_IMU4_SPI3_Init(void);
void P_IMU1_SPI1_Init(void);
void P_IMU3_SPI2_Init(void);


void MPU1_SPI1_init(void);
void MPU2_SPI1_init(void);
void MPU3_SPI2_init(void);
void MPU4_SPI3_init(void);
void MPU5_SPI3_init(void);
unsigned int whoami(void);
unsigned int whoami2(void);
//Pressure Sensor
void Pressure_read(void);
void read_acc();

// USART
void USART1_wr_print(uint8_t *buffer, uint32_t nBytes);
uint8_t USART1_read(void);
void Configure_USART_1(void);


void En1_read_position(void);
void En2_read_position(void);
int Enc2_GetPosition(void);
int Enc1_GetPosition(void);
float knee_angle(void);




struct imu_data
{
   int16_t  AX;
   int16_t  AY;
   int16_t  AZ;
   int16_t  GX;
   int16_t  GY;
   int16_t  GZ;
};

struct imu_angle
{
    float x;
    float y;
    float z;
};


float IMU_orientation( struct imu_data imuMyData, float last_angle, float dt_s);
struct imu_data IMU1_read(void);
