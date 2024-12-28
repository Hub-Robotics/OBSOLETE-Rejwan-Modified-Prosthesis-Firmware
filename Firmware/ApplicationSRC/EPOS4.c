#include "sensor.h"
//#include "CAN.h"
#include "mcp25625.h"
#include "EPOS4.h"
#include "time_functions.h"


//This is useful for later adding in functionality. Should work for any Client to Server SDO
void EPOS4_data_framer(uint8_t * data, uint16_t object, uint8_t subindex, uint32_t value){
    data[0] = 0x22; //[Byte 0] legend Table 5-43 page 5-55 Application Notes
    data[1] = (0x00 | object); //Index LowByte
    data[2] = (0x00 | (object >> 8)); //Index HighByte
    data[3] = subindex; //subindex
    data[4] = (0x00 | value); //SDO Byte 0
    data[5] = (0x00 | (value >> 8)); //SDO Byte 1
    data[6] = (0x00 | (value >> 16)); //SDO Byte 2
    data[7] = (0x00 | (value >> 24)); //SDO Byte 3
}

void EPOS4_set_operation_mode(uint16_t CAN_ID, uint32_t mode){
    uint8_t data[8];
//    uint16_t object = 0x6060;
//    uint8_t subindex = 0x00;

    EPOS4_data_framer(data, 0x6060 , 0, mode);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
}

void EPOS4_enable(uint16_t CAN_ID){

    uint8_t data[8];
//    uint16_t object = 0x6040;
//    int subindex = 0x00;
//    uint32_t value = 0x0006;
    EPOS4_data_framer(data, 0x6040, 0x00, 0x06);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);

    EPOS4_enable2(CAN_ID);



//    value = 0x000F;
//    EPOS4_data_framer(data,object,subindex,value);
//

//    CAN_transmit(CAN_ID, data);
// delay_us(10000);

}


void EPOS4_enable2(uint16_t CAN_ID){

    uint8_t data[8];
//    int object = 0x6040;
//    int subindex = 0x00;
//    int value = 0x000F;

    EPOS4_data_framer(data, 0x6040, 0x00, 0x0F);
    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);

}








void EPOS4_PVM_set_velocity(uint16_t CAN_ID, uint32_t rpm){
    uint8_t data[8];
//    int object = 0x60FF;
//    int subindex = 0x00;
//    int value = rpm;

    EPOS4_data_framer(data, 0x60FF, 0x00, rpm);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
}

void EPOS4_PVM_start(uint16_t CAN_ID){
    uint8_t data[8];
//    int object = 0x6040;
//    int subindex = 0x00;
//    int value = 0x000F;

    EPOS4_data_framer(data, 0x6040, 0x00, 0x0F);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
}


void EPOS4_PVM_stop(uint16_t CAN_ID){
    uint8_t data[8];
//    int object = 0x6040;
//    int subindex = 0x00;
//    int value = 0x010F;
    EPOS4_data_framer(data, 0x6040, 0x00, 0x010F);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
}


void EPOS4_CST_apply_torque(uint16_t CAN_ID, uint32_t torque){
    uint8_t data[8];
//    int object = 0x6071;
//    int subindex = 0x00;
//    int value = torque;
//
//
//    if (value < 0){
//        value=0xFFFFFFFF+value+1;
//    }
//    else{
//
//    }

    EPOS4_data_framer(data, 0x6071, 0x00, torque);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(50); //1500
}

void EPOS4_CST_stop(uint16_t CAN_ID){
    uint8_t data[8];
//    int object = 0x6071;
//    int subindex = 0x00;
//    int value = 0x00;
    EPOS4_data_framer(data, 0x6071, 0x00, 0x00);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(1500);
}

void EPOS4_clear_errors(uint16_t CAN_ID){
    uint8_t data[8];
//    int object = 0x6040;
//    int subindex = 0x00;
//    int value = 0x0080;
    EPOS4_data_framer(data, 0x6040, 0x00, 0x80);

    CAN_transmit(CAN_ID, 8, data);
//    delay_us(200);
//    delay_us(1500); // testing by commenting it
}
