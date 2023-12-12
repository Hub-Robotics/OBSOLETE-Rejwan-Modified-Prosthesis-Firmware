#include "sensor.h"
#include "CAN.h"
#include "EPOS4.h"
#include "time_functions.h"


//This is useful for later adding in functionality. Should work for any Client to Server SDO
void EPOS4_data_framer(int data[],int object, int subindex, int value){
    data[0] = 0x22; //[Byte 0] legend Table 5-43 page 5-55 Application Notes
    data[1] = (0x00 | object); //Index LowByte
    data[2] = (0x00 | (object >> 8)); //Index HighByte
    data[3] = subindex; //subindex
    data[4] = (0x00 | value); //SDO Byte 0
    data[5] = (0x00 | (value >> 8)); //SDO Byte 1
    data[6] = (0x00 | (value >> 16)); //SDO Byte 2
    data[7] = (0x00 | (value >> 24)); //SDO Byte 3
}

void EPOS4_set_operation_mode(int CAN_ID, int mode){
    int data[8];
    int object = 0x6060;
    int subindex = 0x00;
    int value = mode;
    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(10000);
}

void EPOS4_enable(int CAN_ID){

    int data[8];
    int object = 0x6040;
    int subindex = 0x00;
    int value = 0x0006;
    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(10000);

    EPOS4_enable2(CAN_ID);



//    value = 0x000F;
//    EPOS4_data_framer(data,object,subindex,value);
//

//    CAN_transmit(CAN_ID, data);
// delay_us(10000);

}


void EPOS4_enable2(int CAN_ID){

    int data[8];
    int object = 0x6040;
    int subindex = 0x00;
    int value = 0x000F;
    EPOS4_data_framer(data,object,subindex,value);
    CAN_transmit(CAN_ID, data);
    delay_us(10000);

}








void EPOS4_PVM_set_velocity(int CAN_ID, int rpm){
    int data[8];
    int object = 0x60FF;
    int subindex = 0x00;
    int value = rpm;

    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(10000);
}

void EPOS4_PVM_start(int CAN_ID){
    int data[8];
    int object = 0x6040;
    int subindex = 0x00;
    int value = 0x000F;
    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(10000);
}


void EPOS4_PVM_stop(int CAN_ID){
    int data[8];
    int object = 0x6040;
    int subindex = 0x00;
    int value = 0x010F;
    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(10000);
}


void EPOS4_CST_apply_torque(int CAN_ID, int torque){
    int data[8];
    int object = 0x6071;
    int subindex = 0x00;
    int value = torque;


    if (value < 0){
        value=0xFFFFFFFF+value+1;
    }
    else{

    }

    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(50); //1500
}

void EPOS4_CST_stop(int CAN_ID){
    int data[8];
    int object = 0x6071;
    int subindex = 0x00;
    int value = 0x00;
    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
    delay_us(1500);
}

void EPOS4_clear_errors(int CAN_ID){
    int data[8];
    int object = 0x6040;
    int subindex = 0x00;
    int value = 0x0080;
    EPOS4_data_framer(data,object,subindex,value);

    CAN_transmit(CAN_ID, data);
//    delay_us(200);
//    delay_us(1500); // testing by commenting it
}
