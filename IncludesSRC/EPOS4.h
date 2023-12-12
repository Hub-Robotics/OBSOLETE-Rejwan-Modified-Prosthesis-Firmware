//Fills out the can data array with the standard Object, Subindex, Value system used by EPOS4
void EPOS4_data_framer(int data[],int object, int subindex, int value);

//These are Universal
void EPOS4_set_operation_mode(int CAN_ID, int mode);
void EPOS4_enable(int CAN_ID);
void EPOS4_enable2(int CAN_ID);

//Related only to PVM
void EPOS4_PVM_set_velocity(int CAN_ID, int rpm);
void EPOS4_PVM_start(int CAN_ID);
void EPOS4_PVM_stop(int CAN_ID);
void EPOS4_clear_errors(int CAN_ID);
void EPOS4_CST_stop(int CAN_ID);
void EPOS4_CST_apply_torque(int CAN_ID, int torque);
