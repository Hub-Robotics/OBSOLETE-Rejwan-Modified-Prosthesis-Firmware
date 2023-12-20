#include "CAN.h"
#include "sensor.h"

/*configuration for 125 kbs*/
////Configures the can bus for the desired baud rate. See "CAN Controller with Integrated Transceiver" data sheet for more bit timing informaton
//void CAN_configure(){
//// CNF1: CONFIGURATION 1 REGISTER (ADDRESS: 2Ah)
//MCP_write(0x2A,0xC4);
//
//// CNF2: CONFIGURATION 2 REGISTER (ADDRESS: 29h)
//MCP_write(0x29,0xAA);
//
//// CNF3: CONFIGURATION 3 REGISTER (ADDRESS: 28h)
//MCP_write(0x28,0x05);
//}
/*configuration for 1mbps: https://www.kvaser.com/support/calculators/bit-timing-calculator/ */
void CAN_configure() {
// CNF1: CONFIGURATION 1 REGISTER (ADDRESS: 2Ah)
	MCP_write(0x2A, 0xC0);

// CNF2: CONFIGURATION 2 REGISTER (ADDRESS: 29h)
	MCP_write(0x29, 0x89);

// CNF3: CONFIGURATION 3 REGISTER (ADDRESS: 28h)
	MCP_write(0x28, 0x04);
}

// sets the CAN operation mode
void CAN_mode() {
// CANCTRL: CAN CONTROL REGISTER (ADDRESS: XFh)
	MCP_write(0x0F, 0x08);
// CANSTAT: CAN STATUS REGISTER (ADDRESS: XEh)
//MCP_read(0x0E);
}

// Sends the apropriate message information to the transmit buffers and then flags them for transmittal
void CAN_transmit(int CAN_ID, int CAN_message[]) {

	//These manipulate the given CAN_ID to fit into the SIDH and SIDL registers
	int SIDH = 0x00;
	int SIDL = 0x00;
	SIDH = CAN_ID >> 3;
	SIDL = CAN_ID << 5;

//  TXBxSIDH: TRANSMIT BUFFER x STANDARD IDENTIFIER HIGH REGISTER
// (ADDRESS: 31h, 41h, 51h)
	MCP_write(0x51, SIDH);

// TXBxSIDL: TRANSMIT BUFFER x STANDARD IDENTIFIER LOW REGISTER
// (ADDRESS: 32h, 42h, 52h)
	MCP_write(0x52, SIDL);

//  TXBxDLC: TRANSMIT BUFFER x DATA LENGTH CODE REGISTER
// (ADDRESS: 35h, 45h, 55h)
	MCP_write(0x55, 0x08);

// TXBxDn: TRANSMIT BUFFER x DATA BYTE n REGISTER
// (ADDRESS: 36h-3Dh, 46h-4Dh, 56h-5Dh)
// The data to be loaded into the transmit buffer for sending
	MCP_write(0x56, CAN_message[0]);
	MCP_write(0x57, CAN_message[1]);
	MCP_write(0x58, CAN_message[2]);
	MCP_write(0x59, CAN_message[3]);
	MCP_write(0x5A, CAN_message[4]);
	MCP_write(0x5B, CAN_message[5]);
	MCP_write(0x5C, CAN_message[6]);
	MCP_write(0x5D, CAN_message[7]);

// Setting the TXREQ bit high in the TXBxCRTL (0x30) register iniates message sending
// This bit remains high until the message is sent
	MCP_write(0x50, 0x0F);
}
