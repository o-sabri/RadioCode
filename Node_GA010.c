/*
*********************************************************************************
*
*	MasterPing
*
*	MRF24J40 demo for two XLP 16 boards with a 2.4GHz RF pictail board each.
*
*	This is based on code originally created by NERD FEVER (nerdfever.com)
*	which in turn is based on Microchip MiWi P2P stack. It is adapted by me to
*	compile for and run on either the PIC24F16KA102 or PIC24FJ64GA102 the device,
*   which are both supported by the XLP 16 board.
*
*	For more information visit http://nerdfever.com/?p=1797
*
*	Author: Fernando Solano
*	Date: 2012-09-12
*
*********************************************************************************
*/

#define _IN_MRF24J40RadioDemoPic24_

#include "hardwareprofile.h"
#include "radiohardware.h"
#include "MRF24J40.h"
#include "TimeDelay.h"
#include "uart.h"
#include "rtcc.h"
#include "misc.h"

#define CYCLE_PRESC	10		// 2.56ms cycle tick

#define MY_ID 3
#define NUM_OF_NODES 5
#define HELLO 3
#define HELLORES 2
#define JOIN 1
#define JOINBACK 0
#define MAX_RESPONDERS 4

 
typedef struct {
    unsigned short id;
    unsigned short nl;
    unsigned short nc[5];//set
    unsigned short s[5];
    unsigned short rhm[5]; //Received hello message
}NodeInfo;

typedef struct {// template for all messages
    unsigned short id;
    unsigned short sender_id;//the node which should handle the response
    unsigned short nl;
    unsigned short nc[5];//set
    unsigned short s[5];
    unsigned short t;//type of message 1- hello message,2-hello response,3- join,4-join back
}Message;

static NodeInfo nodeInfo = { //initialisation
    .id = MY_ID,
    .nl = 0,
    .nc = {0},
    .s = {0},
    .rhm = {0}
};
Message* receivedHelloResMsgs[4]={0};
BOOL receiving = FALSE;//used to indicate whether the timer is still running

//
static unsigned cyclePresc=1;	// For timing
static unsigned ledRxTimer=0;	// Timer for rx led
static unsigned ledTxTimer=0;	// Timer for tx led
static unsigned short timer1x10=0;	// 
static unsigned short txCounter = 0;
static unsigned long currentTimestamp=0, multipleTestEndTimestamp;
static unsigned channel=17;		// Current RF channel to use. Must be same in both nodes.
static BYTE ZERO_TX_AMP = 0x80;
static BYTE MAX_TX_POWER_EU = 0x70;
static BYTE MAX_TX_POWER_USA = 0x18;
static BYTE MAX_TX_POWER = 0x18; // 0x18 - USA, 0x70 = EU;
static BYTE MIN_TX_POWER = 0xF8;
static BYTE CURRENT_TX_POWER = 0x80;
static BYTE displayOutputBits = 0xFF;		// Bit-0: tx msg, Bit-1: rx msg, Bit-2: status msg, Bit-3: tx led, Bit-4: rx led
static unsigned short rxPacketCounter, NUM_TEST_PACKETS = 10;
static BYTE *txPayload;

void sendHelloMessage(void) {
    BYTE uartText[255];

    static Message helloMessage={
        .id = {MY_ID},
        .nl = {0},
        .nc = {0},// connected nodes
        .s = {0},
        .sender_id = {0},
        .t = HELLO
    };

    helloMessage.s[0]= MY_ID;//add master to set
    Tx.payload = &helloMessage;// makes hello message current payload
    
		sprintf((char*)uartText, "TXNode info id=%u t:%u", helloMessage.id , helloMessage.t);
	RadioTXPacket();	// Payload has already been set up and isn't changed
	txCounter++;
	//if((displayOutputBits & 0x01) == 0x01)
		UARTPrintLine(uartText);
//start timer
}
starttimer(){
    receiving =TRUE;
}
//when the timer expires it should call processCollectedHelloRespMsgs
void processHello(Message* receivedHellomessage){
    int i;
    BOOL exists = FALSE; // check if hello msg received from particular sender

    for(i=0;i<NUM_OF_NODES;i++){
        if(nodeInfo.rhm[i] ==0){
            break;
        }
        if (nodeInfo.rhm[i] == receivedHellomessage->id){
            exists =TRUE;
            break;
        }


    }
    if(exists == FALSE){
        nodeInfo.rhm[i]=receivedHellomessage->id;
        nodeInfo.nl +=1;//incement nl because we added new link
        sendHelloRespMessage(receivedHellomessage->id);
    }

    return;
}

void sendHelloRespMessage(int sender_id) {
    BYTE uartText[255];

    static Message helloRespMessage = {
    .id = MY_ID,
    .nl = {0},
    .nc = {0},
    .s = {0},
    .sender_id = {0},
    .t= HELLORES
    };

    helloRespMessage.nl = nodeInfo.nl;
    helloRespMessage.sender_id = sender_id;
    //helloRespMessage.s[0]= MY_ID;//add master to set
    Tx.payload = &helloRespMessage;// makes hello response message current payload
    ReadRTCC();
	if((displayOutputBits & 0x08) == 0x08)
		ledTxTimer=100;		// If 100, Keep tx LED on for ~250ms
	//txNodeInfo.id = 73;				// Counter incremented by master

	if((displayOutputBits & 0x01) == 0x01)
		sprintf((char*)uartText, "send  id=%u nl=%u t=%u", nodeInfo.id,nodeInfo.nl,helloRespMessage.t);
	RadioTXPacket();	// Payload has already been set up and isn't changed
	txCounter++;
	if((displayOutputBits & 0x01) == 0x01)
		UARTPrintLine(uartText);
}

collectHelloResMsgs(Message* receivedHellomessage){
    int i=0;
    for (i=0;i<MAX_RESPONDERS;i++){
        if (receivedHelloResMsgs[i]==NULL) {
            receivedHelloResMsgs[i]= receivedHellomessage;
            break;
        }
    }

}
processCollectedHelloResMsgs(void){
    int i=0;
    for(i=0;i<MAX_RESPONDERS;i++){
        if(receivedHelloResMsgs[i]!=NULL)
            processHelloResp(receivedHelloResMsgs[i]);
    }
    //maybe receivedHelloResMsgs={0};//dump collected hello responses
}
processHelloResp(Message* receivedHellomessage){
    return;
}
BOOL	HandleRxPacket(void)
// Do something with the rx packet. Here the LCD is updated with data from the rx packet.
{
	BYTE uartText[255];	
	if((displayOutputBits & 0x02) == 0x02){
		//DATA_PAYLOAD* receivedPayload = (DATA_PAYLOAD*) Rx.payload;
                Message* receivedPayload = (Message*) Rx.payload;
		 //sprintf((char*)uartText,"RX MCnt=%u MTs1=%lu MTs2=%lu Slqi=%u Srssi=%u STs=%lu SPow=%u Mlqi=%u Mrssi=%u MPow=%u",
			//receivedPayload->masterCnt, receivedPayload->masterTimestamp, currentTimestamp,
			//receivedPayload->slaveLqi, receivedPayload->slaveRssi, receivedPayload->slaveTimestamp, receivedPayload->slaveTxPower, Rx.lqi,Rx.rssi, receivedPayload->masterTxPower);
	sprintf((char*)uartText, "received t:%u from id:%u",receivedPayload->t,receivedPayload->id);
        
        if (receivedPayload->t == HELLORES){
            sprintf((char*)uartText, "received hello response from id=%u with nl=%u",receivedPayload->id,receivedPayload->nl);
            if(receiving ==TRUE && receivedPayload->sender_id ==MY_ID)
                collectHelloResMsgs(receivedPayload);
        }
        else if(receivedPayload->t == HELLO){
            sprintf((char*)uartText, "received hello from id=%u",receivedPayload->id);
               processHello(receivedPayload);
        }
        else if(receivedPayload->t == JOIN){
            sprintf((char*)uartText, "received test id=%u",receivedPayload->id);
               //processHello(receivedPayload);
               //sendHelloRespMessage();
        }
        else if(receivedPayload->t == JOINBACK){
            sprintf((char*)uartText, "received test id=%u",receivedPayload->id);
               //processHello(receivedPayload);
               //sendHelloRespMessage();
        }
		UARTPrintLine(uartText);	
	}
	rxPacketCounter++;
	return TRUE;
}







void RadioInitP2P(void)
// Initialize the tx packet. This only needs to be done once if the framecontrol and dest address isn't
// changed - as here.
{
	Tx.frameControl.frameType = PACKET_TYPE_DATA;
	Tx.frameControl.securityEnabled = 0;
	Tx.frameControl.framePending = 0;
	Tx.frameControl.ackRequest = 0;
//	Tx.frameControl.ackRequest = 1;
	Tx.frameControl.panIDcomp = 1;
	Tx.frameControl.dstAddrMode = SHORT_ADDR_FIELD;
	Tx.frameControl.frameVersion = 0;
	Tx.frameControl.srcAddrMode = NO_ADDR_FIELD;
	Tx.dstPANID = RadioStatus.MyPANID;
	Tx.dstAddr = RadioStatus.MyShortAddress;		// Both nodes for this demo uses the same addresses.
	//Tx.payload = (BYTE*)&txPayload;
        //Tx.payload = (BYTE*)&txNodeInfo;
        //Tx.payload = txPayload;
	Tx.payloadLength=sizeof(Message);
}		

void __attribute__((interrupt,shadow,auto_psv)) _T1Interrupt(void)
// Timer 1 interrupt routine. Set up to generate an interrupt every 256us.
// Used for general timekeeping.
{
	MRF24J40_Timer();	// Update radio timer. This is used for timeouts.
		
	// Cycle timer (2.56ms)
	cyclePresc--;
	if (cyclePresc==0){
		cyclePresc=CYCLE_PRESC;
		if (ledRxTimer!=0){
			ledRxTimer--;
		}
		if (ledTxTimer!=0){
			ledTxTimer--;
		}	
		if (timer1x10 == 0)
			currentTimestamp++;
		timer1x10 = (timer1x10 + 1) % 20;	//	Adjusts myTimer to increment every 200*256us = 50ms, cannot be greater than the packet generation frequency (otherwise will repeat timers)
	}
	
	IFS0bits.T1IF=0;	
}


///
/*
TODO: Explorer16 seems to handle INT0 for something else
void __attribute__((interrupt,shadow,auto_psv)) _INT0Interrupt(void)		
{
	sendTestPacket();
	IFS0bits.INT0IF = 0;
}
*/

short getRadioPowerx10(BYTE power){
	short out = 200;
	short firstDigit = (power & 0b11000000) >> 6;
 	short secondDigit = (power & 0b00111000) >> 3;	
	out -= firstDigit * 100;
	switch(secondDigit){
		case 7:
			out -= 63;
			break;
		case 6:
			out -= 49;
			break;
		case 5:
			out -= 37;
			break;
		case 4:
			out -= 28;
			break;
		case 3:
			out -= 19;
			break;
		case 2:
			out -= 12;
			break;
		case 1:
			out -= 5;
			break;
	}
	return out;
}

void printRadioStatus(){
	BYTE uartText[255];	
	BYTE currentPower = RadioGetTransmitPower();
	if((displayOutputBits & 0x04) == 0x04){
		short power = getRadioPowerx10(currentPower);
		short integer = power / 10;
		short decimal = power - integer * 10;
		if(decimal < 0) 
			decimal *= -1;
		sprintf((char*)uartText,"* Current Tx Power to: %d.%d dBm", integer, decimal);
		UARTPrintLine(uartText);	
	}
}

void changePower(BYTE currentPower){
	BYTE uartText[255];	
	if((displayOutputBits & 0x04) == 0x04){
		short power = getRadioPowerx10(currentPower);
		short integer = power / 10;
		short decimal = power - integer * 10;
		if(decimal < 0) 
			decimal *= -1;
		sprintf((char*)uartText,"* Request to change Tx Power to: %d.%d dBm", integer, decimal);
		UARTPrintLine(uartText);	
	}	
	if(currentPower <= MAX_TX_POWER)
		RadioSetTransmitPower(MAX_TX_POWER);
	else if(currentPower >= MIN_TX_POWER)
		RadioSetTransmitPower(MIN_TX_POWER);
	else if(currentPower >= MAX_TX_POWER && currentPower <= MIN_TX_POWER){
		RadioSetTransmitPower(currentPower);
	}
	printRadioStatus();
}

void printHelp()
{
	UARTPrintLine("* Options:");
        UARTPrintLine("y - Send Hello Message");
	UARTPrintLine("b - Send burst of 10 packets");
	UARTPrintLine("s - Print status of the radio");
	UARTPrintLine("p - Perform ping-power test");
	UARTPrintLine("t - Enters/exits continuos transmission mode");
	UARTPrintLine("e - Sets power to EU max standard");
	UARTPrintLine("u - Sets power to USA max standard");
	UARTPrintLine("z - 0 signal amplifications");
	UARTPrintLine("i - Increase transmission power by one");
	UARTPrintLine("d - Decrease transmission power by one");
	UARTPrintLine("[0-9] - Sets power transmission level (0=highest)");
	UARTPrintLine("h - Print this");
}

int main(void)
{
	BYTE uartText[255];	
	int blink;
	UINT8 lastFrameNumber=0xff;
	
	BoardInit();						// Set up board hardware (not the radio)
	RadioHW_Init(TRUE);					// Set up 2.4GHz RF pictail hardware
	UARTInit();							// Set up LCD
	InitRTCC();
	DelayMs(150);
	TMR1On(TMR1_PERIOD,TRUE);			// Start Timer 1
	SRbits.IPL=3;						// enable CPU priority levels 4-7 interrupts
	
				// Blink LED_1 and LED_2 a couple of times at reset.
#if __PIC24FJ128GA010__
	for(blink=5;blink>0;blink--){
		LED_1=1;//LED_1=0;
		LED_2=0;//LED_2=1;
		DelayMs(50);
		LED_2=1;//LED_2=0;
		LED_1=0;//LED_1=1;
		if (blink!=1){
			DelayMs(50);
		}
	}
	LED_1=1;//LED_1=0;	
#endif	
	
	UARTPrintLine("Welcome to Power-ping - Master!");
	UARTPrintLine("* Initializing radio... ");
	RadioInit();						// Initialize radio
	Delay10us(120);						// Wait for it to be ready	
	RadioSetChannel(channel);			// Change channel if other than default is desired.
	sprintf((char*)uartText,"* MRF24J40 on ch%d",channel);		// Welcome message on LCD
	UARTPrintLine(uartText);
	RadioInitP2P();						// Init Tx packet structure.
	UARTPrintLine("* Radio on");

	lowWrite(WRITE_RXMCR,1); // bit 0=1 => Don't filter packets on PAN/address. For test.

		// Repeat forever
	unsigned short multipleTest = 0, totalPacketCounter;		// 1 = Ping-power, 2 = Continuous Tx
	while(1){
		while(RadioRXPacket()){ 	// Anything to receive? RadioRXPacket() must be called repeatedly.
			if (Rx.frameNumber != lastFrameNumber){	// Have we already received this packet?
													// Note that the framenumber is reset to 0 whenever RadioInit()
													// is called, so if the txing radio goes to sleep between txing
													// Rx.frameNumber can't be used here. Use a variable in the 
													// payload instead if handling duplicates matters.
													
				lastFrameNumber=Rx.frameNumber;		
				if((displayOutputBits & 0x10) == 0x10)
					ledRxTimer=100;						// If 100, Keep rx LED on for ~250ms
			
				HandleRxPacket();					// Do something with the received packet
														// Don't take too long in this routine, otherwise 
													// packets might be missed. 
			}	
			RadioDiscardPacket();					// Tell radio code that we are done with this packet.
		}
		

		BYTE currentPower = RadioGetTransmitPower();
		if(multipleTest == 0){
			if(UARTIsPressed() == 1){
				char key = UARTGetChar();
				switch(key){
					case 'i':
						currentPower -= 8;
						changePower(currentPower);
						break;
					case 'd':
						currentPower += 8;
						changePower(currentPower);
						break;
					case 's':
						printRadioStatus();
						break;
                                        case 'y':
                                            sendHelloMessage();
                                            break;
					case 'b':
						totalPacketCounter = NUM_TEST_PACKETS;
						rxPacketCounter = 0;			
						multipleTest = 1;
						displayOutputBits = 0x00;
						multipleTestEndTimestamp = currentTimestamp + 100;
						sprintf((char*)uartText,"* Starting transmission burst of %u packets...", NUM_TEST_PACKETS);		// Welcome message on LCD
						UARTPrintLine(uartText);
						break;
					case 't':
						multipleTest = 2;
						displayOutputBits = 0x00;
						UARTPrintLine("Entering continous transmission mode...");
						break;
					case 'h':
						printHelp();
						break;
					case 'e':
						changePower(MAX_TX_POWER_EU);
						break;
					case 'u':
						changePower(MAX_TX_POWER_USA);
						break;
					case 'z':
						changePower(ZERO_TX_AMP);
						break;
					case '9':
						changePower(MIN_TX_POWER);
						break;
					case '0':
						changePower(MAX_TX_POWER);
						break;
					default:
						if(key >= '1' && key <= '8'){
							BYTE reqPowerKey = key - '0';
							BYTE step = (MIN_TX_POWER - MAX_TX_POWER) >> 3;
							step = step / 8;
							step = step << 3;
							BYTE reqPower = step * reqPowerKey;
							changePower(reqPower);
						}
						break;
				}
			}
		}
		else if(multipleTest == 1){
			if(totalPacketCounter > 0){
				//sendTestPacket();
                                //sendTest2Packet();
				totalPacketCounter--;
			}
			else
				if(multipleTestEndTimestamp <= currentTimestamp){
					multipleTest = 0;
					displayOutputBits = 0xFF;
					sprintf((char*)uartText,"* Sent %u packets, received %u packets", NUM_TEST_PACKETS, rxPacketCounter);		// Welcome message on LCD
					UARTPrintLine(uartText);
				}
		}
		else if(multipleTest == 2){
			//sendTestPacket();
                        //sendTest2Packet();
			if(UARTIsPressed() == 1){
				char key = UARTGetChar();
				if(key == 't'){
					multipleTest = 0;
					displayOutputBits = 0xFF;
					UARTPrintLine("Exiting continous transmission mode...");
				}
			}
		}

#if __PIC24FJ128GA010__
			// Rx LED
		if (ledRxTimer==0){
			LED_1=1;//LED_1=0;	// Off
		} else {
			LED_1=0;//LED_1=1;	// On
		}

			// Tx LED
		if (ledTxTimer==0){
			LED_2=1;//LED_2=0;	// Off
		} else {
			LED_2=0;//LED_2=1;	// On
		}
#endif
	}
	return 0;
}


#undef _IN_MRF24J40RadioDemoPic24_



