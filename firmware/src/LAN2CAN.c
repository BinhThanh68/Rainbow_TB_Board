


#include "lan2can.h"
#include "config_cocoa_general.h"
#include "function_utilities.h"
#include <stdio.h>
#include <math.h>
#include <proc/p32mz2048efh064.h>
#include "HX711.h"

#define LOADCELL_CALIB_FACTOR_AT_POS_1 741
#define LOADCELL_CALIB_FACTOR_AT_POS_2 877
#define LOADCELL_CALIB_FACTOR_AT_POS_3 1000
#define LOADCELL_CALIB_FACTOR_AT_POS_4 1116

ST_LOADCELL_DATA LOADCELL_INFO;

unsigned char start_making=0;
//float LOADCELL_DATA;
//bool LOADCELL_ENABLE = false;
//unsigned char LOADCELL_SAMPLE_NUM = 2;
unsigned char ICE_Wast_Water_Sensor;

LAN2CAN_DATA gv;
ST_ICE_DATA ICE_INFO;
ST_CUP_DATA CUP_INFO;
ST_DRINKOUT_DATA DRINKOUT_INFO;
ST_ICE_VALVE_DATA ICE_VALVE;
ST_ROMOTE_CONTROLER_DATA REMOTE_CONTROLLER;

unsigned char BARCODE_CNT = 0;
unsigned char BARCODE_DATA[BARCODE_SIZE] = {0,};
int BARCODE_LOCK = 0;

//averaged number of pulses in delay time
//#define DELAY_LOADCELL_TIME_IN_TIMER 35536644.65
#define LOADCELL_DELAY_TIME_1 30650000
#define LOADCELL_DELAY_TIME_2 20000000
float LOADCELL_THRESHOLD = 160;

unsigned char DIO_0[4] = {0,};
unsigned char DIO_1[4]= {0,};
 
int test_count = 1;

extern int BOARD_ID;


static int ice_connection_count = 20;
static int cup_connection_count = 20;


void PORTFunction();
void UART2Function();
void UART3Function();
void UART4Function();

    

void LAN2CAN_Initialize(void) {
    uint8_t tempIndex = 0;

    // LAN chip disable
    LAN2CAN_LANChipOff();

    // Initialize state machine
    gv.state = LAN2CAN_STATE_INIT;

    // Initialize system timer handle
    gv.sysTaskHandle = SYS_TMR_HANDLE_INVALID;
    gv.sysTmrUserHandle = SYS_TMR_HANDLE_INVALID;
    gv.sysTmrPCHandle = SYS_TMR_HANDLE_INVALID;
    gv.lanData.sysTmrLANCheckHandle = SYS_TMR_HANDLE_INVALID;
    gv.lanData.checkCount = 0;
        
    INIT_PINS(); 
        
    if(BOARD_ID == 0){
        // Outlet Motor * 8 --> UART2
        // Cup Dispenser --> UART3
        // Bardcode --> UART4
        // Outlet Sensor (Din) * 4
        // Relay (Dout) * 2
        
        INIT_UART_2(57600, 1, 0);
        INIT_UART_3(9600, 1, 0);
        INIT_UART_4(9600, 1, 0);
        
    }else if(BOARD_ID == 1){
        // Ice Valve --> UART2
        // Ice Dispenser --> UART3
        // Water Leveler
        
        INIT_UART_2(57600, 1, 0);
        INIT_UART_3(9600, 1, 0);
        
    }else if(BOARD_ID == 2){
        // Tea Leveler (Din) * 4
        // Load Cell (Digital) * 2
               
       HX711_begin(128);
       HX711_set_scale(LOADCELL_CALIB_FACTOR_AT_POS_1);
       HX711_tare(3);

       LOADCELL_INFO.isEnabled = 0;
       LOADCELL_INFO.focusMode = 0;
       LOADCELL_INFO.start_making = 0;
       LOADCELL_INFO.threshold = 160;
       
    }else if(BOARD_ID == 3){
       PORTBbits.RB2 =1;
       PORTBbits.RB3 =1;
       PORTBbits.RB4 =1;
    }
}

void LAN2CAN_Reset(void) {
    gv.sysTmrPCHandle = SYS_TMR_HANDLE_INVALID;
    
    // Reset LAN
    LAN2CAN_LANChipOn();
}

char data[4];
char data2[4];

int lan_connection_delay = 0;
float test_data[255];
float val1=0, val2=0, time_real, flowrate = 0, delta_time, pre_data, error_data, delayed_amount;
int t1, t2, time1, time2, time_end = 0;
int test=0, real_threshold = 160;
unsigned char focus_mode = 0;
int Test_datapasring=0;
void LAN2CAN_Tasks(void) {
    
    // Task
    LAN2CAN_TaskFunction(); 
    
    switch (gv.state) {
        case LAN2CAN_STATE_INIT:
        {
            // Reset peripheral device
            LAN2CAN_Reset();

            // Wait for the TCPIP Stack to become initialized                    
            if (TCPIP_STACK_Status(sysObj.tcpip) == SYS_STATUS_READY) {
                // now that the stack is ready we can check the
                 // available interfaces
                if (TCPIP_STACK_NumberOfNetworksGet() > 0) {
                    // Get a handle to the network interface
                    // For this application only one interface is used
                    gv.lanData.netHandle = TCPIP_STACK_IndexToNet(0);
                }

                gv.state = LAN2CAN_STATE_TCPIP_WAIT_FOR_LINK;
//                LAN2CAN_DEBUGMonitoring("Wait for link.\n");
            }
            break;
        }

        case LAN2CAN_STATE_TCPIP_WAIT_FOR_LINK:
        {
            if (TCPIP_STACK_NetIsLinked(gv.lanData.netHandle) == true) {
                // If network is linked than get IP address
                gv.lanData.ipAddr.Val = TCPIP_STACK_NetAddress(gv.lanData.netHandle);
                
                gv.state = LAN2CAN_STATE_SETUP_TCP_SERVER;
 //               LAN2CAN_DEBUGMonitoring("Setup TCP or UDP server.\n");
            }
            break;
        }

        case LAN2CAN_STATE_SETUP_TCP_SERVER:
        {
            //            IP_MULTI_ADDRESS tempAddr;
            //            tempAddr.v4Add.v[0] = 10;
            //            tempAddr.v4Add.v[1] = 0;
            //            tempAddr.v4Add.v[2] = 1;
            //            tempAddr.v4Add.v[3] = 1;
            //            gv.lanData.socketHandle = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, SERVER_PORT, &tempAddr);
            gv.lanData.socketHandle = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, SERVER_PORT, 0);
            if (gv.lanData.socketHandle != INVALID_SOCKET) {
                gv.state = LAN2CAN_STATE_WAIT_FOR_CONNECTION;

                // dummy check for reseting previous reset check
                TCPIP_TCP_WasReset(gv.lanData.socketHandle);
                //                if(TCPIP_TCP_WasReset(gv.lanData.socketHandle)){
                //                    LAN2CAN_DEBUGMonitoring("WasReset true\n");
                //                }else{
                //                    LAN2CAN_DEBUGMonitoring("WasReset false\n");
                //                }

//                LAN2CAN_DEBUGMonitoring("Wait for connection.\n");
            }
            break;
        }

        case LAN2CAN_STATE_WAIT_FOR_CONNECTION:
        {
            lan_connection_delay = 500;
            // Waiting client connection
            if (TCPIP_TCP_IsConnected(gv.lanData.socketHandle)) {
                //if (TCPIP_TCP_WasReset(gv.lanData.socketHandle)) {
                // Clear TCPIP Rx Buffer
                TCPIP_TCP_Discard(gv.lanData.socketHandle);

                // We got a connection
                gv.state = LAN2CAN_STATE_SERVING_CONNECTION;

                // CAN Buffer clear
//                LAN2CAN_CANClearBuffer();

            }
            break;
        }

        case LAN2CAN_STATE_SERVING_CONNECTION:
        {
            // Client is connected.
            // Now we can receive and transmit data

            // Checking a connection is closed or not.
            // If connection is closed, than we have to close and restart
            //if (!TCPIP_TCP_IsConnected(gv.lanData.socketHandle))
            //            if(TCPIP_TCP_WasReset(gv.lanData.socketHandle)){
            //                LAN2CAN_DEBUGMonitoring("----WasReset true\n");
            //            }else{
            //                LAN2CAN_DEBUGMonitoring("----WasReset false\n");
            //            }

            if (TCPIP_TCP_WasReset(gv.lanData.socketHandle)) {
                gv.state = LAN2CAN_STATE_CLOSING_CONNECTION;
                
            } else {
                // Send CAN message to the Main controller
                LAN2CAN_SendToPC();              
                // Check LAN data
                LAN2CAN_LANDataParsing();
                // Check LAN connection
//                LAN2CAN_LANCheckConnection();
            }

            break;
        }
       
        case LAN2CAN_STATE_CLOSING_CONNECTION:
        {
            // Close the socket connection.
            TCPIP_TCP_Close(gv.lanData.socketHandle);
            gv.state = LAN2CAN_STATE_SETUP_TCP_SERVER;
            break;
        }

        case LAN2CAN_STATE_ERROR:
        {
            // Re-initialize the program.
            // When we lost connection with the Main controller and don't know the reason,
            // then we need to reset the program.
            LAN2CAN_Initialize();
            break;
        }

        default:
            break;
    }
    
}

int LAN2CAN_LANChipOff(void) {
//    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_11);
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13);
    return GPIF_NO_ERR;
}

int LAN2CAN_LANChipOn(void) {
//    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_11);
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13);
    return GPIF_NO_ERR;
}

int LAN2CAN_LANDataParsing(void) {

    uint16_t receivedDataSize = 0;
    uint16_t tempSize = 0;
//    uint16_t tempIndex = 0;
    uint8_t tempData[3];
    UINT16_UNION packetDataSize;
//    UINT16_UNION noOfSettingMessage;

    tempSize = TCPIP_TCP_GetIsReady(gv.lanData.socketHandle);

//        if(tempSize > 4){
//            char buf[256];
//            sprintf(buf, "Size = %d\n", tempSize);
//            LAN2CAN_DEBUGMonitoring(buf);
//        }
    //if (tempSize > 8) { 
    if (tempSize > 3) {
        TCPIP_TCP_ArrayPeek(gv.lanData.socketHandle, tempData, 3, 0);
        if (tempData[0] != 0x24) {
//            LAN2CAN_DEBUGMonitoring("Wrong LAN header.\n");
            TCPIP_TCP_Discard(gv.lanData.socketHandle);
            return GPIF_ERR1;
        }

        packetDataSize.UINT8[0] = tempData[1];
        packetDataSize.UINT8[1] = tempData[2];

        uint16_t a = TCPIP_TCP_GetIsReady(gv.lanData.socketHandle);
        uint16_t b = packetDataSize.UINT16 + 3;
        bool c = (a < b);
        if (c) {            
            return GPIF_ERR2;
        }
        
//        if (TCPIP_TCP_GetIsReady(gv.lanData.socketHandle) < (packetDataSize.UINT16 + 3)) {
//            
//            return GPIF_ERR2;
//        }

        // Transfer the data out of the RX FIFO and into our local processing buffer.
        receivedDataSize = TCPIP_TCP_ArrayGet(gv.lanData.socketHandle, gv.lanData.msgFromClient, packetDataSize.UINT16 + 3);

        // Check packet footer
        if (gv.lanData.msgFromClient[receivedDataSize - 1] != 0x25) {
            // Wrong footer
//            LAN2CAN_DEBUGMonitoring("Wrong LAN footer.\n");
            return GPIF_ERR3;
        }

        // Packet is perfect, so we can parse the packet.
        // Reset LAN check count
        gv.lanData.checkCount = 0;
        if (receivedDataSize == 4) {
            //            TCPIP_TCP_ArrayPut(gv.lanData.socketHandle, gv.lanData.msgToClient, currentIndex);
            //            TCPIP_TCP_Flush(gv.lanData.socketHandle);
            return GPIF_NO_ERR;
        }

        // Check data type and execute
        // 0x00: CAN Data
        // 0x01: 
        // 0x02: DAC & DOUT
        // 0x03: Power command
        //GPIF_MonitoringWrite("Parsing started.\n");

        switch (gv.lanData.msgFromClient[5]) {
                // CAN Data packet handling
                
            case 0x10:
            {
                // Board ID 0 (192.168.100.110)
                // 0. Outlet Motor * 8 --> UART2
                // 1. Cup Dispenser --> UART3
                // Bardcode --> UART4
                // Outlet Sensor (Din) * 4
                // 2. Relay (Dout) * 2
            
                int target = gv.lanData.msgFromClient[6];
                int command = gv.lanData.msgFromClient[7];
                int para1 = ((int)(gv.lanData.msgFromClient[8]) | (int)(gv.lanData.msgFromClient[9]<<8) | (int)(gv.lanData.msgFromClient[10]<<16) | (int)(gv.lanData.msgFromClient[11]<<24));
                int para2 = ((int)(gv.lanData.msgFromClient[12]) | (int)(gv.lanData.msgFromClient[13]<<8) | (int)(gv.lanData.msgFromClient[14]<<16) | (int)(gv.lanData.msgFromClient[15]<<24));
                
                
                if(target == 0){
                    // Outlet Motor
                    switch(command){
                    case 0:
                        DRINKOUT_Spin(para1, para2);
//                        DRINKOUT_DoorWork(para1, para2);
                        break;
                    case 1:
                        //para2 = 6 to check moving status
                        SetMotorCommandState(para1, MOTOR_WAIT_READ_MOVING_STATUS);
                        DRINKOUT_CheckMovingStatus(para1);
                        break;
                    case 2:
                        //para2 = 7 to read position
                        SetMotorCommandState(para1, MOTOR_WAIT_READ_POSITION);
                        DRINKOUT_ReadPosition(para1);                        
                        break;                        
                    case 3:
                        SetMotorCommandState(para1, MOTOR_WAIT_SET_PROFILE);
                        DRINKOUT_SetProfile(para1);
                        break;
                    case 4:
                        SetMotorCommandState(para1, MOTOR_WAIT_SET_PP_GAIN);
                        DRINKOUT_SetPPGain(para1);
                        break;
                    case 5:
                        SetMotorCommandState(para1, MOTOR_WAIT_SET_PD_GAIN);
                        DRINKOUT_SetPDGain(para1);
                        break;
                    case 6:
                        SetMotorCommandState(para1, MOTOR_WAIT_ENABLE_TORQUE);
                        DRINKOUT_TurnTorque(para1, 1);
                        break;
                    }
                }else if(target == 1){
                    // Cup Dispenser
                    switch(command){
                        case 0:
                            CUP_RequestCupOut(para1, para2);
                            break;
                        case 1:
                            CUP_RequestSoldOut();
                            break;
                    }
                }else if(target == 2){
                    // Relay
                    
                }
                break;
            }
            
            case 0x11:
            {
                // Board ID 1 (192.168.100.111)
                // 0. Ice Valve --> UART2
                // 1. Ice Dispenser --> UART3
                // Water Leveler
                
                int target = gv.lanData.msgFromClient[6];
                int command = gv.lanData.msgFromClient[7];
                int para1 = ((int)(gv.lanData.msgFromClient[8]) | (int)(gv.lanData.msgFromClient[9]<<8) | (int)(gv.lanData.msgFromClient[10]<<16) | (int)(gv.lanData.msgFromClient[11]<<24));
                int para2 = ((int)(gv.lanData.msgFromClient[12]) | (int)(gv.lanData.msgFromClient[13]<<8) | (int)(gv.lanData.msgFromClient[14]<<16) | (int)(gv.lanData.msgFromClient[15]<<24));

                
                if(target == 0){
                    // Ice Valve
                    switch(command){
                        case 0:
                            //close or open
                            if(ICE_VALVE.lock_state){
                                ICE_VALVE.motor.command_state = MOTOR_WAIT_DOOR_OPEN;
                            }else{
                                ICE_VALVE.motor.command_state = MOTOR_WAIT_DOOR_CLOSE;
                            }
                            ICE_Valve_Set_Lock(para1);
                            break;
                    }
                }else if(target == 1){
                    // Ice Dispenser
                    switch(command){
                        case 0:
                            ICE_SetCommunicationMode(para1, para2);
                            break;
                        case 1:
                            ICE_RequestIceOutWaterOut((float)(para1)/1000.0, (float)(para2)/1000.0);
                            break;
                        case 2:
                            ICE_RequestClearStatusData();
                            break;
                        case 3:
                            ICE_SetIceTimeout(para1);
                            break;
                        case 4:
                            ICE_RequestRebootController();
                            break;
                        case 5:
                            ICE_SetAmbientTemperatureSetting(para1, para2);
                            break;
                    }
                }
                break;
            }
            
            case 0x12:
            {
                // Board ID 2 (192.168.100.112)
                // Load Cell (Digital) * 2
                // Tea Leveler (Din) * 4
                
                int target = gv.lanData.msgFromClient[6];
                int command = gv.lanData.msgFromClient[7];
                int para1 = ((int)(gv.lanData.msgFromClient[8]) | (int)(gv.lanData.msgFromClient[9]<<8) | (int)(gv.lanData.msgFromClient[10]<<16) | (int)(gv.lanData.msgFromClient[11]<<24));
                int para2 = ((int)(gv.lanData.msgFromClient[12]) | (int)(gv.lanData.msgFromClient[13]<<8) | (int)(gv.lanData.msgFromClient[14]<<16) | (int)(gv.lanData.msgFromClient[15]<<24));

                
                if(target == 0){
                    //load cell
                    switch(command){
                        case 0:
                            //set 0
                            HX711_tare(1);
                            break;
                        case 1:
                            LOADCELL_INFO.isEnabled = para1;
                            break;
                        case 2:
                            LOADCELL_INFO.threshold = para1;
                            break;
                        case 3:{
                            if(para1==1){
                                HX711_set_scale(LOADCELL_CALIB_FACTOR_AT_POS_1);
                            }else if(para1==2){
                                HX711_set_scale(LOADCELL_CALIB_FACTOR_AT_POS_2);
                            }
                            else if(para1==3){
                                HX711_set_scale(LOADCELL_CALIB_FACTOR_AT_POS_3);
                            }
                            else if(para1==4){
                                HX711_set_scale(LOADCELL_CALIB_FACTOR_AT_POS_4);
                            }
                            break;
                        }
                        case 4:
                            LOADCELL_INFO.start_making = para1;
                            break;
                        default:
                            break;
                    }
                }
                break;
            }
            
            case 0x13:
            {
                // Board ID 3 (192.168.100.113)
                // Remote Controller (LED * 3, Button * 4)
                
                int target = gv.lanData.msgFromClient[6];
                int command = gv.lanData.msgFromClient[7];
                int para1 = ((int)(gv.lanData.msgFromClient[8]) | (int)(gv.lanData.msgFromClient[9]<<8) | (int)(gv.lanData.msgFromClient[10]<<16) | (int)(gv.lanData.msgFromClient[11]<<24));
                int para2 = ((int)(gv.lanData.msgFromClient[12]) | (int)(gv.lanData.msgFromClient[13]<<8) | (int)(gv.lanData.msgFromClient[14]<<16) | (int)(gv.lanData.msgFromClient[15]<<24));

                
                if(target == 0){
                    RemoteController_LED_Operation(para1, para2);
                }
                break;
            }
            
            default:
                // Wrong data type/
//                LAN2CAN_DEBUGMonitoring("Wrong data type.\n");
                return GPIF_ERR6;
                break;
        }
    }

    return GPIF_NO_ERR;
}

int LAN2CAN_LANCheckConnection(void) {
    // Set 1msec system timer
    if (gv.lanData.sysTmrLANCheckHandle == SYS_TMR_HANDLE_INVALID) {
        gv.lanData.sysTmrLANCheckHandle = SYS_TMR_DelayMS(10);
        return GPIF_NO_ERR;
    }
    if (SYS_TMR_DelayStatusGet(gv.lanData.sysTmrLANCheckHandle) == false) {
        return GPIF_NO_ERR;
    }
    gv.lanData.sysTmrLANCheckHandle = SYS_TMR_DelayMS(10);
    gv.lanData.checkCount++;

    if(lan_connection_delay > 0)
        lan_connection_delay--;
    
    //    char buf[256];
    //    sprintf(buf, "gv.lanData.checkCount = %d\n", gv.lanData.checkCount);
    //    LAN2CAN_DEBUGMonitoring(buf);

    // 100*10 = 1000msec = 1sec
    if (gv.lanData.checkCount > 60) {//ORIGINAL 200
        gv.lanData.checkCount = 0;
        gv.state = LAN2CAN_STATE_CLOSING_CONNECTION;
        //gv.state = GPIF_STATE_ERROR;
//        LAN2CAN_DEBUGMonitoring("LAN check error.\n");
    }

    return GPIF_NO_ERR;
}



int LAN2CAN_SendToPC(void) {
        
    // Set 10msec system timer
    if (gv.sysTmrPCHandle == SYS_TMR_HANDLE_INVALID) {
        gv.sysTmrPCHandle = SYS_TMR_DelayMS(10);
        return;
    }

    if (SYS_TMR_DelayStatusGet(gv.sysTmrPCHandle) == false) {
        return;
    }
    gv.sysTmrPCHandle = SYS_TMR_DelayMS(10);

    
    uint8_t tempIndex = 0;
    uint16_t currentIndex = 0;
    uint16_t dataSize = 0;

    if(BOARD_ID == 0){
        // Board ID 0 (192.168.100.110)
        // Outlet Motor * 8 --> UART2
        // Cup Dispenser --> UART3
        // Bardcode --> UART4
        // Outlet Sensor (Din) * 4  [module_x.cupPresent]
        // Relay (Dout) * 2
        
        gv.lanData.msgToClient[currentIndex] = 0x24;        currentIndex++;
        // Calculate data size
        dataSize = 4 +76 + 7 + BARCODE_SIZE;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize);   currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize >> 8);  currentIndex++;
        // From slave   
        gv.lanData.msgToClient[currentIndex] = 0x01;       currentIndex++;
        // To master
        gv.lanData.msgToClient[currentIndex] = 0x00;       currentIndex++;
        // Data type: Board ID 0 (0xF0+0)
        gv.lanData.msgToClient[currentIndex] = 0xF0;       currentIndex++;
        
        // Outlet Motor (19*4 = 76byte)
        int module_cnt = 0;
        for(module_cnt = 0; module_cnt<4; module_cnt++){
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].cupPresent;                            currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Disk.connection;                       currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Disk.isTorqueOn;                       currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Disk.isProfileSet;                     currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Disk.isPPGainSet;                      currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Disk.isPDGainSet;                      currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Disk.movingStatus;                     currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Disk.presentPosition)&0xFF;           currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Disk.presentPosition >> 8)&0xFF;      currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Disk.presentPosition >>16)&0xFF;      currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Disk.presentPosition >> 24)&0xFF;     currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Door.connection;                       currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Door.isTorqueOn;                       currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Door.isProfileSet;                     currentIndex++;
            gv.lanData.msgToClient[currentIndex] = DRINKOUT_INFO.module[module_cnt].Door.movingStatus;                     currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Door.presentPosition)&0xFF;           currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Door.presentPosition >> 8)&0xFF;      currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Door.presentPosition >>16)&0xFF;      currentIndex++;
            gv.lanData.msgToClient[currentIndex] = (DRINKOUT_INFO.module[module_cnt].Door.presentPosition >> 24)&0xFF;     currentIndex++;
        }
        
        
        // Cup Data (7 byte)
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.connection;             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.operating_port;         currentIndex++;
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.output_setting_num;     currentIndex++;
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.output_current_num;     currentIndex++;
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.op_stat;                currentIndex++;
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.err_stat;               currentIndex++;
        gv.lanData.msgToClient[currentIndex] = CUP_INFO.soldout_stat;           currentIndex++;
        
        
         // Bar code Data (BARCODE_SIZE bytes)
        int idx = 0;
        for(idx=0; idx<BARCODE_SIZE; idx++){
            gv.lanData.msgToClient[currentIndex] =  BARCODE_DATA[idx];             currentIndex++;
        }
        
        
        // Packet footer
        gv.lanData.msgToClient[currentIndex] = 0x25;
        currentIndex++;
        
    }else if(BOARD_ID == 1){
        // Board ID 1 (192.168.100.111)
        // Ice Valve --> UART2
        // Ice Dispenser --> UART3
        // Water Leveler [ICE_VALVE.sensor]
        
        gv.lanData.msgToClient[currentIndex] = 0x24;        currentIndex++;
        // Calculate data size
        dataSize = 4 + 5 + 16;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize);   currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize >> 8);  currentIndex++;
        // From slave   
        gv.lanData.msgToClient[currentIndex] = 0x01;       currentIndex++;
        // To master
        gv.lanData.msgToClient[currentIndex] = 0x00;       currentIndex++;
        // Data type: Board ID 1 (0xF0+1)
        gv.lanData.msgToClient[currentIndex] = 0xF1;       currentIndex++;
        
        
        //Ice valve (5 bytes)
        gv.lanData.msgToClient[currentIndex] = ICE_VALVE.motor.connection;                                      currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_VALVE.motor.isProfileSet;                                    currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_VALVE.motor.isTorqueOn;                                      currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_VALVE.lock_state;                                            currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_VALVE.sensor;                                                currentIndex++;
        
        
        // Ice Data (16 byte)
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.connection;             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.comm_mode;              currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.cup_ignore;             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.setting_ambient_temperature_high;   currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.setting_ambient_temperature_low;    currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.ambient_temperature;    currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.evaporator_temperature; currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.condensor_temperature;  currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (ICE_INFO.timeout_sec)&0xFF;     currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (ICE_INFO.timeout_sec>>8)&0xFF;  currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.last_ice_out_time;      currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.last_water_out_time;    currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.status.B[0];            currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.status.B[1];            currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.status.B[2];            currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ICE_INFO.status.B[3];            currentIndex++;
        
                
        // Packet footer
        gv.lanData.msgToClient[currentIndex] = 0x25;
        currentIndex++;
        
    }else if(BOARD_ID == 2){
        // Board ID 2 (192.168.100.112)
        // Load Cell (Digital) * 2
        // Tea Leveler (Din) * 4
                
        gv.lanData.msgToClient[currentIndex] = 0x24;        currentIndex++;
        // Calculate data size
        dataSize = 4 + 7 + 4;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize);   currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize >> 8);  currentIndex++;
        // From slave   
        gv.lanData.msgToClient[currentIndex] = 0x01;       currentIndex++;
        // To master
        gv.lanData.msgToClient[currentIndex] = 0x00;       currentIndex++;
        // Data type: Board ID 2 (0xF0+2)
        gv.lanData.msgToClient[currentIndex] = 0xF2;       currentIndex++;
        
        
        // Load cell Data (7 byte)
        gv.lanData.msgToClient[currentIndex] = (int)(LOADCELL_INFO.data) & 0xFF;                     currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ((int)(LOADCELL_INFO.data) >> 8) & 0xFF;              currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ((int)(LOADCELL_INFO.data) >> 16) & 0xFF;             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ((int)(LOADCELL_INFO.data) >> 24) & 0xFF;             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = LOADCELL_INFO.isEnabled;                              currentIndex++;
        gv.lanData.msgToClient[currentIndex] = LOADCELL_INFO.start_making;                           currentIndex++;
        gv.lanData.msgToClient[currentIndex] = LOADCELL_INFO.targetPosition;                         currentIndex++;
        
        
        // Tea dispenser status (4 bytes)
        gv.lanData.msgToClient[currentIndex] = DIO_1[0];                        currentIndex++;
        gv.lanData.msgToClient[currentIndex] = DIO_1[1];                        currentIndex++;
        gv.lanData.msgToClient[currentIndex] = DIO_1[2];                        currentIndex++;
        gv.lanData.msgToClient[currentIndex] = DIO_1[3];                        currentIndex++;
        
        
        // Packet footer
        gv.lanData.msgToClient[currentIndex] = 0x25;
        currentIndex++;
        
    }else if(BOARD_ID == 3){
        // Remote Controller (LED * 3, Button * 4)
        
        gv.lanData.msgToClient[currentIndex] = 0x24;        currentIndex++;
        // Calculate data size
        dataSize = 4 + 4;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize);   currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize >> 8);  currentIndex++;
        // From slave   
        gv.lanData.msgToClient[currentIndex] = 0x01;       currentIndex++;
        // To master
        gv.lanData.msgToClient[currentIndex] = 0x00;       currentIndex++;
        // Data type: Board ID 3 (0xF0+3)
        gv.lanData.msgToClient[currentIndex] = 0xF3;       currentIndex++;
        
        
        // Remote Controller Data (4 bytes))
        gv.lanData.msgToClient[currentIndex] = REMOTE_CONTROLLER.startCleaning;                                 currentIndex++;
        gv.lanData.msgToClient[currentIndex] = REMOTE_CONTROLLER.stopCleaning;                                  currentIndex++;
        gv.lanData.msgToClient[currentIndex] = REMOTE_CONTROLLER.openPlatform;                                  currentIndex++;
        gv.lanData.msgToClient[currentIndex] = REMOTE_CONTROLLER.closePlatform;                                  currentIndex++;
 
        
        // Packet footer
        gv.lanData.msgToClient[currentIndex] = 0x25;
        currentIndex++;
        
    }
    else
    {
        // Test sending data from tb board to pc
        gv.lanData.msgToClient[currentIndex] = 0x24;        currentIndex++;
        // Calculate data size
        dataSize = 4 + 5;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize);   currentIndex++;
        gv.lanData.msgToClient[currentIndex] = (uint8_t) (dataSize >> 8);  currentIndex++;
        // From slave   
        gv.lanData.msgToClient[currentIndex] = 0x01;       currentIndex++;
        // To master
        gv.lanData.msgToClient[currentIndex] = 0x00;       currentIndex++;
              

        // test Data (5 byte)
        gv.lanData.msgToClient[currentIndex] = ('h');             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ('e');             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ('l');             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ('l');             currentIndex++;
        gv.lanData.msgToClient[currentIndex] = ('o');             currentIndex++;
                
        // Packet footer
        gv.lanData.msgToClient[currentIndex] = 0x25;
        currentIndex++;
        
    }
    
    
    TCPIP_TCP_ArrayPut(gv.lanData.socketHandle, gv.lanData.msgToClient, currentIndex);
    TCPIP_TCP_Flush(gv.lanData.socketHandle);
    
    return GPIF_NO_ERR;
}


void LAN2CAN_TaskFunction(void){    
    static int ice_cup_cnt = 0;
    static unsigned int cup_cnt = 0;

    PORTFunction();
    UART2Function();
    UART3Function();
    UART4Function();
    
    
    
    if(BOARD_ID == 2){
        if(LOADCELL_INFO.isEnabled){
            if(LOADCELL_INFO.focusMode == 0){
                if(test%350000==0){
                    LOADCELL_INFO.data = HX711_get_units(1);
                    printf("0 ");
                }
            }else if(LOADCELL_INFO.focusMode == 1){
                if(test%100000==0){
                    LOADCELL_INFO.data = HX711_get_units(1);
                    printf("1 ");
                }
            }else if(LOADCELL_INFO.focusMode == 2){
                if(test%10000==0){
                    LOADCELL_INFO.data = HX711_get_units(1);
                    printf("2 ");
                }
            }else{
                if(test%800==0){
                    LOADCELL_INFO.data = HX711_get_units(1);
                    printf("3 ");
                }
            }test++;

            if(LOADCELL_INFO.start_making){
                if(LOADCELL_INFO.focusMode == 0) { 
                    LOADCELL_INFO.focusMode = 1; 
                }
           
                if((LOADCELL_INFO.data >= ((real_threshold *30)/100)) && val1 == 0){            
                     time1 = ((int)(TMR3) << 16) | TMR2;
                     val1 = LOADCELL_INFO.data;
                    // printf("%f ",val1);
                }
                else if((LOADCELL_INFO.data > ((real_threshold *75)/100)) && val1 != 0 && flowrate == 0){    
                     time2 = ((int)(TMR3) << 16) | TMR2;
                     val2 = LOADCELL_INFO.data;
                   // printf("%f ",val2);
                     delta_time = (time2>time1) ? (time2-time1) : ((pow(2,32) - 1)-time2)+time1;
                     flowrate = ((val2-val1)*1000) / delta_time;
                     LOADCELL_INFO.focusMode = 2;
                     if(flowrate < 0){          
                     }
                     real_threshold = LOADCELL_INFO.threshold - (flowrate * LOADCELL_DELAY_TIME_1/1000);
                    // printf("%f ",LOADCELL_THRESHOLD);
                }
        
                if((LOADCELL_INFO.data > ((real_threshold *95)/100)) ){
                    LOADCELL_INFO.focusMode = 3;
                }
        
                if(LOADCELL_INFO.data >= real_threshold){
                    PORTBbits.RB5 = 0;
                    time1=0;time2=0;val1=0;val2=0;flowrate=0; LOADCELL_INFO.focusMode = 0; LOADCELL_INFO.start_making = 0;
                    real_threshold = LOADCELL_INFO.threshold;
                }else{
                    PORTBbits.RB5 = 1;
                }
            }
        }
    }
    

   
    
    
    // Loop Timer 50 msec
    if (gv.sysTaskHandle == SYS_TMR_HANDLE_INVALID) {
        gv.sysTaskHandle = SYS_TMR_DelayMS(50);
        return;
    }
    if (SYS_TMR_DelayStatusGet(gv.sysTaskHandle) == false) {
        return;
    }
    gv.sysTaskHandle = SYS_TMR_DelayMS(50);
    
    if(BOARD_ID == 0){
        cup_cnt++;
        if(cup_cnt%10 == 0){
            CUP_RequestStatus();
            cup_connection_count++;
        }
        
        if(cup_connection_count < 5){
            CUP_INFO.connection = 1;
        }else{
            CUP_INFO.connection = 0;
        }
        
    }
    if(BOARD_ID == 1){
        // Ice & Cup
        ice_cup_cnt++;
        switch(ice_cup_cnt%5){
            case 0:
                ICE_RequestAmbientTemperatureSetting();
                ice_connection_count++;
                break;
            case 1:
                ICE_RequestAmbientTemperature();
                ice_connection_count++;
                break;
            case 2:
                ICE_RequestEvaporatorCondensorTemperature();
                ice_connection_count++;
                break;
            case 3:
                ICE_RequestStatus();
                ice_connection_count++;
                break;
        }
        
        if(ice_connection_count < 10){
            ICE_INFO.connection = 1;
        }else{
            ICE_INFO.connection = 0;
        }
    }
    
    
    
    
}




void PORTFunction(){
    if(BOARD_ID == 0){
        // Board ID 0 (192.168.100.110)
        // Outlet Motor * 8 --> UART2
        // Cup Dispenser --> UART3
        // Bardcode --> UART4
        // Outlet Sensor (Din) * 4
        // Relay (Dout) * 2
        
        DRINKOUT_INFO.module[0].cupPresent = PORTBbits.RB2;
        DRINKOUT_INFO.module[1].cupPresent = PORTBbits.RB3;
        DRINKOUT_INFO.module[2].cupPresent = PORTBbits.RB4;
        DRINKOUT_INFO.module[3].cupPresent = PORTBbits.RB5;
        
    }else if(BOARD_ID == 1){
        // Board ID 1 (192.168.100.111)
        // Ice Valve --> UART2
        // Ice Dispenser --> UART3
        // Water Leveler
        
        ICE_VALVE.sensor = PORTBbits.RB4;
        
    }else if(BOARD_ID == 2){
        // Board ID 2 (192.168.100.112)
        // Load Cell (Digital) * 2
        // Tea Leveler (Din) * 4
        
        DIO_1[0] = PORTBbits.RB2;
        DIO_1[1] = PORTBbits.RB3;
        DIO_1[2] = PORTBbits.RB4;
        DIO_1[3] = PORTBbits.RB5;
        
    }else if(BOARD_ID == 3){
        // Board ID 3 (192.168.100.113)
        // Remote Controller (LED * 3, Button * 4)
        
        REMOTE_CONTROLLER.startCleaning = !PORTBbits.RB2;
        REMOTE_CONTROLLER.stopCleaning = !PORTBbits.RB3;
        REMOTE_CONTROLLER.openPlatform = !PORTBbits.RB4;
        REMOTE_CONTROLLER.closePlatform = !PORTBbits.RB5;
        
    }
}

unsigned int TEST_CNT = 0;
void UART2Function(){
    static unsigned char DrinkOut_state = 0;
    static unsigned char DrinkOut_index;
    static short DrinkOut_data_length = 0;
    static unsigned char DrinkOut_buf[30] = {0,};
    
    
 
    if(U2STAbits.URXDA == TRUE){
        unsigned char temp_ch = U2RXREG;
        
        
        if(BOARD_ID == 0 || BOARD_ID == 1){
            // Outlet Motor * 8
            // Ice Valve
            switch(DrinkOut_state){
                case 0:
                    if(temp_ch == 0xFF){
                        // match header 1
                        DrinkOut_state = 1;
                    }
                    break;
                case 1:
                    if(temp_ch == 0xFF){
                        // match header  2
                        DrinkOut_state = 2;
                    }else{
                        DrinkOut_state = 0;
                    }
                    break;
                case 2:
                    if((temp_ch == 0xFD) || (temp_ch == 0x00)){
                        // match header 3
                        DrinkOut_state = 3;
                    }else{
                        DrinkOut_state = 0;
                    }
                    break;
                case 3:
                    if((temp_ch == 0x00) || (temp_ch == 0xFD)){
                        // match header 4
                        DrinkOut_state = 4;
                        DrinkOut_index = 0;
                        DrinkOut_data_length = 0;
                    }else{
                        DrinkOut_state = 0;
                    }
                    break;    
                case 4:
                {
                    DrinkOut_buf[DrinkOut_index] = temp_ch;
                    DrinkOut_index++;
                    if((DrinkOut_index >= 3) && (DrinkOut_data_length == 0)){
                        DrinkOut_data_length = (unsigned short)((unsigned short)(DrinkOut_buf[1]) | ((unsigned short)(DrinkOut_buf[2]) << 8));
//                        DrinkOut_data_length = (DrinkOut_buf[1]);
                        
                        if(DrinkOut_data_length > 50){
                            DrinkOut_state = 0;
                        }
                    }
                    if((DrinkOut_data_length != 0) && (DrinkOut_index >= DrinkOut_data_length+3)){
                        DrinkOut_state = 5;
                        DrinkOut_data_length=0;
                    }
                }
                    break;
                case 5:
                {
                    if((DrinkOut_buf[3] != 0x55) || (DrinkOut_buf[4] != 0)){
                        //error occurred
                        DrinkOut_state = 0;
//                        Nop();
                        break;
                    }
                    int motor_id = DrinkOut_buf[0];
                    int module_num = (motor_id-1)/2;
                    if(motor_id%2 == 0){
                        // door
                        switch(DRINKOUT_INFO.module[module_num].Door.command_state){
                            case MOTOR_WAIT_PING:
                                DRINKOUT_INFO.module[module_num].Door.connection = true;
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_SET_PROFILE:
                                DRINKOUT_INFO.module[module_num].Door.isProfileSet = true;
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;   
                            case MOTOR_WAIT_ENABLE_TORQUE:
                                DRINKOUT_INFO.module[module_num].Door.isTorqueOn = true;
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_DOOR_OPEN:
                                DRINKOUT_INFO.module[module_num].doorstate = true;
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_DOOR_CLOSE:
                                DRINKOUT_INFO.module[module_num].doorstate = false;
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_READ_MOVING_STATUS:
                                DRINKOUT_INFO.module[module_num].Door.movingStatus = DrinkOut_buf[5];
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_READ_POSITION:
                                DRINKOUT_INFO.module[module_num].Door.presentPosition = (unsigned int)(((unsigned int)DrinkOut_buf[8]<<24) | ((unsigned int)DrinkOut_buf[7]<<16) | ((unsigned int)DrinkOut_buf[6]<<8) | ((unsigned int)DrinkOut_buf[5]));
                                DRINKOUT_INFO.module[module_num].Door.command_state = MOTOR_IDLE;
                                break;
                            default:
                                break;
                        }
                    }else{
                        // disk
                        switch(DRINKOUT_INFO.module[module_num].Disk.command_state){
                            case MOTOR_WAIT_PING:
                                DRINKOUT_INFO.module[module_num].Disk.connection = true;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_SET_PROFILE:
                                DRINKOUT_INFO.module[module_num].Disk.isProfileSet = true;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;      
                            case MOTOR_WAIT_SET_PROFILE_ACELE:
                                DRINKOUT_INFO.module[module_num].Disk.isProfileAceleSet = true;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;            
                            case MOTOR_WAIT_SET_PP_GAIN:
                                DRINKOUT_INFO.module[module_num].Disk.isPPGainSet = true;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;    
                            case MOTOR_WAIT_SET_PD_GAIN:
                                DRINKOUT_INFO.module[module_num].Disk.isPDGainSet = true;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_ENABLE_TORQUE:
                                DRINKOUT_INFO.module[module_num].Disk.isTorqueOn = true;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_GO_POS1:
                                DRINKOUT_INFO.module[module_num].currentPosition = 1;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_GO_POS2:
                                DRINKOUT_INFO.module[module_num].currentPosition = 2;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_GO_POS3:
                                DRINKOUT_INFO.module[module_num].currentPosition = 3;
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;
                            case MOTOR_WAIT_READ_MOVING_STATUS:
                                DRINKOUT_INFO.module[module_num].Disk.movingStatus = DrinkOut_buf[5];
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;  
                            case MOTOR_WAIT_READ_POSITION:
                                DRINKOUT_INFO.module[module_num].Disk.presentPosition = (unsigned int)(((unsigned int)DrinkOut_buf[8]<<24) | ((unsigned int)DrinkOut_buf[7]<<16) | ((unsigned int)DrinkOut_buf[6]<<8) | ((unsigned int)DrinkOut_buf[5]));
                                DRINKOUT_INFO.module[module_num].Disk.command_state = MOTOR_IDLE;
                                break;      
                            default:
                                break;
                        }
                    }
                    DrinkOut_state = 0;
                    break;
                }
                default:
                    break;            
            }
            
        }else{
            ;
        }
        
    }
    
    
//    case 9:{
//        if(ICE_VALVE.motor.command_state == MOTOR_WAIT_PING){
//            ICE_VALVE.motor.connection = true;
//            ICE_VALVE.motor.command_state = MOTOR_IDLE;
//        }
//        else if(ICE_VALVE.motor.command_state == MOTOR_WAIT_SET_PROFILE){
//            ICE_VALVE.motor.isProfileSet = true;
//            ICE_VALVE.motor.command_state = MOTOR_IDLE;
//        }
//        else if(ICE_VALVE.motor.command_state == MOTOR_WAIT_ENABLE_TORQUE){
//            ICE_VALVE.motor.isTorqueOn = true;
//            ICE_VALVE.motor.command_state = MOTOR_IDLE;
//        }
//        else if(ICE_VALVE.motor.command_state == MOTOR_WAIT_DOOR_OPEN){
//            ICE_VALVE.lock_state = false;
//            ICE_VALVE.motor.command_state = MOTOR_IDLE;
//        }
//        else if(ICE_VALVE.motor.command_state == MOTOR_WAIT_DOOR_CLOSE){
//            ICE_VALVE.lock_state = true;
//            ICE_VALVE.motor.command_state = MOTOR_IDLE;
//        }
//
//    }
}


void UART3Function(){
    static unsigned char ice_buf[10] = {0,};
    static int ice_state = 0;
    static int ice_index = 0;
    
    static unsigned char cup_buf[15] = {0,};
    static int cup_state = 0;
    static int cup_index = 0;
        
    
    if(U3STAbits.URXDA == TRUE){
        unsigned char temp_ch = U3RXREG;
        
        if(BOARD_ID == 0){
            // Cup Dispenser
            switch(cup_state){
                case 0:
                    if(temp_ch == 0x02){
                        // match STX
                        cup_state = 1;
                        
                        cup_buf[0] = 0x02;
                        cup_index = 1;
                    }
                    break;
                case 1:{
                    if((cup_index == 1) && (temp_ch != 0x07)){
                        // length must be 7
                        cup_index = 0;
                        break;
                    }
                    cup_buf[cup_index] = temp_ch;
                    cup_index++;
                    if(cup_index > 10){
                        cup_state = 2;
                    }
                }
                    break;
                case 2:
                    if(cup_buf[9] == 0x03){
                        // match ETX
                        short temp_sum = (short)(cup_buf[1]) + (short)(cup_buf[2]) + (short)(cup_buf[3]) + (short)(cup_buf[4]) +
                                        (short)(cup_buf[5]) + (short)(cup_buf[6]) + (short)(cup_buf[7]) +
                                        (short)(cup_buf[8]) + (short)(cup_buf[9]);
                        
                        if((temp_sum&0xFF) == cup_buf[10]){
                            // match BCC
                            CUP_INFO.operating_port = cup_buf[3];
                            CUP_INFO.output_setting_num = cup_buf[4];
                            CUP_INFO.output_current_num = cup_buf[5];
                            CUP_INFO.op_stat = cup_buf[6];
                            CUP_INFO.err_stat = cup_buf[7];
                            CUP_INFO.soldout_stat = cup_buf[8];
                                                        
                            cup_connection_count = 0;
                            cup_state = 0;
                        }else{
                            cup_state = 0;
                        }
                    }else{
                        cup_state = 0;
                    }
                    break;
                default:
                    break;
            }
        }else if(BOARD_ID == 1){
            // Ice Dispenser
            switch(ice_state){
                case 0:
                    if(temp_ch == 0x02){
                        // match STX
                        ice_state = 1;
                        
                        ice_buf[0] = 0x02;
                        ice_index = 1;
                    }
                    break;
                case 1:
                    ice_buf[ice_index] = temp_ch;
                    ice_index++;
                    if(ice_index >= 7){
                        ice_state = 2;
                    }
                    break;
                case 2:
                    if((ice_buf[1] == 0x00) && (ice_buf[6] == 0x03)){
                        // match direction && match ETX
                        if(ice_buf[1]^ice_buf[2]^ice_buf[3]^ice_buf[4] == ice_buf[5]){
                            // match checksum
                            switch(ice_buf[2]){
                                case 0xA0:
                                    ICE_INFO.comm_mode = ice_buf[3];
                                    ICE_INFO.cup_ignore = ice_buf[4];
                                    break;
                                case 0xB0:
                                    ICE_INFO.last_ice_out_time = ice_buf[3];
                                    ICE_INFO.last_water_out_time = ice_buf[4];
                                    break;
                                case 0xC0:
                                    // dummy
                                    break;
                                case 0xC1:
                                    ICE_INFO.setting_ambient_temperature_low = ice_buf[3];
                                    ICE_INFO.setting_ambient_temperature_high = ice_buf[4];
                                    break;
                                case 0xC2:
                                    ICE_INFO.ambient_temperature = ice_buf[3];
                                    break;
                                case 0xC3:
                                    ICE_INFO.evaporator_temperature = ice_buf[3];
                                    ICE_INFO.condensor_temperature = ice_buf[4];
                                    break;
                                case 0xD0:
                                    ICE_INFO.timeout_sec = ice_buf[3]*60 + ice_buf[4];
                                    break;
                                case 0xE0:
                                    // dummy
                                    break;
                                case 0xE1:
                                    // dummy
                                    break;
                                case 0xCF:
                                    ICE_INFO.status.B[0] = ice_buf[3];
                                    ICE_INFO.status.B[1] = ice_buf[4];
                                    break;
                            }
                            
                            ice_connection_count = 0;
                            ice_state = 0;
                        }else{
                            ice_state = 0;
                        }
                    }else{
                        ice_state = 0;
                    }
                    break;
            }
        }else{
            ;
        }
    }
}

void UART4Function(){
    static unsigned char barcode_buf[BARCODE_SIZE] = {0,};
    static int barcode_state = 0;
    static int barcode_index = 0;
    
    
    if(U4STAbits.URXDA == TRUE){
        unsigned char temp_ch = U4RXREG;
        if(BOARD_ID == 0){       
            // Barcode
            switch(barcode_state){
                case 0:
                    if(temp_ch == 'a'){
                        // match STX
                        barcode_state = 1;
                        barcode_index = 1;
                    }
                    break;
                case 1:
                {
                    if((temp_ch != 'z') && barcode_index < BARCODE_SIZE){
                        barcode_buf[barcode_index] = temp_ch;
                        barcode_index++;
                        
                    }else{
                        int i;
                        for(i = barcode_index; i<BARCODE_SIZE; i++){
                            barcode_buf[i] = 0;
                        }
                        // increase BARCODE_CNT
                        barcode_buf[0] = BARCODE_CNT;
                        BARCODE_CNT = (++BARCODE_CNT)%256;
                        
                        int idx = 0;
                        for(idx=0; idx<BARCODE_SIZE; idx++){
                            BARCODE_DATA[idx] = barcode_buf[idx];
                        }
                        barcode_state = 0;
                    }
                }
                    break;
                default:
                    break;
            }
        }
    }
}

void __ISR(_UART2_RX_VECTOR, ipl4AUTO) Handler_UART2_RX(void){
    //UART2Function();
    IFS4bits.U2RXIF = 0;    
}

void __ISR(_UART3_RX_VECTOR, ipl4AUTO) Handler_UART3_RX(void){
    //UART3Function();
    IFS4bits.U3RXIF = 0;    
}

void __ISR(_UART4_RX_VECTOR, ipl4AUTO) Handler_UART4_RX(void){
    //UART4Function();
    IFS5bits.U4RXIF = 0;    
}


void SetMotorCommandState(unsigned char id, unsigned char command){
    int module_num = id/2;
    if(id%2 == 0){
        // door
        DRINKOUT_INFO.module[module_num].Door.command_state = command;
    }else{
        // disk
        DRINKOUT_INFO.module[module_num].Disk.command_state = command;
    }
}