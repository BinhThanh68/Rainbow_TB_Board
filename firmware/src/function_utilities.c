#include "config_cocoa_general.h"
#include "LAN2CAN.h"
#include <math.h>
#include "HX711.h"

void WriteUART2(unsigned char *data, int len){
    int i=0;
    for(i=0; i<len; i++){
        while(U2STAbits.UTXBF==1);
            U2TXREG =data[i];
    }
}

void WriteUART3(unsigned char *data, int len){
    int i=0;
    for(i=0; i<len; i++){
        while(U3STAbits.UTXBF==1);
            U3TXREG =data[i];
    }
}

void WriteUART4(unsigned char *data, int len){
    int i=0;
    for(i=0; i<len; i++){
        while(U4STAbits.UTXBF==1);
            U4TXREG =data[i];
    }
}


// ICE DISPENSER ==============================================================================================
// ICE DISPENSER ==============================================================================================
// ICE DISPENSER ==============================================================================================
unsigned char CalculateCheckSum(unsigned char status, unsigned char command, unsigned char data1, unsigned char data2){
    return status^command^data1^data2;
}

void ICE_SetCommunicationMode(unsigned char mode, unsigned char unfocus_ignore){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xA0;
    send_data[3] = mode;
    send_data[4] = unfocus_ignore;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;
    
    WriteUART3(send_data, 7);
}
void ICE_RequestIceOutWaterOut(float ice_time_sec, float water_time_sec){
    int val_H, val_L;
    unsigned char ice, water;
    if(ice_time_sec > 10.0f){
        ice_time_sec = 9.9f;
    }
    val_H = (int)(floor(ice_time_sec));
    val_L = (int)(floor((ice_time_sec-val_H)*10.0f));
    ice = (unsigned char)(val_H*16+val_L);

    if(water_time_sec > 10.0f){
        water_time_sec = 9.9f;
    }
    val_H = (int)(floor(water_time_sec));
    val_L = (int)(floor((water_time_sec-val_H)*10.0f));
    water = (unsigned char)(val_H*16+val_L);

    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xB0;
    send_data[3] = ice;
    send_data[4] = water;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}
void ICE_RequestClearStatusData(){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xC0;
    send_data[3] = 0x01;
    send_data[4] = 0x00;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_RequestAmbientTemperatureSetting(){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xC1;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_RequestAmbientTemperature(){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xC2;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_RequestEvaporatorCondensorTemperature(){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xC3;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_SetIceTimeout(int timeout_sec){
    if(timeout_sec > 600){
        timeout_sec = 599;
    }
    unsigned char min,sec;
    min = (unsigned char)(timeout_sec/60);
    sec = (unsigned char)(timeout_sec-min*60);

    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xD0;
    send_data[3] = min;
    send_data[4] = sec;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_RequestRebootController(){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xE0;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_SetAmbientTemperatureSetting(unsigned char low_temp, unsigned char high_temp){
    // low_temp : 0~10
    // high_temp : 40~70

    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xE1;
    send_data[3] = low_temp;
    send_data[4] = high_temp;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}

void ICE_RequestStatus(){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0xCF;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = CalculateCheckSum(send_data[1], send_data[2], send_data[3], send_data[4]);
    send_data[6] = 0x03;

    WriteUART3(send_data, 7);
}



// CUP DISPENSER ==============================================================================================
// CUP DISPENSER ==============================================================================================
// CUP DISPENSER ==============================================================================================
unsigned char CalculateBCC(unsigned char *data, int len){
    short temp_sum = 0;
    int i = 0;
    for(i=0; i<len; i++){
        temp_sum = temp_sum + (short)(data[i]);
    }
    return temp_sum&0xFF;
}

void CUP_RequestStatus(){
    unsigned char send_data[5];
    send_data[0] = 0x02;
    send_data[1] = 0x01;
    send_data[2] = 0x40;
    send_data[3] = 0x03;
    send_data[4] = 0x44;

    WriteUART2(send_data, 5);
}

void CUP_RequestCupOut(int port_num, int output_num){
    unsigned char send_data[7];
    send_data[0] = 0x02;
    send_data[1] = 0x03; 
    send_data[2] = 0x41;
    send_data[3] = port_num; // 1~4
    send_data[4] = 1;//output_num;
    send_data[5] = 0x03;
    send_data[6] = CalculateBCC(&(send_data[1]), 5);
    
    WriteUART2(send_data, 7);
}

void CUP_RequestSoldOut(){
    unsigned char send_data[6];
    send_data[0] = 0x02;
    send_data[1] = 0x02; 
    send_data[2] = 0x42;
    send_data[3] = 0x00;
    send_data[4] = 0x03;
    send_data[5] = CalculateBCC(&(send_data[1]), 4);    
    WriteUART2(send_data, 6);
}



void DRINKOUT_Go(unsigned char ID, short goal_pos){
    unsigned char send_data[16];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x09; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction
    send_data[8] = 0x74; //address
    send_data[9] = 0x00; //address
    if(ID == MODULE_LEFT_DISK){
        switch(goal_pos){
        case MOTOR_POSITION_OUTLET1:{
            //go 825
            send_data[10] = 0x39; //data1
            send_data[11] = 0x03; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xD7; //CRC1
            send_data[15] = 0x7D; //CRC2
        }
            break;
        case MOTOR_POSITION_OUTLET2:{
            //go 2187
            send_data[10] = 0x9D; //data1
            send_data[11] = 0x08; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x78; //CRC1
            send_data[15] = 0x2D; //CRC2
        }
            break;
        case MOTOR_POSITION_OUTLET3:{
            //go 3552
            send_data[10] = 0xEE; //data1
            send_data[11] = 0x0D; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x2B; //CRC1
            send_data[15] = 0x51; //CRC2
        }
            break;
        default:
            break;   
        }
    }
    else if(ID == MODULE_LEFT_DOOR){
        if(goal_pos == MOTOR_POSITION_DOOR_OPEN){
            //go open
            //test 2930
            send_data[10] = 0x72; //data1
            send_data[11] = 0x0B; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x56; //CRC1
            send_data[15] = 0xC1; //CRC2
        }
        else if(goal_pos == MOTOR_POSITION_DOOR_CLOSE){
            //go close
            //test 4000
            send_data[10] = 0xEA; //data1
            send_data[11] = 0x29; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xC4; //CRC1
            send_data[15] = 0x23; //CRC2
        }
    }
//    WriteUART4(send_data, 16);
    WriteUART3(send_data, 16);
}

void DRINKOUT_TurnTorque(unsigned char ID, unsigned char ON_OFF){
    unsigned char send_data[13];
    send_data[0] = 0xFF; //header 1
    send_data[1] = 0xFF; //header 2
    send_data[2] = 0xFD; //header 3
    send_data[3] = 0x00; //reserved
    
    send_data[4] = ID;   //ID
    send_data[5] = 0x06; //length1
    send_data[6] = 0x00; //length2
   
    send_data[7] = 0x03; //instruction
    send_data[8] = 0x40; //address1
    send_data[9] = 0x00; //address2
    send_data[10] = ON_OFF;//data
    
    if(ID == 1){
        if(ON_OFF == 1){
            send_data[11] = 0xDB; //CRC1
            send_data[12] = 0x66; //CRC2
        }
        else{
            send_data[11] = 0xDE; //CRC1
            send_data[12] = 0xE6; //CRC2
        }
    }
    else if(ID == 2) {
        if(ON_OFF == 1){
            send_data[11] = 0xEB; //CRC1
            send_data[12] = 0x65; //CRC2
        }
        else{
            send_data[11] = 0xEE; //CRC1
            send_data[12] = 0xE5; //CRC2
        }
    }
  
//    WriteUART4(send_data, 13);
    WriteUART3(send_data, 13);
}

void DRINKOUT_CheckConnection(unsigned char ID){
    unsigned char send_data[10];
    send_data[0] = 0xFF; //header 1
    send_data[1] = 0xFF; //header 2
    send_data[2] = 0xFD; //header 3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x03; //length
    send_data[6] = 0x00; //length 2
    send_data[7] = 0x01; //instruction
    if(ID == 1){
        send_data[8] = 0x19; //CRC1
        send_data[9] = 0x4E; //CRC2    
    }
    else if(ID == 0x02){
        send_data[8] = 0x19; //CRC1
        send_data[9] = 0x72; //CRC2    
    }
    
//    WriteUART4(send_data, 10);
    WriteUART3(send_data, 10);
}

void DRINKOUT_SetProfile(unsigned char ID){
    unsigned char send_data[16];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x09; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x70; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0x28; //data1  default:40
    send_data[11] = 0x00; //data2
    send_data[12] = 0x00; //data3
    send_data[13] = 0x00; //data4
    if(ID == MODULE_LEFT_DISK){
        send_data[14] = 0xE8; //CRC1
        send_data[15] = 0x49; //CRC2
    }
    else if(ID == MODULE_LEFT_DOOR){
        send_data[14] = 0xD4; //CRC1
        send_data[15] = 0xE9; //CRC2
    }
    else if(ID == MODULE_MIDDLE_LEFT_DISK){
        send_data[14] = 0xC3; //CRC1
        send_data[15] = 0x09; //CRC2
    }
    else if(ID == MODULE_MIDDLE_LEFT_DOOR){
        send_data[14] = 0xA9; //CRC1
        send_data[15] = 0x29; //CRC2
    }
    else if(ID == MODULE_MIDDLE_RIGHT_DISK){
        send_data[14] = 0xBE; //CRC1
        send_data[15] = 0xC9; //CRC2
    }
    else if(ID == MODULE_MIDDLE_RIGHT_DOOR){
        send_data[14] = 0x82; //CRC1
        send_data[15] = 0x69; //CRC2
    }
    else if(ID == MODULE_RIGHT_DISK){
        send_data[14] = 0x95; //CRC1
        send_data[15] = 0x89; //CRC2
    }
    else if(ID == MODULE_RIGHT_DOOR){
        send_data[14] = 0x56; //CRC1
        send_data[15] = 0x29; //CRC2
    }
    else if(ID == DRINKOUT_ALL_DEVICES){
        send_data[14] = 0xE2; //CRC1
        send_data[15] = 0x79; //CRC2
    }
      
//    WriteUART4(send_data, 16);
    WriteUART3(send_data, 16);
}


void DRINKOUT_ReadTorqueStatus(unsigned char ID){
    unsigned char send_data[14];
    
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x07; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x02; //instruction    
    send_data[8] = 0x40; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0x01; //data1  default:40
    send_data[11] = 0x00; //data2
   
    if(ID == MODULE_LEFT_DISK){
        send_data[12] = 0x3C; //CRC1
        send_data[13] = 0xDB; //CRC2
    }
    else if(ID == MODULE_LEFT_DOOR){
        send_data[12] = 0x36; //CRC1
        send_data[13] = 0xEB; //CRC2
    }
    else if(ID == MODULE_MIDDLE_LEFT_DISK){
        send_data[12] = 0x30; //CRC1
        send_data[13] = 0xFB; //CRC2
    }
    else if(ID == MODULE_MIDDLE_LEFT_DOOR){
        send_data[12] = 0x22; //CRC1
        send_data[13] = 0x8B; //CRC2
    }
    else if(ID == MODULE_MIDDLE_RIGHT_DISK){
        send_data[12] = 0x24; //CRC1
        send_data[13] = 0x9B; //CRC2
    }
    else if(ID == MODULE_MIDDLE_RIGHT_DOOR){
        send_data[12] = 0x2E; //CRC1
        send_data[13] = 0xAB; //CRC2
    }
    else if(ID == MODULE_RIGHT_DISK){
        send_data[12] = 0x28; //CRC1
        send_data[13] = 0xBB; //CRC2
    }
    else if(ID == MODULE_RIGHT_DOOR){
        send_data[12] = 0x0A; //CRC1
        send_data[13] = 0x4B; //CRC2
    }

    WriteUART4(send_data, 14);
//    WriteUART2(send_data, 14);
}

long DRINK_GetWeight(){
    return HX711_get_units(1);
}


void ICE_Valve_Set_Lock(unsigned char lock){
    unsigned char send_data[16];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = 0x09;   //ID
    send_data[5] = 0x09; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x74; //address1
    send_data[9] = 0x00; //address1
    
    if(lock){
        send_data[10] = 0xB0; //data1 1200
        send_data[11] = 0x04; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0x2E; //CRC1
        send_data[15] = 0x49; //CRC2
    }else{
        send_data[10] = 0xAA; //data1  170
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0x7B; //CRC1
        send_data[15] = 0x01; //CRC2
    }
      
    WriteUART4(send_data, 16);
}

void ICE_Valve_Check_Connection(){
    unsigned char send_data[10];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = 0x09; //ID
    send_data[5] = 0x03; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x01; //instruction    
    send_data[8] = 0x1A; //address1
    send_data[9] = 0x6E; //address1
   
    WriteUART4(send_data, 10);
}

void ICE_Valve_Set_Profile(){
    unsigned char send_data[13];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = 0x09; //ID
    send_data[5] = 0x06; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x40; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0x01; //data
    send_data[11] = 0x58; //CRC1
    send_data[12] = 0xEE; //CRC2
      
    WriteUART4(send_data, 13);
}


void ICE_Valve_Set_Torque(){
    unsigned char send_data[13];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = 0x09; //ID
    send_data[5] = 0x06; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x40; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0x01; //data
    send_data[11] = 0x58; //CRC1
    send_data[12] = 0xEE; //CRC2
      
    WriteUART4(send_data, 13);
}