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



void DRINKOUT_Go(unsigned char ID, unsigned char goal_pos_index){
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
    //id = 1
    if(ID == MODULE_1_DISK){
        if(goal_pos_index == MOTOR_POSITION_OUTLET1){
            //go 840
            send_data[10] = 0x48; //data1
            send_data[11] = 0x03; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xC3; //CRC1
            send_data[15] = 0xA9; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET2){
            //go 2205
            send_data[10] = 0x9D; //data1
            send_data[11] = 0x08; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x78; //CRC1
            send_data[15] = 0x2D; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET3){
            //go 3575
            send_data[10] = 0xF7; //data1
            send_data[11] = 0x0D; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x2D; //CRC1
            send_data[15] = 0xA5; //CRC2
        } 
    }
    //id =2
    else if(ID == MODULE_1_DOOR){
        if(goal_pos_index == MOTOR_POSITION_DOOR_OPEN){
            //go open
            //test 3000
            send_data[10] = 0xB8; //data1
            send_data[11] = 0x0B; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x74; //CRC1
            send_data[15] = 0x49; //CRC2
        }
        else if(goal_pos_index == MOTOR_POSITION_DOOR_CLOSE){
            //go close
            //test 10730
            send_data[10] = 0xEA; //data1
            send_data[11] = 0x29; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xC4; //CRC1
            send_data[15] = 0x23; //CRC2
        }
    }
    //id=3
    else if(ID == MODULE_2_DISK){
        if(goal_pos_index == MOTOR_POSITION_OUTLET1){
            //go 1183
            send_data[10] = 0x9F; //data1
            send_data[11] = 0x04; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xA0; //CRC1
            send_data[15] = 0xC5; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET2){
            //go 2548
            send_data[10] = 0xF4; //data1
            send_data[11] = 0x09; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x55; //CRC1
            send_data[15] = 0x59; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET3){
            //go 3903
            send_data[10] = 0x3F; //data1
            send_data[11] = 0x0F; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x0C; //CRC1
            send_data[15] = 0x45; //CRC2
        } 
    }
    //id = 4
    else if(ID == MODULE_2_DOOR){
        if(goal_pos_index == MOTOR_POSITION_DOOR_OPEN){
            //go open
            //test 80
            send_data[10] = 0x50; //data1
            send_data[11] = 0x00; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xBB; //CRC1
            send_data[15] = 0xA9; //CRC2
        }
        else if(goal_pos_index == MOTOR_POSITION_DOOR_CLOSE){
            //go close
            //test 7830
            send_data[10] = 0x96; //data1
            send_data[11] = 0x1E; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x01; //CRC1
            send_data[15] = 0xD0; //CRC2
        }
    }
    
    //id=5
    else if(ID == MODULE_3_DISK){
        if(goal_pos_index == MOTOR_POSITION_OUTLET1){
            //go 1170
            send_data[10] = 0x92; //data1
            send_data[11] = 0x04; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xDE; //CRC1
            send_data[15] = 0x61; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET2){
            //go 2535
            send_data[10] = 0xE7; //data1
            send_data[11] = 0x09; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x2E; //CRC1
            send_data[15] = 0xE5; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET3){
            //go 3900
            send_data[10] = 0x3C; //data1
            send_data[11] = 0x0F; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x71; //CRC1
            send_data[15] = 0xB9; //CRC2
        } 
    }
    //id = 6
    else if(ID == MODULE_3_DOOR){
        if(goal_pos_index == MOTOR_POSITION_DOOR_OPEN){
            //go open
            //test 2650
            send_data[10] = 0x5A; //data1
            send_data[11] = 0x0A; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x18; //CRC1
            send_data[15] = 0x61; //CRC2
        }
        else if(goal_pos_index == MOTOR_POSITION_DOOR_CLOSE){
            //go close
            //test 10370
            send_data[10] = 0x82; //data1
            send_data[11] = 0x28; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x97; //CRC1
            send_data[15] = 0x03; //CRC2
        }
    }
    
    
    //id=7
    else if(ID == MODULE_4_DISK){
        if(goal_pos_index == MOTOR_POSITION_OUTLET1){
            //go 482
            send_data[10] = 0xE2; //data1
            send_data[11] = 0x01; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xA6; //CRC1
            send_data[15] = 0x61; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET2){
            //go 1847
            send_data[10] = 0x37; //data1
            send_data[11] = 0x07; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xFA; //CRC1
            send_data[15] = 0x65; //CRC2
        }else if(goal_pos_index == MOTOR_POSITION_OUTLET3){
            //go 3212
            send_data[10] = 0x8C; //data1
            send_data[11] = 0x0C; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x53; //CRC1
            send_data[15] = 0xB9; //CRC2
        } 
    }
    //id = 8
    else if(ID == MODULE_4_DOOR){
        if(goal_pos_index == MOTOR_POSITION_DOOR_OPEN){
            //go open
            //test 3140
            send_data[10] = 0x44; //data1
            send_data[11] = 0x0C; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0xB1; //CRC1
            send_data[15] = 0x39; //CRC2
        }
        else if(goal_pos_index == MOTOR_POSITION_DOOR_CLOSE){
            //go close
            //test 10850
            send_data[10] = 0x62; //data1
            send_data[11] = 0x2A; //data2
            send_data[12] = 0x00; //data3
            send_data[13] = 0x00; //data4
            send_data[14] = 0x46; //CRC1
            send_data[15] = 0x43; //CRC2
        }
    }
    
//    WriteUART4(send_data, 16);
    WriteUART3(send_data, 16);
    Nop();
}



void DRINKOUT_Go_by_value(unsigned char ID, int goal_pos){
    unsigned char send_data[16];
    unsigned short Check_bytes;
  
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
    send_data[10] = goal_pos&0xFF;      //data1
    send_data[11] = (goal_pos>>8)&0xFF; //data2
    send_data[12] = (goal_pos>>16)&0xFF;//data3
    send_data[13] = (goal_pos>>24)&0xFF;//data4
    //id = 1
    if(ID == MODULE_1_DISK){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_1_DOOR){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_2_DISK){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_2_DOOR){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_3_DISK){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_3_DOOR){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_4_DISK){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
    }else if(ID == MODULE_4_DOOR){
        Check_bytes   = update_crc(0,&send_data[0],14);
        send_data[14] = Check_bytes&0xFF; //CRC1
        send_data[15] = (Check_bytes>>8)&0xFF; //CRC2
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
        else{//for turning torque off
        }
    }
    else if(ID == 2) {
        if(ON_OFF == 1){
            send_data[11] = 0xEB; //CRC1
            send_data[12] = 0x65; //CRC2
        }
        else{
        }
    }
    else if(ID == 3) {
        if(ON_OFF == 1){
            send_data[11] = 0xF8; //CRC1
            send_data[12] = 0xE4; //CRC2
        }
        else{
        }
    }
    else if(ID == 4) {
        if(ON_OFF == 1){
            send_data[11] = 0x8B; //CRC1
            send_data[12] = 0x63; //CRC2
        }
        else{
        }
    }
    else if(ID == 5) {
        if(ON_OFF == 1){
            send_data[11] = 0x98; //CRC1
            send_data[12] = 0xE2; //CRC2
        }
        else{
        }
    }
    else if(ID == 6) {
        if(ON_OFF == 1){
            send_data[11] = 0xA8; //CRC1
            send_data[12] = 0xE1; //CRC2
        }
        else{
        }
    }
    else if(ID == 7) {
        if(ON_OFF == 1){
            send_data[11] = 0xBB; //CRC1
            send_data[12] = 0x60; //CRC2
        }
        else{
        }
    }
    else if(ID == 8) {
        if(ON_OFF == 1){
            send_data[11] = 0x4B; //CRC1
            send_data[12] = 0x6F; //CRC2
        }
        else{
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
    }else if(ID == 2){
        send_data[8] = 0x19; //CRC1
        send_data[9] = 0x72; //CRC2    
    }else if(ID == 3){
        send_data[8] = 0x1A; //CRC1
        send_data[9] = 0xE6; //CRC2    
    }else if(ID == 4){
        send_data[8] = 0x19; //CRC1
        send_data[9] = 0x0A; //CRC2    
    }else if(ID == 5){
        send_data[8] = 0x1A; //CRC1
        send_data[9] = 0x9E; //CRC2    
    }else if(ID == 6){
        send_data[8] = 0x1A; //CRC1
        send_data[9] = 0xA2; //CRC2    
    }else if(ID == 7){
        send_data[8] = 0x19; //CRC1
        send_data[9] = 0x36; //CRC2    
    }else if(ID == 8){
        send_data[8] = 0x19; //CRC1
        send_data[9] = 0xFA; //CRC2    
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
//    send_data[10] = 0x28; //data1  default:40
//    send_data[11] = 0x00; //data2
//    send_data[12] = 0x00; //data3
//    send_data[13] = 0x00; //data4
    if(ID == MODULE_1_DISK){
        send_data[10] = 0x28; //data1  default:40
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0xE8; //CRC1
        send_data[15] = 0x49; //CRC2
    }
    else if(ID == MODULE_1_DOOR){
        send_data[10] = 0x96; //data1  default:150
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0xE2; //CRC1
        send_data[15] = 0xF1; //CRC2
    }
    else if(ID == MODULE_2_DISK){
        send_data[10] = 0x28; //data1  default:40
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0xC3; //CRC1
        send_data[15] = 0x09; //CRC2
    }
    else if(ID == MODULE_2_DOOR){
        send_data[10] = 0x96; //data1  default:150
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0x9F; //CRC1
        send_data[15] = 0x31; //CRC2
    }
    else if(ID == MODULE_3_DISK){
        send_data[10] = 0x28; //data1  default:40
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0xBE; //CRC1
        send_data[15] = 0xC9; //CRC2
    }
    else if(ID == MODULE_3_DOOR){
        send_data[10] = 0x96; //data1  default:150
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0xB4; //CRC1
        send_data[15] = 0x71; //CRC2
    }
    else if(ID == MODULE_4_DISK){
        send_data[10] = 0x28; //data1  default:40
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0x95; //CRC1
        send_data[15] = 0x89; //CRC2
    }
    else if(ID == MODULE_4_DOOR){
        send_data[10] = 0x96; //data1  default:150
        send_data[11] = 0x00; //data2
        send_data[12] = 0x00; //data3
        send_data[13] = 0x00; //data4
        send_data[14] = 0x60; //CRC1
        send_data[15] = 0x31; //CRC2
    }
    else if(ID == DRINKOUT_ALL_DEVICES){
        send_data[14] = 0xE2; //CRC1
        send_data[15] = 0x79; //CRC2
    }
      
//    WriteUART4(send_data, 16);
    WriteUART3(send_data, 16);
}


void DRINKOUT_SetProfile_Acele(unsigned char ID){
    unsigned char send_data[16];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x09; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x6C; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0x0A; //data1  default:10
    send_data[11] = 0x00; //data2
    send_data[12] = 0x00; //data3
    send_data[13] = 0x00; //data4
    if(ID == MODULE_1_DISK){
        send_data[14] = 0xFA; //CRC1
        send_data[15] = 0x41; //CRC2
    }
    else if(ID == MODULE_2_DISK){
//        send_data[14] = 0xC3; //CRC1
//        send_data[15] = 0x09; //CRC2
    }
    else if(ID == MODULE_3_DISK){
//        send_data[14] = 0xBE; //CRC1
//        send_data[15] = 0xC9; //CRC2
    }
    else if(ID == MODULE_4_DISK){
//        send_data[14] = 0x95; //CRC1
//        send_data[15] = 0x89; //CRC2
    }
   
//    WriteUART4(send_data, 16);
    WriteUART3(send_data, 16);
}

void DRINKOUT_SetPPGain(unsigned char ID){
    unsigned char send_data[16];
  
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x07; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x54; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0xB8; //data1  default:3000
    send_data[11] = 0x0B; //data2
    if(ID == MODULE_1_DISK){
        send_data[12] = 0x72; //CRC1
        send_data[13] = 0x5D; //CRC2
    } 
    else if(ID == MODULE_2_DISK){
        send_data[12] = 0x7E; //CRC1
        send_data[13] = 0x7D; //CRC2
    }
    else if(ID == MODULE_3_DISK){
        send_data[12] = 0x6A; //CRC1
        send_data[13] = 0x1D; //CRC2
    }
    else if(ID == MODULE_4_DISK){
        send_data[12] = 0x66; //CRC1
        send_data[13] = 0x3D; //CRC2
    }
//    WriteUART4(send_data, 14);
    WriteUART3(send_data, 14);
}


void DRINKOUT_SetPDGain(unsigned char ID){
    unsigned char send_data[16];
    
    send_data[0] = 0xFF; //header1
    send_data[1] = 0xFF; //header2
    send_data[2] = 0xFD; //header3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x07; //length1
    send_data[6] = 0x00; //length2
    send_data[7] = 0x03; //instruction    
    send_data[8] = 0x50; //address1
    send_data[9] = 0x00; //address1
    send_data[10] = 0xB8; //data1  default:3000
    send_data[11] = 0x0B; //data2
    if(ID == MODULE_1_DISK){
        send_data[12] = 0x71; //CRC1
        send_data[13] = 0x8D; //CRC2
    }   
    else if(ID == MODULE_2_DISK){
        send_data[12] = 0x7D; //CRC1
        send_data[13] = 0xAD; //CRC2
    }
    else if(ID == MODULE_3_DISK){
        send_data[12] = 0x69; //CRC1
        send_data[13] = 0xCD; //CRC2
    }
    else if(ID == MODULE_4_DISK){
        send_data[12] = 0x65; //CRC1
        send_data[13] = 0xED; //CRC2
    } 
//    WriteUART4(send_data, 14);
    WriteUART3(send_data, 14);
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
   
    if(ID == MODULE_1_DISK){
        send_data[12] = 0x3C; //CRC1
        send_data[13] = 0xDB; //CRC2
    }
    else if(ID == MODULE_1_DOOR){
        send_data[12] = 0x36; //CRC1
        send_data[13] = 0xEB; //CRC2
    }
    else if(ID == MODULE_2_DISK){
        send_data[12] = 0x30; //CRC1
        send_data[13] = 0xFB; //CRC2
    }
    else if(ID == MODULE_2_DOOR){
        send_data[12] = 0x22; //CRC1
        send_data[13] = 0x8B; //CRC2
    }
    else if(ID == MODULE_3_DISK){
        send_data[12] = 0x24; //CRC1
        send_data[13] = 0x9B; //CRC2
    }
    else if(ID == MODULE_3_DOOR){
        send_data[12] = 0x2E; //CRC1
        send_data[13] = 0xAB; //CRC2
    }
    else if(ID == MODULE_4_DISK){
        send_data[12] = 0x28; //CRC1
        send_data[13] = 0xBB; //CRC2
    }
    else if(ID == MODULE_4_DOOR){
        send_data[12] = 0x0A; //CRC1
        send_data[13] = 0x4B; //CRC2
    }

//    WriteUART4(send_data, 14);
    WriteUART3(send_data, 14);
}


void DRINKOUT_CheckMovingStatus(unsigned char ID){
    unsigned char send_data[14];
    send_data[0] = 0xFF; //header 1
    send_data[1] = 0xFF; //header 2
    send_data[2] = 0xFD; //header 3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x07; //length
    send_data[6] = 0x00; //length 2
    send_data[7] = 0x02; //instruction
    send_data[8] = 0x7A; //address 1
    send_data[9] = 0x00; //address 2
    send_data[10] = 0x01; //address 1
    send_data[11] = 0x00; //address 2
    if(ID == 1){
        send_data[12] = 0x36; //CRC1
        send_data[13] = 0x93; //CRC2    
    }else if(ID == 2){
        send_data[12] = 0x3C; //CRC1
        send_data[13] = 0xA3; //CRC2    
    }else if(ID == 3){
        send_data[12] = 0x3A; //CRC1
        send_data[13] = 0xB3; //CRC2    
    }else if(ID == 4){
        send_data[12] = 0x28; //CRC1
        send_data[13] = 0xC3; //CRC2    
    }else if(ID == 5){
        send_data[12] = 0x2E; //CRC1
        send_data[13] = 0xD3; //CRC2    
    }else if(ID == 6){
        send_data[12] = 0x24; //CRC1
        send_data[13] = 0xE3; //CRC2    
    }else if(ID == 7){
        send_data[12] = 0x22; //CRC1
        send_data[13] = 0xF3; //CRC2    
    }else if(ID == 8){
        send_data[12] = 0x00; //CRC1
        send_data[13] = 0x03; //CRC2    
    }
    
//    WriteUART4(send_data, 14);
    WriteUART3(send_data, 14);
}

void DRINKOUT_ReadPosition(unsigned char ID){
    unsigned char send_data[14];
    send_data[0] = 0xFF; //header 1
    send_data[1] = 0xFF; //header 2
    send_data[2] = 0xFD; //header 3
    send_data[3] = 0x00; //reserved
    send_data[4] = ID;   //ID
    send_data[5] = 0x07; //length
    send_data[6] = 0x00; //length 2
    send_data[7] = 0x02; //instruction
    send_data[8] = 0x84; //address 1
    send_data[9] = 0x00; //address 2
    send_data[10] = 0x04; //address 1
    send_data[11] = 0x00; //address 2
    if(ID == 1){ 
        send_data[12] = 0x1D; //CRC1
        send_data[13] = 0x15; //CRC2    
    }else if(ID == 2){
        send_data[12] = 0x17; //CRC1
        send_data[13] = 0x25; //CRC2    
    }else if(ID == 3){
        send_data[12] = 0x11; //CRC1
        send_data[13] = 0x35; //CRC2    
    }else if(ID == 4){
        send_data[12] = 0x03; //CRC1
        send_data[13] = 0x45; //CRC2    
    }else if(ID == 5){
        send_data[12] = 0x05; //CRC1
        send_data[13] = 0x55; //CRC2    
    }else if(ID == 6){
        send_data[12] = 0x0F; //CRC1
        send_data[13] = 0x65; //CRC2    
    }else if(ID == 7){
        send_data[12] = 0x09; //CRC1
        send_data[13] = 0x75; //CRC2    
    }else if(ID == 8){
        send_data[12] = 0x2B; //CRC1
        send_data[13] = 0x85; //CRC2    
    } 
//    WriteUART4(send_data, 14);
    WriteUART3(send_data, 14);
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



void RemoteController_LED_Operation(unsigned char led, unsigned char state){
    if(led == 1){
        PORTBbits.RB2 = state;
    }else if(led == 2){
       PORTBbits.RB4 = state;
    }else if(led == 3){
        PORTBbits.RB6 = state;
    }
}

void DRINKOUT_CloseAll(){
    
    DRINKOUT_Go(MODULE_1_DOOR, MOTOR_POSITION_DOOR_CLOSE);
    HX711_Delay_ms(200);
    DRINKOUT_Go(MODULE_2_DOOR, MOTOR_POSITION_DOOR_CLOSE);
    HX711_Delay_ms(200);
    DRINKOUT_Go(MODULE_3_DOOR, MOTOR_POSITION_DOOR_CLOSE);
    HX711_Delay_ms(200);
    DRINKOUT_Go(MODULE_4_DOOR, MOTOR_POSITION_DOOR_CLOSE);
}

void DRINKOUT_OpenAll(){
    DRINKOUT_Go(MODULE_1_DOOR, MOTOR_POSITION_DOOR_OPEN);
    HX711_Delay_ms(200);
    DRINKOUT_Go(MODULE_2_DOOR, MOTOR_POSITION_DOOR_OPEN);
    HX711_Delay_ms(200);
    DRINKOUT_Go(MODULE_3_DOOR, MOTOR_POSITION_DOOR_OPEN);
    HX711_Delay_ms(200);
    DRINKOUT_Go(MODULE_4_DOOR, MOTOR_POSITION_DOOR_OPEN);
}


unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}