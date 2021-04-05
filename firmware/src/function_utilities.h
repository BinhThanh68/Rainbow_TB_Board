#ifndef _FUNCTION_UTILITIES_H
#define _FUNCTION_UTILITIES_H

void WriteUART2(unsigned char *data, int len);
void WriteUART3(unsigned char *data, int len);
void WriteUART4(unsigned char *data, int len);

unsigned char CalculateCheckSum(unsigned char status, unsigned char command, unsigned char data1, unsigned char data2);
void ICE_SetCommunicationMode(unsigned char mode, unsigned char unfocus_ignore);
void ICE_RequestIceOutWaterOut(float ice_time_sec, float water_time_sec);
void ICE_RequestClearStatusData();
void ICE_RequestAmbientTemperatureSetting();
void ICE_RequestAmbientTemperature();
void ICE_RequestEvaporatorCondensorTemperature();
void ICE_SetIceTimeout(int timeout_sec);
void ICE_RequestRebootController();
void ICE_SetAmbientTemperatureSetting(unsigned char low_temp, unsigned char high_temp);
void ICE_RequestStatus();


unsigned char CalculateBCC(unsigned char *data, int len);
void CUP_RequestStatus();
void CUP_RequestCupOut(int port_num, int output_num);
void CUP_RequestSoldOut();

void DRINKOUT_CheckConnection(unsigned char ID);
void DRINKOUT_SetProfile(unsigned char ID);
void DRINKOUT_SetProfile_Acele(unsigned char ID);
void DRINKOUT_SetPPGain(unsigned char ID);
void DRINKOUT_SetPDGain(unsigned char ID);
void DRINKOUT_TurnTorque(unsigned char ID, unsigned char ON_OFF);
//goal_position 1~3
void DRINKOUT_Go(unsigned char ID, unsigned char goal_pos_index);
void DRINKOUT_Go_by_value(unsigned char ID, int goal_pos);
void DRINKOUT_CheckMovingStatus(unsigned char ID);
void DRINKOUT_ReadPosition(unsigned char ID);
long DRINK_GetWeight();
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

void ICE_Valve_Set_Lock(unsigned char lock);
void ICE_Valve_Check_Connection();
void ICE_Valve_Set_Profile();
void ICE_Valve_Set_Torque();

#endif