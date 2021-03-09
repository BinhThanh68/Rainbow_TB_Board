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


void DRINKOUT_SetProfile(unsigned char ID);
void DRINKOUT_CheckConnection(unsigned char ID);
//goal_position 1~3
void DRINKOUT_Go(unsigned char ID, short goal_pos);
void DRINKOUT_TurnTorque(unsigned char ID, unsigned char ON_OFF);

long DRINK_GetWeight();

#endif