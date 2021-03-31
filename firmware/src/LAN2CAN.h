
#ifndef _LAN2CAN_H
#define _LAN2CAN_H



#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

// Definitions -----------------------------------------------------------------
//#define _DEBUG_MONITOR_         // Enable debug message
// Error code definition
#define GPIF_NO_ERR              0
#define GPIF_ERR1                -1
#define GPIF_ERR2                -2
#define GPIF_ERR3                -3
#define GPIF_ERR4                -4
#define GPIF_ERR5                -5
#define GPIF_ERR6                -6
#define GPIF_ERR7                -7
#define GPIF_ERR8                -8
#define GPIF_ERR9                -9
#define GPIF_ERR10               -10    
    
#define SERVER_PORT             1977
#define CLIENT_PORT             2011
//#define __UDP__
#define __TCPIP__
#define DRINKOUT_ALL_DEVICES     0xFE

typedef union _UINT16_
{
    uint8_t UINT8[2];
    uint16_t UINT16;
} UINT16_UNION, *pUINT16_UNION;
    
typedef union _FLOAT_
{
    uint8_t UINT8[4];
    float FLOAT;
} FLOAT_UNION, *pFLOAT_UNION; 
    
typedef enum
{
    LAN2CAN_STATE_INIT = 0,
    LAN2CAN_STATE_TCPIP_WAIT_FOR_LINK,
    LAN2CAN_STATE_SETUP_TCP_SERVER,
    LAN2CAN_STATE_WAIT_FOR_CONNECTION,
    LAN2CAN_STATE_SERVING_CONNECTION,
    LAN2CAN_STATE_CLOSING_CONNECTION,
    
    LAN2CAN_STATE_ERROR
} LAN2CAN_STATES;


typedef enum
{
    MOTOR_IDLE = 0,
    MOTOR_WAIT_PING,
    MOTOR_WAIT_SET_PROFILE,
    MOTOR_WAIT_SET_PROFILE_ACELE,        
    MOTOR_WAIT_SET_PD_GAIN,
    MOTOR_WAIT_SET_PP_GAIN,        
    MOTOR_WAIT_ENABLE_TORQUE,        
    MOTOR_WAIT_GO_POS1,
    MOTOR_WAIT_GO_POS2,
    MOTOR_WAIT_GO_POS3,
    MOTOR_WAIT_DOOR_OPEN,
    MOTOR_WAIT_DOOR_CLOSE,
} MOTOR_COMMAND_STATE;

typedef struct
{
    unsigned char connection;
    unsigned char isProfileSet;
    unsigned char isProfileAceleSet;
    unsigned char isPDGainSet;
    unsigned char isPPGainSet;
    unsigned char isTorqueOn;
    int           presentPosition;
    MOTOR_COMMAND_STATE command_state;
} MOTOR_DISK_STATE;

typedef struct
{
    unsigned char connection;
    unsigned char isProfileSet;
    unsigned char isTorqueOn;
    int           presentPosition;
    MOTOR_COMMAND_STATE command_state;
} MOTOR_DOOR_STATE;

typedef struct
{
    unsigned char cupPresent;
    unsigned char ready;
    unsigned char currentPosition;
    unsigned char doorstate;
    MOTOR_DISK_STATE Disk;
    MOTOR_DOOR_STATE Door;
} DRINKOUT_ONE_MODULE;

typedef struct
{
    unsigned char connection;
    DRINKOUT_ONE_MODULE module_left;
    DRINKOUT_ONE_MODULE module_middle_left;
    DRINKOUT_ONE_MODULE module_middle_right;
    DRINKOUT_ONE_MODULE module_right;
} ST_DRINKOUT_DATA;

typedef enum
{
    MODULE_LEFT_DISK = 1,
    MODULE_LEFT_DOOR,
    MODULE_MIDDLE_LEFT_DISK,
    MODULE_MIDDLE_LEFT_DOOR,
    MODULE_MIDDLE_RIGHT_DISK,
    MODULE_MIDDLE_RIGHT_DOOR,
    MODULE_RIGHT_DISK,        
    MODULE_RIGHT_DOOR        
} DRINKOUT_MOTOR_ID;

typedef enum
{
    MOTOR_POSITION_OUTLET1 = 1,
    MOTOR_POSITION_OUTLET2,
    MOTOR_POSITION_OUTLET3,
    MOTOR_POSITION_DOOR_OPEN,
    MOTOR_POSITION_DOOR_CLOSE       
} DRINKOUT_MOTOR_POSITION;

typedef struct
{
    MOTOR_DOOR_STATE motor;
    unsigned char lock_state;
    unsigned char sensor;
} ST_ICE_VALVE_DATA;

// Client socket definition
typedef struct
{
    // IP Address
    IPV4_ADDR ipAddr;
    // Network handler
    TCPIP_NET_HANDLE netHandle;
    // Socket handler
    TCP_SOCKET socketHandle;
    // Flag to signify when the network link has dropped
	bool dropFlag;
    // LAN Rx data from Main Controller
    uint8_t msgFromClient[1500];
    // LAN Tx data to Main Controller 
    uint8_t msgToClient[1500];
    // LAN connection check timer
    SYS_TMR_HANDLE sysTmrLANCheckHandle;
    // LAN connection check count
    uint16_t checkCount;
} LAN_DATA;

// UART definition
typedef struct
{
    // UART handler
    DRV_HANDLE uartHandle;
    // UART Rx data
    uint8_t uartRxData[256];
    // UART Tx data
    uint8_t uartTxData[256];
    // Current received UART Rx data size
    uint8_t uartRxCount;
} UART_DATA;

// CAN definition
typedef struct
{
    // system timer for transmitting CAN data to the Main controller 
    SYS_TMR_HANDLE sysTmrCANOutHandle;
    // CAN driver handle
    DRV_HANDLE canHandle;
    // CAN Rx data
    CAN_RX_MSG_BUFFER *canRxMessage[256];
    // CAN Rx count
    uint8_t canRxCount;
} CAN_DATA;


typedef struct
{
    /* The application's current state */
    LAN2CAN_STATES state;
    LAN_DATA        lanData;
    CAN_DATA        canData;
    
    SYS_TMR_HANDLE  sysTmrUserHandle;
    SYS_TMR_HANDLE  sysTaskHandle;
    
} LAN2CAN_DATA;


typedef union{
    struct{
        unsigned    last_no_ice:1;          // ?????? ?? ???? ??
        unsigned    last_yes_ice:1;         // ?????? ?? ??? ??
        unsigned    comp_working:1;         // ?? ??? (1)
        unsigned    gear_motor_working:1;   // ???? ??? (1)
        unsigned    out_sol_detected:1;     // ?? SOL ?? (1)
        unsigned    cup_lever_detected:1;   // ??? ?? (1)
        unsigned    comm_mode:1;            // ???? (1)
        unsigned    full_ice:1;             // ?? (1)

        unsigned    err_code_1:1;           // ????1
        unsigned    err_code_2:1;           // ????2
        unsigned    err_code_3:1;           // ????3
        unsigned    err_code_4:1;           // ????4
        unsigned    cur_state:1;            // 1:???? ??  2: ????/????
        unsigned    rsvd:3;

        unsigned    rsvd2:8;
        unsigned    rsvd3:8;
    }b;
    unsigned char B[4];
}U_ICE_STAT;

typedef struct _ST_ICE_DATA_{
    unsigned char     connection;
    
    unsigned char     comm_mode;
    unsigned char     cup_ignore;

    unsigned char     setting_ambient_temperature_high;
    unsigned char     setting_ambient_temperature_low;

    unsigned char     ambient_temperature;

    unsigned char     evaporator_temperature;
    unsigned char     condensor_temperature;

    unsigned short     timeout_sec;

    unsigned char   last_ice_out_time;
    unsigned char   last_water_out_time;
    U_ICE_STAT  status;
}ST_ICE_DATA;



typedef struct _ST_CUP_DATA_{
    unsigned char     connection;
    unsigned char     operating_port;
    unsigned char     output_setting_num;
    unsigned char     output_current_num;
    unsigned char     op_stat;
    unsigned char     err_stat;
    unsigned char     soldout_stat;
}ST_CUP_DATA;

typedef struct _ST_LOADCELL_DATA{
    float             data;
    float             threshold;
    unsigned char     isEnabled;
    unsigned char     focusMode;
    unsigned char     start_making;
}ST_LOADCELL_DATA;

#define BARCODE_SIZE    20
extern unsigned char BARCODE_DATA[BARCODE_SIZE];
extern int BARCODE_LOCK;

extern unsigned char DIO[4];
extern unsigned char DIO_1[4];


void LAN2CAN_Initialize(void);
void LAN2CAN_Reset(void);
void LAN2CAN_Tasks(void);

int LAN2CAN_LANChipOn(void);
int LAN2CAN_LANChipOff(void);
int LAN2CAN_LANDataParsing(void);
int LAN2CAN_LANCheckConnection(void);

int LAN2CAN_CANReceiveCheckFromRCR(void);
int LAN2CAN_CANSendToMainController(void);
int LAN2CAN_CANClearBuffer(void);


void LAN2CAN_TaskFunction(void);
void SetMotorCommandState(unsigned char id, unsigned char command);
#endif /* _LAN2CAN_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNOEndRE-END

/*******************************************************************************
  of File
 */

