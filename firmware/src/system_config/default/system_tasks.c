/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all polled MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/



void SYS_Tasks ( void )
{
    /* Maintain system services */
    /* SYS_TMR Device layer tasks routine */ 
    SYS_TMR_Tasks(sysObj.sysTmr);

    /* Maintain Device Drivers */
    DRV_MIIM_Tasks (sysObj.drvMiim);

    /* Maintain Middle ware & Other Libraries */
    /* Maintain the TCP/IP Stack*/
    TCPIP_STACK_Task(sysObj.tcpip);

    /* Maintain the application's state machine. */
    LAN2CAN_Tasks();
    
//    bool isTank1Empty = TEA_DISP_RequestStatus(1);
//    if(isTank1Empty){
//    bool isTank2Empty = TEA_DISP_RequestStatus(2);
//    }
    
    //read bar code from scanner
    //Read_Scanner_Data();
//    DOOR_TestLED();
//    int i,j;
//      bool test = DRINK_OUT_RequestCupStatus();
//      if(test){
//          DOOR_TestLED(true);
////          LATBbits.LATB10 = 1;
//          for(i=0; i<800000; i++);
//       }
//       else{
//         DOOR_TestLED(false);
////         LATBbits.LATB10 = 0;
//         for(j=0; j<800000; j++);
//        }

//    //test motor on off led
//    char send_data[13];
//    char send_data2[13];
//    
//    send_data[0]= 0xFF;
//    send_data[1]= 0xFF;
//    send_data[2]= 0xFD;
//    send_data[3]= 0x00;
//    send_data[4]= 0x01;
//    send_data[5]= 0x06;
//    send_data[6]= 0x00;
//    send_data[7]= 0x03;
//    send_data[8]= 0x41;
//    send_data[9]= 0x00;
//    send_data[10]= 0x01;
//    send_data[11]= 0xCC;
//    send_data[12]= 0xE6;
//    
//    send_data2[0]= 0xFF;
//    send_data2[1]= 0xFF;
//    send_data2[2]= 0xFD;
//    send_data2[3]= 0x00;
//    send_data2[4]= 0x01;
//    send_data2[5]= 0x06;
//    send_data2[6]= 0x00;
//    send_data2[7]= 0x03;
//    send_data2[8]= 0x41;
//    send_data2[9]= 0x00;
//    send_data2[10]= 0x00;
//    send_data2[11]= 0xC9;
//    send_data2[12]= 0x66;
//    
//
//    WriteUART2(send_data, 13);
//    HX711_Delay_ms(400);
//    WriteUART2(send_data2, 13);
//    HX711_Delay_ms(400);
   
        //test motor rotate
    
    //test reading sensor
//    if(PORTBbits.RB2==1){
//        Nop();
//        
//    }
//    else{
//        Nop();
//    }
    
//    DRINKOUT_SetProfile(MODULE_LEFT_DISK);
//    HX711_Delay_ms(200);
//    DRINKOUT_TurnTorque(MODULE_LEFT_DISK, 1);
//    HX711_Delay_ms(200);
//    int i;
//    for(i=1;i<4;i++){
//    DRINKOUT_Go(MODULE_LEFT_DISK, i);
//    HX711_Delay_ms(1000);
//    }

}


/*******************************************************************************
 End of File
 */
