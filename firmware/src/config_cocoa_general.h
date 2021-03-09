#ifndef CONFIG_COCOA_GENERAL_H
#define CONFIG_COCOA_GENERAL_H

#define             LATEST_VERSION      190519
#define             MAIN_BOARD_VERSION      30
// 30 : 2020 - 01 - 13




//#define _IS_VERSION_40_
#define _IS_VERSION_42_

//#define _WILL_USE_AUTO_OFF_
/*******************************************************************************
 * COMMON INCLUDE
 ******************************************************************************/
#include <proc/p32mz2048efh064.h>
#include <xc.h>
#include <sys/kmem.h>
#include <sys/attribs.h>
#include <sys/types.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>

#include <math.h>

/*******************************************************************************
 * COMMON VARIABLES
 ******************************************************************************/
extern double              READ_POWER_IN_V;
extern double              READ_POWER_OUT_V;
extern double              READ_POWER_24_V;
extern double              READ_POWER_IN_A;
extern double              READ_TEMPER_A_C;
extern double              READ_TEMPER_B_C;

extern int                 READ_EMG_SW_SELECTION;
extern int                 READ_MC_SW_SELECTION;

extern double              READ_ANALOG_IN_A_V;
extern double              READ_ANALOG_IN_B_V;
extern double              READ_ANALOG_IN_C_V;
extern double              READ_ANALOG_IN_D_V;

extern unsigned char       WRITE_DOUT_GPIO[16];
extern unsigned char       READ_DIN_GPIO[16];
/*******************************************************************************
 * COMMON CONSTANTS
 ******************************************************************************/
#define             SYSFREQ             200000000
#define             PBCLK               100000000
#define             TMR_FREQ            1000

#define             FALSE               0
#define             TRUE                1
#define             OUTPUT              0
#define             INPUT               1
#define             LOW                 0
#define             HIGH                1
#define             DIGITAL             0
#define             ANALOG              1
#define             TURN_OFF            0
#define             TURN_ON             1

typedef double      REAL8;
static double       PI      = 3.141592653589793;
static double       D2R     = 0.017453292519943;
static double       R2D     = 57.295779513082323;

/*******************************************************************************
 * BOARD PIN SETTING
 ******************************************************************************/
#define             ANSEL_ALL               ANSELB = ANSELE = ANSELG
#define             TRIS_ALL                TRISB = TRISC = TRISD = TRISE = TRISF = TRISG
// Primary Osc
#define             TRIS_POSC               TRISCbits.TRISC12


// UART
#define             TRIS_UART_2_TX          TRISDbits.TRISD0
#define             PPS_UART_2_TX           RPD0Rbits.RPD0R = 0b0010




/*******************************************************************************
 * HEADER INCLUDE
 ******************************************************************************/
#include "function_initialization.h"
#include "function_utilities.h"

#endif