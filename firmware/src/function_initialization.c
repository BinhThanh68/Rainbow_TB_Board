#include "config_cocoa_general.h"

void INIT_PINS(void){
    // Init all AN pin as digital
    ANSEL_ALL = DIGITAL;
    
    TRISCbits.TRISC12 = 1;  // OSC
    
    TRISCbits.TRISC13 = 0;  // UART1 TX     not use --              RX3
    TRISCbits.TRISC14 = 1;  // UART1 RX     not use -- TX3
    
    
    TRISDbits.TRISD5 = 0;   // UART2 TX     TX2     -- TX2 TX6      RX4 RX6
    TRISDbits.TRISD4 = 1;   // UART2 RX     RX2     -- TX4 TX6      RX2
    RPD5Rbits.RPD5R = 0b0010;   // U2TX(D5)
    U2RXRbits.U2RXR = 0b0100;   // U2RX(D4)
    
    
    TRISDbits.TRISD2 = 0;   // UART3 TX     TX3     -- TX3          RX1
    TRISDbits.TRISD3 = 1;   // UART3 RX     RX3     -- TX1 TX5      RX3
    RPD2Rbits.RPD2R = 0b0001;   // U3TX(D2)
    U3RXRbits.U3RXR = 0b0000;   // U3RX(D3)
            
    
    TRISDbits.TRISD9 = 0;   // UART4 TX     TX4     -- TX4 Tx6      RX2 
    TRISDbits.TRISD0 = 1;   // UART4 RX     RX4     -- TX2 TX6      RX4 RX6
    RPD9Rbits.RPD9R = 0b0010;   // U4TX(D9)
    U4RXRbits.U4RXR = 0b0011;   // U4RX(D0)
    
    
    
    TRISFbits.TRISF0 = 1;   // CAN RX
    TRISFbits.TRISF1 = 0;   // CAN TX
    
    
    TRISBbits.TRISB9 = 0;   // Debug LED A
    TRISBbits.TRISB10 = 0;  // Debug LED B
    
    //test for hx711
    TRISBbits.TRISB2 = 0;   // Digital In 0
    TRISBbits.TRISB3 = 1;   // Digital In 1
    //output for testing load cell
    TRISBbits.TRISB4 = 0;   // Digital In 2 
    TRISBbits.TRISB5 = 1;   // Digital In 3
    
    TRISBbits.TRISB10 = 1;  // ID Selection
    TRISBbits.TRISB11 = 1;  // ID Selection
    
    TRISBbits.TRISB13 = 0;  // Ethernet Reset
    
   
    
    //timer B (2 and 3)
    T2CONbits.ON = 0;
    T3CONbits.ON = 0;
    T2CONbits.TCKPS = 0;
    T2CONbits.T32 = 1;
    
    TMR2 = 0x0;
    TMR3 = 0x0;
    T2CONSET = 0x8000;
       
}

void INIT_UART_1(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode){
    // baudRate = XX (bps)
    
    // stop_bit = 1 or 2
    
    // parity_mode 0 = none
    //             1 = odd
    //             2 = even                   
    
    // clear uart
    U1MODE = 0;
    
    // setting
    U1MODEbits.UEN = 0;             // RXTX mode
    U1STAbits.UTXISEL0=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
	U1STAbits.UTXISEL1=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
    U1STAbits.URXISEL = 0;          // Rx interrupt mode selection
    U1STAbits.URXISEL0 = 0;
    U1STAbits.URXISEL1 = 0;
    //U1MODEbits.RTSMD = 0;
    
    
    if(parity_mode == 0){
        // 8bit+none
        U1MODEbits.PDSEL = 0b00;
    }else if(parity_mode == 1){
        // 8bit+odd
        U1MODEbits.PDSEL = 0b10;
    }else if(parity_mode == 2){
        // 8bit+even
        U1MODEbits.PDSEL = 0b01;
    }
    U1MODEbits.STSEL = stop_bit - 1;
    U1MODEbits.BRGH = 0;            // standard mode
    
    if(baudRate > 230400){
        U1MODEbits.BRGH = 1;
        U1BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/4. - 1.);
    }else{
        U1MODEbits.BRGH = 0;
        U1BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/16. - 1.);
    }

    U1STAbits.UTXEN = 1;            // enable TX
    U1STAbits.URXEN = 0;            // enalbe RX
    U1MODEbits.ON = 1;              // enable U1  
}

void INIT_UART_2(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode){
    // baudRate = XX (bps)
    
    // stop_bit = 1 or 2
    
    // parity_mode 0 = none
    //             1 = odd
    //             2 = even                   
    
    // clear uart
    U2MODE = 0;
    
    // setting
    U2MODEbits.UEN = 0;             // RXTX mode
    U2STAbits.UTXISEL0=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
	U2STAbits.UTXISEL1=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
    U2STAbits.URXISEL = 0;          // Rx interrupt mode selection
    U2STAbits.URXISEL0 = 0;
    U2STAbits.URXISEL1 = 0;
    //U2MODEbits.RTSMD = 0;
    
    
    if(parity_mode == 0){
        // 8bit+none
        U2MODEbits.PDSEL = 0b00;
    }else if(parity_mode == 1){
        // 8bit+odd
        U2MODEbits.PDSEL = 0b10;
    }else if(parity_mode == 2){
        // 8bit+even
        U2MODEbits.PDSEL = 0b01;
    }
    U2MODEbits.STSEL = stop_bit - 1;
    U2MODEbits.BRGH = 0;            // standard mode
    
    if(baudRate > 230400){
        U2MODEbits.BRGH = 1;
        U2BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/4. - 1.);
    }else{
        U2MODEbits.BRGH = 0;
        U2BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/16. - 1.);
    }

    U2STAbits.UTXEN = 1;            // enable TX
    U2STAbits.URXEN = 1;            // enable RX
    U2MODEbits.ON = 1;              // enable U2  
    
    IPC36bits.U2RXIP = 4;           // priority
    IPC36bits.U2RXIS = 0;           // sub priority
//    IFS4bits.U2RXIF = 0;            // U2RX INT flag clear
//    IEC4bits.U2RXIE = 1;            // U2RX INT enable
}

void INIT_UART_3(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode){
    // baudRate = XX (bps)
    
    // stop_bit = 1 or 2
    
    // parity_mode 0 = none
    //             1 = odd
    //             2 = even                   
    
    // clear uart
    U3MODE = 0;
    
    // setting
    U3MODEbits.UEN = 0;             // RXTX mode
    U3STAbits.UTXISEL0=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
	U3STAbits.UTXISEL1=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
    U3STAbits.URXISEL = 0;          // Rx interrupt mode selection
    U3STAbits.URXISEL0 = 0;
    U3STAbits.URXISEL1 = 0;
    //U3MODEbits.RTSMD = 0;
    
    
    if(parity_mode == 0){
        // 8bit+none
        U3MODEbits.PDSEL = 0b00;
    }else if(parity_mode == 1){
        // 8bit+odd
        U3MODEbits.PDSEL = 0b10;
    }else if(parity_mode == 2){
        // 8bit+even
        U3MODEbits.PDSEL = 0b01;
    }
    U3MODEbits.STSEL = stop_bit - 1;
    U3MODEbits.BRGH = 0;            // standard mode
    
    if(baudRate > 230400){
        U3MODEbits.BRGH = 1;
        U3BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/4. - 1.);
    }else{
        U3MODEbits.BRGH = 0;
        U3BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/16. - 1.);
    }

    U3STAbits.UTXEN = 1;            // enable TX
    U3STAbits.URXEN = 1;            // enalbe RX
    U3MODEbits.ON = 1;              // enable U3 
    
    IPC39bits.U3RXIP = 4;           // priority
    IPC39bits.U3RXIS = 1;           // sub priority
//    IFS4bits.U3RXIF = 0;            // U2RX INT flag clear
//    IEC4bits.U3RXIE = 1;            // U2RX INT enalbe
}


void INIT_UART_4(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode){
    // baudRate = XX (bps)
    
    // stop_bit = 1 or 2
    
    // parity_mode 0 = none
    //             1 = odd
    //             2 = even                   
    
    // clear uart
    U4MODE = 0;
    
    // setting
    U4MODEbits.UEN = 0;             // RXTX mode
    U4STAbits.UTXISEL0=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
	U4STAbits.UTXISEL1=0;           // UART_INTERRUPT_ON_TX_NOT_FULL
    U4STAbits.URXISEL = 0;          // Rx interrupt mode selection
    U4STAbits.URXISEL0 = 0;
    U4STAbits.URXISEL1 = 0;
    //U4MODEbits.RTSMD = 0;
    
    
    if(parity_mode == 0){
        // 8bit+none
        U4MODEbits.PDSEL = 0b00;
    }else if(parity_mode == 1){
        // 8bit+odd
        U4MODEbits.PDSEL = 0b10;
    }else if(parity_mode == 2){
        // 8bit+even
        U4MODEbits.PDSEL = 0b01;
    }
    U4MODEbits.STSEL = stop_bit - 1;
    U4MODEbits.BRGH = 0;            // standard mode
    
    if(baudRate > 230400){
        U4MODEbits.BRGH = 1;
        U4BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/4. - 1.);
    }else{
        U4MODEbits.BRGH = 0;
        U4BRG = (unsigned int)(((double)PBCLK)/((double)baudRate)/16. - 1.);
    }

    U4STAbits.UTXEN = 1;            // enable TX
    U4STAbits.URXEN = 1;            // enable RX
    U4MODEbits.ON = 1;              // enable U4  
    
    IPC42bits.U4RXIP = 6;           // priority
    IPC42bits.U4RXIS = 2;           // sub priority
//    IFS5bits.U4RXIF = 0;            // U2RX INT flag clear
//    IEC5bits.U4RXIE = 1;            // U2RX INT enable
}