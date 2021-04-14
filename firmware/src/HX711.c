#include "HX711.h"
#include <proc/p32mz2048efh064.h>
#include <time.h>
#include <stdio.h>


#define LOAD_CELL_CLOCK     LATBbits.LATB2
#define LOAD_CELL_DATA      PORTBbits.RB3

//#define LOAD_CELL_CLOCK     LATGbits.LATG6
//#define LOAD_CELL_DATA      PORTGbits.RG7


#define MSBFIRST 0
#define LSBFIRST 1

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.

uint8_t shiftInSlow(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(1);
        if(bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
        delayMicroseconds(1);
    }
    return value;
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftInSlow(data,clock,order)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(order) shiftIn(order)
#endif



void HX711_begin(unsigned char gain) {
	HX711_set_gain(gain);
}

int HX711_is_ready() {
	return (LOAD_CELL_DATA == 0);
}

void HX711_set_gain(unsigned char gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

}

long HX711_read() {
	// Wait for the chip to become ready.
	HX711_wait_ready(1);
//    int times=0;
//    while(times<200000){
//       if(HX711_is_ready()){          
//           break;
//       }
//       times++;
//    }
//    if(times>=200000){
//        return 0;
//    }
    
//    if(!HX711_is_ready()){
//        return 0;
//    }
    
	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;
	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.
	#if HAS_ATOMIC_BLOCK
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

	#elif IS_FREE_RTOS
	// Begin of critical section.
	// Critical sections are used as a valid protection method
	// against simultaneous access in vanilla FreeRTOS.
	// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
	// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	#else
	// Disable interrupts.
//    SYS_INT_Disable();
//    __builtin_disable_interrupts();
	#endif

	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(MSBFIRST);
	data[1] = SHIFTIN_WITH_SPEED_SUPPORT(MSBFIRST);
	data[0] = SHIFTIN_WITH_SPEED_SUPPORT(MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
    unsigned int i, k=0, l=0;
	for (i = 0; i < GAIN; i++) {
        LOAD_CELL_CLOCK = 1;
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
//        for(k=0; k<8000; k++);
        LOAD_CELL_CLOCK = 0;
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
//        for(l=0; l<8000; l++);
	}

	#if IS_FREE_RTOS
	// End of critical section.
	portEXIT_CRITICAL(&mux);

	#elif HAS_ATOMIC_BLOCK
	}

	#else
	// Enable interrupts again.
//    SYS_INT_Enable();
//    __builtin_mtc0(12, 0,(__builtin_mfc0(12, 0) | 0x0001));
	#endif

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( (unsigned long)(filler) << 24
			| (unsigned long)(data[2]) << 16
			| (unsigned long)(data[1]) << 8
			| (unsigned long)(data[0]) );

    
	return (unsigned long)(value);
}

void HX711_wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!HX711_is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
        //HX711_Delay_ms(delay_ms);
	}
}

int HX711_wait_ready_in_period(unsigned long times) {
	// repeat checking if the chip is ready in specified times.
	// return true if the chip is ready within that time 
	// return false if the chip is not ready 
	while (--times > 0) {
		if(HX711_is_ready()) {return 1;}
	}
    return 0;
}

int HX711_wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	int count = 0;
	while (count < retries) {
		if (HX711_is_ready()) {
			return 1;
		}  
        HX711_Delay_ms(delay_ms);
		count++;
	}
	return 0;
}

int HX711_wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
    time_t tstart;
    tstart= clock();
	while (clock() - tstart < timeout) {
		if (HX711_is_ready()) {
			return 1;
		}
        HX711_Delay_ms(delay_ms);
	}
	return 0;
}

long HX711_read_average(unsigned char times) {
	long sum = 0;
    unsigned char i;
	for (i = 0; i < times; i++) {
		sum += HX711_read();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
               
        HX711_Delay_ms(0);
	}
	return sum / times;
}

double HX711_get_value(unsigned char times) {
	return HX711_read_average(times) - OFFSET;
}

float HX711_get_units(unsigned char times) {
	return HX711_get_value(times) / SCALE;
}

void HX711_tare(unsigned char times) {
	double sum = HX711_read_average(times);
	HX711_set_offset(sum);
}

void HX711_set_scale(float scale) {
	SCALE = scale;
}

float HX711_get_scale() {
	return SCALE;
}

void HX711_set_offset(long offset) {
	OFFSET = offset;
}

long HX711_get_offset() {
	return OFFSET;
}

void HX711_power_down() {
	LOAD_CELL_CLOCK = 0;
	LOAD_CELL_CLOCK = 1;
}

void HX711_power_up() {
	LOAD_CELL_CLOCK = 0;
}


uint8_t shiftIn(uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;
//    unsigned int dd;
    for (i = 0; i < 8; ++i) {
        LOAD_CELL_CLOCK = 1;
//        for(dd = 0; dd < 50; dd++)
//            ;
      
        if (bitOrder == LSBFIRST)
            value |= (LOAD_CELL_DATA) << i;
        else
            value |= (LOAD_CELL_DATA) << (7 - i);
//        for(dd = 0; dd < 100; dd++)
//            ;
        LOAD_CELL_CLOCK = 0;
//        for(dd = 0; dd < 150; dd++)
//            ;
    }
    return value;  
}

 // Assumes the SYSCLK and PBCLK frequencies are the same
// Occupies Timer1
void HX711_Delay_ms( unsigned int t)
{
//    T1CON = 0x8000;     // enable Timer1, source PBCLK, 1:1 prescaler
//    while( t--)
//    {
//        TMR1 = 0;
//        while( TMR1 < SYS_CLK_FREQ/1000); // wait 1ms
//    }
//    // disable Timer1
//    T1CONCLR = 0x8000;
    volatile long delay = SYS_CLK_FREQ/5000*t;
    while(--delay>0)
        ;
}