
#ifndef _HX711_LIBRARY_C_H    /* Guard against multiple inclusion */
#define _HX711_LIBRARY_C_H


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#include "system_config.h"
#include "system_definitions.h"
#include "system_config/default/system_config.h"

unsigned char GAIN;		// amplification factor
long OFFSET = 0;	// used for tare weight
//float SCALE = 1;	// used to return weight in grams, kg, ounces, whatever
float SCALE = 1;


// Initialize library with data output pin, clock input pin and gain factor.
// Channel selection is made by passing the appropriate gain:
// - With a gain factor of 64 or 128, channel A is selected
// - With a gain factor of 32, channel B is selected
// The library default is "128" (Channel A).
void HX711_begin(unsigned char gain);

// Check if HX711 is ready
// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
int HX711_is_ready();

// Wait for the HX711 to become ready
void HX711_wait_ready(unsigned long delay_ms);
int HX711_wait_ready_in_period(unsigned long times);
int HX711_wait_ready_retry(int retries, unsigned long delay_ms);
int HX711_wait_ready_timeout(unsigned long timeout, unsigned long delay_ms);

// set the gain factor; takes effect only after a call to read()
// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
// depending on the parameter, the channel is also set to either A or B
void HX711_set_gain(unsigned char gain);

// waits for the chip to be ready and returns a reading
long HX711_read();

// returns an average reading; times = how many times to read
long HX711_read_average(unsigned char times);

// returns (read_average() - OFFSET), that is the current value without the tare weight; times = how many readings to do
double HX711_get_value(unsigned char times);

// returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
// times = how many readings to do
float HX711_get_units(unsigned char times);

// set the OFFSET value for tare weight; times = how many times to read the tare value
void HX711_tare(unsigned char times);

// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
void HX711_set_scale(float scale);

// get the current SCALE
float HX711_get_scale();

// set OFFSET, the value that's subtracted from the actual reading (tare weight)
void HX711_set_offset(long offset);

// get the current OFFSET
long HX711_get_offset();

// puts the chip into power down mode
void HX711_power_down();

// wakes up the chip after power down mode
void HX711_power_up();
        
void HX711_Delay_ms( unsigned int t);

unsigned char shiftIn(unsigned char bitOrder);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */