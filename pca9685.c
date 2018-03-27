#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "i2c_master.h"
#include "pca9685.h"

/* **** deprecated, we won't be using internal PWM, not enough channels ****
 * none of these need to be high resolution, all can be 8 bit
 *
 * PB7  OC0A/OC1C
 * PB6  OC1B
 * PB5  OC1A
 * PB4  OC2A
 * PE5  OC3C
 * PE4  OC3B
 * PE3  OC3A
 *
 * TCNT - timer counter; frequency of operatoin
 * TCCR - timer control registers;
 * OCRnx - overflow control variable; set the speed/brightness here
 *
 */

void PCA9685_init(void) {
    PORTC |= _BV(PC1);
    //_delay_ms(2);

    i2c_start(PCA9685_1);
    i2c_write(PCA9685_MODE1);
    i2c_write(0b00110001);  // set mode to sleep so we can change the default PWM frequency
    i2c_stop();
    _delay_ms(1);           // meet required 50us sleep

    i2c_start(PCA9685_1);
    i2c_write(PCA9685_PRESCALE); // PWM frequency PRE_SCALE address
    i2c_write(0x1e);        // osc_clk/(4096*update_rate) // 25000000/(4096*1500)= 4.069 ~4
    i2c_stop();
    _delay_ms(1);           // meet required delay of 500us

    i2c_start(PCA9685_1);
    i2c_write(PCA9685_MODE1);
    i2c_write(0xa1);        // Set to our prefered mode[ Reset, INT_CLK, Auto-Increment, Normal Mode]
    i2c_stop();
    _delay_ms(1);           // meet required delay of 500us

    i2c_start(PCA9685_1);
    i2c_write(PCA9685_MODE2);
    i2c_write(0b00000100);  // Set to our prefered mode[Output logic state not inverted, Outputs change on STOP,
    i2c_stop();             // totem pole structure, When OE = 1 (output drivers not enabled), LEDn = 0]
}

void PCA9685_set_ch(uint8_t channel, uint16_t startc, uint16_t stopc) {
    /* channels 1-16 are numbered starting at 0 in this code
     * each channel state consumes two bytes, 12 bits are used for values.
     * channel_Low_ON
     * channel_High_ON
     * channel_Low_OFF
     * channel_High_OFF
     */

    // make sure we mask off forbidden bits; 0x1fff are the allowed bits, but 0x1nnn bit
    // won't be used
    startc = startc & 0xfff; // 4095
    stopc  = stopc & 0xfff;

    uint8_t bytes[]={startc&0xff, startc >> 8, stopc&0xff, stopc >> 8};
    //if (channel > 2)
    //    printf("ch#%i %x %x %x %x\r\n", channel, bytes[0], bytes[1], bytes[2], bytes[3]);

    cli();
    i2c_writeReg(PCA9685_1, PCA9685_CH0L+(channel*4), bytes, 4);
    sei();
}

void PCA9685_set_ch_percent(uint8_t channel, uint8_t pct) {
    uint16_t hval;

    if (pct == 0) {
        hval = 0;
    } else {
        hval = (int)(40.96*pct);
        hval -= 1;
        if (hval < 0) {
            hval=0;
        }
    }

    PCA9685_set_ch(channel, 0, hval);
}
