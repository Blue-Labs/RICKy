/*
 * (c) david ford <david@blue-labs.org>, 2017-2018
    Apache License 2.0


    Courtesy lights
        Glow on/off. Glows operation is triggered at door ajar switch and follows state of door.
        Glow on/off operation changes direction immediately upon change of door state. If on
        when ignition is turned on, door triggered light will remain on for 60 seconds before
        glowing off. If ignition is set to run, the timer is set to zero and glow off function
        starts.

        If glow function is triggered by the manual cabin switches, the lit state is toggled by
        each press of the switch. The manual cabin switches override the door triggered state
        immediately. Such as, you can open the door and tap the light to turn it off.

    Main dome light
        Only glowed on/off by the dash on/off switch, no other feature.

    Fans
        The radiator fans operate based on engine coolant temperature. The two small fans turn
        on to 50%, 70%, and 100%. The main fan turns on to 100% when the small fans hit 100%.
        All radiator fans go to 100% when the A/C clutch engages.

        [todo] Main cabin climate fan operates as normally expected, the control circuit simply
        replaces the old wire resistor speed control with PWM control. Possible future code will
        control the speed based on temperature sensing.

    Door locks
        [todo] Activation or deactivation of locks will blink the running lights
        [todo] Remote disarm of locks with close-by area sense of owner

    Door windows
        [todo] Operation of the doors will alter the window position slightly to reduce the cabin
        air pressure lock when opening/closing doors

        [todo] monitor cabin temp, crack windows and ventilate if external rain sensor is dry and
        temp is really hot


    Hardware in use:
        AT90CANxx (developed on a 32)
            http://ww1.microchip.com/downloads/en/DeviceDoc/doc7679.pdf

        16Mhz ECS-2208 external crystal oscillator module
        MC14490 input debounce chip
            https://www.onsemi.com/pub/Collateral/MC14490-D.PDF

        TC4427 High Speed MOSFET driver chip
            http://www.mouser.com/ds/2/268/20001422G-967549.pdf

        IRF3205 N-channel MOSFETs
        PCA9685 12 channel 12 bit programmable PWM
            https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf

        If you plan to implement the CAN bus functionality:
        MC2551 CAN bus hardware interface

        If you use the OLED display; SSD1306 128x64, or similar (i2c driven)

    Functions used on AT90CANxx:
        I2C, aka TWI
        Digital IO, both INPUT and /INPUT
        ADC for analog measurement of ECT
        Timer0 for ~12ms polling of inputs
        Timer1 for 1 second tick
        CAN messaging


    BUGS:
        PE3 won't tri-state for input, floats at 3.76V, have to use a fairly low value resistor to get it to drop to 2V...


    The ssd, graphics, and glcdfont are modified versions of the Adafruit files



    on/off inputs:
        door open, turns on appropriate courtesy/map lamp, pushes window to 1/4in open/closes, will require an accurate window height indication
            driver door
            passenger door
        dash light controls
            OEM PWM/main cabin overhead dome light, state matches switch state, glow on/off
            me/button switches for courtesy lamps above drv and psgr seats, button press toggles state, glow on/off
        alarm
            arm/disarm, blink running lamps 2x quick on arm or disarm, on/off
        A/C clutch
            engages all 3 fans 100%

    analog inputs:
        engine coolant temp, controls radiator fans

    i2c digital io input nodes:
        climate fan speed (off, low, medium, high)
        driver window position
        passenger window position

    outputs:
        on/off
            running lights
        positionally controlled
            window motors
            door locks
        PWM
            glow on/off ~1sec
                all cabin lighting
            percentage set for fan control:
                3x radiator fans
                1 climate control cabin fan



    CAN references:
      Ford Radio CAN IDs (and some others)
        https://docs.google.com/spreadsheets/d/1aqWTyXr4O9Fnjt_ZvojrrKbXVxQZ8Ms7isXg39QUh70/edit#gid=0
      sniffed CAN messages
        https://groups.google.com/forum/#!topic/openxc/-UoxXVcfoVY
      https://sourceforge.net/p/ecu/wiki/canbus/



All input triggers are debounced

ECT sensor is a thermister between ground and input. 350Ω at ~80°F, 26°C. This yields
a "cold" temp voltage of 5.32vdc. "hot" is about 4.19vdc which is about 84°C

 */

/* Standard Includes */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/iocanxx.h>

#include "i2c_master.h"
#include "pca9685.h"
#include "ssd1306.h"
#include "graphics.h"
#include "usart_printf.h"

#define False false
#define True  true

enum _Types { NONE=0, ONOFF=1, GLOW=2, RANGE=3, POSITIONAL=4 };

typedef struct {
    char *name;
    enum _Types type;
    uint8_t timer;
    struct {
        bool enabled;               // is this device wholly ignored?
        bool running;               // event planner is making changes on this device
        bool direction;             // 0=down, 1=up
        bool triggered_by_door;     // only the door triggers the 60sec countdown
    } flags;
    uint8_t current_level;
} dev_t;

dev_t * fans;
dev_t * locks;
dev_t * motors;
dev_t * lights;

struct {
    bool changed:1;
    bool door_left:1;
    bool door_right:1;
    bool courtesy_switch_left:1;
    bool courtesy_switch_right:1;
    bool ac_clutch:1;
    bool locks_arm:1;
    bool locks_disarm:1;
    bool dome_light:1;
    int16_t ect:16;
    bool ignition_on:1;
    bool ignition_run:1;
} inputs;

uint16_t timer0_ovf=0;

void periodic(void);


void chip_init(void){
    DDRA   = 0xff;
    DDRB   = 0b00000000;        // PORTB7:0 are inputs
    DDRC   = 0xff;              // enable outputs for I2C, USART1, CAN msgs
    DDRD  |= _BV(PD0)|_BV(PD1); // i2c pins as outputs

    DDRE   = 0x00;              // these are all INPUTS
    DDRF   = 0x00;              // 0x00 is default, use PF0/ADC0 input
    DDRG   = 0x00;              // inputs

    PORTB  = 0b00001110;        // make sure input pull-up resistor isn't up for B7:4 and :1, these are HIGH inputs
    PORTE  = 0b11110111;        // turn on the pull-up resistors for PORTE
    PORTF  = 0b11111110;
    PORTG  = 0b00011111;
    
    DIDR1=0;

    //PRR = 0x00; // Individual peripheral clocks enabled

    /* configure the input switches (and window position modules? i2c i suppose)
     * EICRx is on pg 93
     *
     * The two doors and alarm arm/disarm need to be external interrupts
     * to enable a wakeup call when the truck is off. The rest can be
     * polled.
     *
     * PE0 - door lock ARM
     * PE1 - door lock DISARM
     * PB1 - ac_clutch                               || can't use PE2 due to ISP circuit
     * PB0 - overhead dome light input      + input  || can't use PE3 due to ISP circuit
     * PE4 - driver courtesy switch
     * PE5 - passenger courtesy switch
     * PE6(INT6) - driver door ajar
     * PE7(INT7) - passenger door ajar
     *
     * PG3 - ignition RUN                   + input
     * PG4 - ignition START                 + input
     *
     */

                                      // configure timer0 to fire off every 100.16025641025641 milliseconds
    TCCR0A |= _BV(CS01)|_BV(CS00);    // prescaling set to /1024, pg 111
    TCNT0   = 0;                      // initialize counter
    TIMSK0 |= _BV(TOIE0);             // enable the OVF interrupt for this timer
    timer0_ovf = 0;

                                        // configure timer1 to fire every 1 second
    TCCR1A = _BV(WGM11)|_BV(WGM10);     // set bits to use OCR1A for counter TOP, disconnect OC1A from port pins
    TCCR1B = _BV(WGM13)|_BV(WGM12) | _BV(CS12)|_BV(CS10);   // with a /1024 prescaler
    OCR1A = 15625;
    TIMSK1 |= _BV(TOIE1);


    // setup the ADC to read the Engine Coolant Temp
    // pg 288
    //ADMUX = 0x00; // default
    ADCSRA = 0x07; // set prescaler to 128, 16M/128=125kHz sample rate
    ADCSRB = 0x00;
    DIDR0  = 0x00; //0xff;  // disable digital input on these ports
    ADCSRA &= ~_BV(ADEN);         // Disable ADC
    ADMUX   = (ADMUX & 0xf8)|0xe0; // Set vref, adjust left, and channel, pg 280,287
    ADCSRA |= _BV(ADEN);          // Enable ADC


    // configure the hardware interrupts
    // LT & RT door open switches

    //EIMSK = 0; // mask off all interrupts prior to changes
    //EICRA |= _BV(ISC40);             // 1.0 is any logical change on INTn
    EICRB  = _BV(ISC70) | _BV(ISC60);  // any logical change generates interrupt
    EIMSK  = _BV(INT7)  | _BV(INT6);   // allow int7 and 6
}

void device_structure_init(void) {
    fans = calloc(3, sizeof(dev_t));
    fans[0].name="Main radiator fan";
    fans[0].flags.enabled=1;
    fans[0].current_level=0;

    fans[1].name="Small radiator fans";
    fans[1].flags.enabled=1;
    fans[1].current_level=0;

    fans[2].name="Cabin climate fan";
    fans[2].flags.enabled=1;          // and commanded state
    fans[2].current_level=0;

    locks = calloc(2, sizeof(dev_t));
    locks[0].name="Driver door lock";
    locks[0].flags.enabled=1;
    locks[0].current_level=0;

    locks[1].name="Passenger door lock";
    locks[1].flags.enabled=1;
    locks[1].current_level=0;

    motors = calloc(2, sizeof(dev_t));
    motors[0].name="Driver door window motor";
    motors[0].flags.enabled=1;
    motors[0].current_level=0;

    motors[1].name="Passenger door window motor";
    motors[1].flags.enabled=1;
    motors[1].current_level=0;

    lights = calloc(5, sizeof(dev_t));
    lights[0].name = "Main cabin overhead light";
    lights[0].flags.enabled=1;
    lights[0].current_level=0;

    lights[1].name = "Driver spotlight";
    lights[1].flags.enabled=1;
    lights[1].current_level=0;

    lights[2].name = "Passenger spotlight";
    lights[2].flags.enabled=1;
    lights[2].current_level=0;

    lights[3].name = "Instrument cluster/dash/trim lighting";
    lights[3].flags.enabled=1;
    lights[3].current_level=0;

    lights[4].name = "Running lights";
    lights[4].flags.enabled=1;
    lights[4].current_level=0;
}

/*
 * at ~100Hz, run this service handler
 *
 * loop through the collection of events, for each active element,
 * handle the next event such as stepping the lamp brightness one
 * notch
 *
 */

ISR(TIMER0_OVF_vect) {
    timer0_ovf++;

    if (timer0_ovf >= 10) {         // ~97.6Hz
        timer0_ovf=0;
        periodic();
    }
}

ISR(TIMER1_OVF_vect) {              // 1 second timer
    int i;

    if (!inputs.ignition_on) {
        for (i=0; i<3; i++) {
            if (lights[i].timer >0) {
                lights[i].timer=0;
            }
        }
    }

    for (i=0; i<3; i++) {
        if (lights[i].timer >0) {
            lights[i].timer--;
        }
    }
}

void periodic(void) {
    uint8_t z, _adcl;
    int16_t or_ect, cr_ect;
    static int16_t ect_p=0, ect_a[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int16_t ect_f;

    inputs.changed=False;

    z = !((PINE & _BV(PE0)) == _BV(PE0));
    if (inputs.locks_arm != z) {
        inputs.changed=True;
        inputs.locks_arm=z;
    }

    z = !((PINE & _BV(PE1)) == _BV(PE1));
    if (inputs.locks_disarm != z) {
        inputs.locks_disarm=z;
        inputs.changed=True;
    }

    z = !((PINE & _BV(PE2)) == _BV(PE2));
    if (inputs.ac_clutch != z) {
        inputs.ac_clutch=z;
        inputs.changed=True;
    }

    z = ((PINB & _BV(PB0)) == _BV(PB0));
    if (inputs.dome_light != z) {
        lights[0].flags.running =1;
        if (z)
            lights[0].flags.direction =1;
        else
            lights[0].flags.direction =0;

        inputs.dome_light=z;
        inputs.changed=True;
    }

    z = ((PINB & _BV(PB6)) == _BV(PB6));
    if (inputs.ignition_on != z) {
        inputs.ignition_on=z;
    }

    z = ((PINB & _BV(PB5)) == _BV(PB5));
    if (inputs.ignition_run != z) {
        inputs.ignition_run=z;

        // turn off the spot lights on engine start
        if (lights[1].timer > 0) {
            lights[1].timer=0;
            lights[1].flags.direction=0;
        }
        if (lights[2].timer > 0) {
            lights[2].timer=0;
            lights[2].flags.direction=0;
        }
    }

    // for door closing states, start a 60 second timer before fading off the lights
    // if we use another input to detect ignition ON state, we can null the 60 second
    // timer

    // toggles state
    z = !((PINE & _BV(PE4)) == _BV(PE4));
    if (z && inputs.courtesy_switch_left!=z) {
        /* check if the spotlight is already on and,
           if the timer is running and,
           if the manual switch was activated
             if so, negate the timer and start the fadeout
         */
        if (lights[1].current_level>0 && lights[1].timer > 0 && z) { // this is actually a manual turn off
            lights[1].timer=0;
            lights[1].flags.direction=0;
        } else {
            lights[1].flags.direction = !lights[1].flags.direction;
        }
        lights[1].flags.running =1;
        inputs.changed=True;
        inputs.courtesy_switch_left=z;
    } else if (inputs.courtesy_switch_left && !z) {
        inputs.courtesy_switch_left=z;
    }

    // toggles state
    z = !((PINE & _BV(PE5)) == _BV(PE5));
    if (z && inputs.courtesy_switch_right!=z) {
        /* check if the spotlight is already on and,
           if the timer is running and,
           if the manual switch was activated
             if so, negate the timer and start the fadeout
         */
        if (lights[2].current_level>0 && lights[2].timer > 0 && z) { // this is actually a manual turn off
            lights[2].timer=0;
            lights[2].flags.direction=0;
        } else {
            lights[2].flags.direction = !lights[2].flags.direction;
        }
        lights[2].flags.running =1;
        inputs.changed=True;
        inputs.courtesy_switch_left=z;
    } else if (inputs.courtesy_switch_right && !z) {
        inputs.courtesy_switch_right=z;
    }

    // radiator fans
    ADCSRA |= _BV(ADSC);                 // start conversion
    while(ADCSRA & _BV(ADSC));           // Wait for conversion

    _adcl  = ADCL & 0xf0;                // Take reading
    or_ect = (ADCH <<2) + (_adcl>>6);;   // Take reading
    cr_ect = (1024-or_ect)*.672-430;     // adjust (1024-or_ect+35)/3.125
    //cr_ect = (1024-or_ect+35)/3.125;

    // hysterisis model so a +/- 1 doesn't bounce around
    if (cr_ect < inputs.ect-1 || cr_ect > inputs.ect+1) {
        //printf("cr: %d\n", cr_ect);
        if (cr_ect >=90) {              // 90+
            fans[0].current_level=100;
            fans[1].current_level=100;
            fans[2].current_level=100;
            if (!(inputs.ect >= 90) ) {
                inputs.changed = True;
            }
        } else if (cr_ect >=80) {        // 80-89
            fans[0].current_level=50;
            fans[1].current_level=70;
            fans[2].current_level=70;
            if ((inputs.ect <80) || (inputs.ect >89) ) {
                inputs.changed = True;
            }
        } else if (cr_ect >=70) {        // 70-79
            fans[0].current_level=0;
            fans[1].current_level=50;
            fans[2].current_level=50;
            if ((inputs.ect <70) || (inputs.ect >79) ) {
                inputs.changed = True;
            }
        } else if (cr_ect >=50) {        // 50-69
            fans[0].current_level=0;
            fans[1].current_level=25;
            fans[2].current_level=25;
            if ((inputs.ect <50) || (inputs.ect >69) ) {
                inputs.changed = True;
            }
        } else {                        // up to 49
            fans[0].current_level=0;
            fans[1].current_level=0;
            fans[2].current_level=0;
            if (inputs.ect >49) {
                inputs.changed = True;
            }
        }
    }
    
    // running average
    ect_a[ect_p] = cr_ect;
    ect_p = (ect_p+1) % 30;
    ect_f=0;
    for (int n=0; n<30; n++) {
        ect_f += ect_a[n];
    }
    ect_f /= 30;
    
    inputs.ect = ect_f;

    if (inputs.ac_clutch) {
        fans[0].current_level=100;
        fans[1].current_level=100;
        fans[2].current_level=100;
    }

    // check dome light
    // check alarm arm/disarm

    // ::future:: check cabin temp and rain sensor, if over temp, drop the
    // windows half an inch and turn on a slow fan to ventilate



    //
    // now that the input checks are done, parse through the devices and update them.
    // this covers the courtesy lights and dome lights
    //

    if (lights[0].flags.running) { // dome light
        if (lights[0].flags.direction==1) {
            if (lights[0].current_level<100) {
                lights[0].current_level++;
                PCA9685_set_ch_percent(4, lights[0].current_level);

                if (lights[0].current_level>=100) {
                    lights[0].flags.running=0;
                }
            }
        } else {
            if (lights[0].current_level>0) {
                lights[0].current_level--;
                PCA9685_set_ch_percent(4, lights[0].current_level);

                if (lights[0].current_level<=0) {
                    lights[0].flags.running=0;
                }
            }
        }
    }

    if (lights[1].flags.running) { // left courtesy light
        if (lights[1].flags.direction==1) {
            if (lights[1].current_level<100) {
                lights[1].current_level++;
                PCA9685_set_ch_percent(5, lights[1].current_level);

                if (lights[1].current_level>=100) {
                    lights[1].flags.running=0;
                }
            }
        } else {
            if (lights[1].current_level>0) {
                if (lights[1].timer<=0) {
                    lights[1].current_level--;
                    PCA9685_set_ch_percent(5, lights[1].current_level);

                    if (lights[1].current_level<=0) {
                        lights[1].flags.running=0;
                    }
                }
            }
        }
    }

    if (lights[2].flags.running) { // right courtesy light
        if (lights[2].flags.direction==1) {
            if (lights[2].current_level<100) {
                lights[2].current_level++;
                PCA9685_set_ch_percent(5, lights[2].current_level);

                if (lights[2].current_level>=100) {
                    lights[2].flags.running=0;
                }
            }
        } else {
            if (lights[2].current_level>0) {
                if (lights[2].timer<=0) {
                    lights[2].current_level--;
                    PCA9685_set_ch_percent(5, lights[2].current_level);

                    if (lights[2].current_level<=0) {
                        lights[2].flags.running=0;
                    }
                }
            }
        }
    }

}

ISR(INT6_vect) {
    inputs.door_left = !((PINE & _BV(PE6)) == _BV(PE6));
    inputs.changed = True;

    lights[1].flags.running =1;
    if (inputs.door_left) {
        lights[1].flags.direction =1;
        lights[1].flags.triggered_by_door=True;

        // if door is re-opened with the timer running, reset the timer
        if (lights[1].timer>0) {
            lights[1].timer=60;
        }
    }
    else { // if the door is closed, start a timer, ignition_run can terminate it early
        if (inputs.ignition_on && !inputs.ignition_run && lights[1].flags.triggered_by_door) {
            if (lights[1].current_level==100) {
                lights[1].timer=60;                         // delay shutoff
            }
        }

        if (lights[1].timer>0 && lights[1].current_level<0) {
            // door was opened and closed quickly, make sure timer isn't set
        }
        lights[1].flags.direction =0;
    }
}

ISR(INT7_vect) {
    inputs.door_right = !((PINE & _BV(PE7)) == _BV(PE7));
    inputs.changed = True;

    lights[2].flags.running =1;
    if (inputs.door_right) {
        lights[2].flags.direction =1;
        lights[2].flags.triggered_by_door=True;

        // if door is re-opened with the timer running, reset the timer
        if (lights[2].timer>0) {
            lights[2].timer=60;
        }
    }
    else { // if the door is closed, start a timer, ignition_run can terminate it early
        if (inputs.ignition_on && !inputs.ignition_run && lights[2].flags.triggered_by_door) {
            if (lights[2].current_level==100) {
                lights[2].timer=60;                         // delay shutoff
            }
        }

        if (lights[2].timer>0 && lights[2].current_level<0) {
            // door was opened and closed quickly, make sure timer isn't set
        }
        lights[2].flags.direction =0;
    }
}

int main(void) {
    int16_t ect_p=-99;

    cli();
    usart_init();
    
    chip_init();
    //internal_pwm_init();
    i2c_init();
    device_structure_init();
    sei();
    
    //printf("Boot done\n");

    //TMP006_init();
    //DS1307_init();
    //DS1307_printsec();
    
    PCA9685_init();
    PCA9685_set_ch_percent(0, 0);
    PCA9685_set_ch_percent(1, 0);
    PCA9685_set_ch_percent(2, 0);
    for (int zz=0; zz<16; zz++) {
        PCA9685_set_ch_percent(zz, 0);
    }
    
    Adafruit_ssd1306syp_initialize();
    Adafruit_ssd1306syp_clear(1);
    Adafruit_GFX_setTextSize(1);
    Adafruit_GFX_setTextColor(WHITE, BLACK);
    #if 0
    //Adafruit_GFX_drawLine(0, 3, 5, 0, WHITE);
    //Adafruit_GFX_drawLine(127, 0, 127, 63, WHITE);
    //Adafruit_GFX_drawLine(127, 63, 0, 63, WHITE);
    //Adafruit_GFX_drawLine(0, 63, 0, 0, WHITE);
    //Adafruit_GFX_fillCircle(120,10,5,1);
    //Adafruit_GFX_fillRect(0,0,126,62,1);

    //Adafruit_ssd1306syp_update();
    Adafruit_GFX_setCursor(0,3);
    Adafruit_GFX_println("Tickle our display");
    Adafruit_GFX_setCursor(0,21);
    Adafruit_GFX_println("and make us happy");
    Adafruit_ssd1306syp_update();
    //Adafruit_GFX_startscrollright(0x00, 0x0F);
    //Adafruit_GFX_stopscroll();
    _delay_ms(1000);
    #endif
    
    uint8_t ch;
    char s[24];
    inputs.changed=1;
    for(;;) {
        if (inputs.ect != ect_p) {
            printf("inputs.ect= %i\n", inputs.ect);
            Adafruit_GFX_setCursor(0,0);
            sprintf(s, "ECT%3i", inputs.ect);
            Adafruit_GFX_println(s);
        }
    
        if (inputs.changed) {
            sprintf(s, "D%s%s C%s  %s  %s",
                inputs.door_left?"L":"l",
                inputs.door_right?"R":"r",
                inputs.courtesy_switch_left ? "L":"l",
                inputs.courtesy_switch_right ? "R":"r",
                inputs.dome_light?"DO":"do"
                );
            Adafruit_GFX_setCursor(56,0);
            Adafruit_GFX_println(s);

            sprintf(s, "       L%s%s I%s%s %s",
                inputs.locks_arm?"A":"a",
                inputs.locks_arm?"D":"d",
                inputs.ignition_on?"O":"o",
                inputs.ignition_run?"R":"r",
                inputs.ac_clutch? "AC":"ac"
                );
            Adafruit_GFX_setCursor(0,8);
            Adafruit_GFX_println(s);

            for (ch=0; ch<4; ch++) {
                PCA9685_set_ch_percent(ch, fans[ch].current_level);
                sprintf(s, "%i %3i", ch, fans[ch].current_level);
                Adafruit_GFX_setCursor(0,(ch+2)*8);
                Adafruit_GFX_println(s);
            }

            sprintf(s, "%3i %3i %3i", lights[1].current_level, lights[2].current_level, lights[0].current_level);
            Adafruit_GFX_setCursor(56,16);
            Adafruit_GFX_println(s);

            sprintf(s, "%c   %c   %c", lights[1].flags.running?'R':' ', lights[2].flags.running?'R':' ', lights[0].flags.running?'R':' ');
            Adafruit_GFX_setCursor(62,24);
            Adafruit_GFX_println(s);

            sprintf(s, "%3i %3i %3i", lights[1].timer, lights[2].timer, lights[0].timer);
            Adafruit_GFX_setCursor(56,32);
            Adafruit_GFX_println(s);

        }

        if (inputs.ect != ect_p || inputs.changed) {
            cli();
            Adafruit_ssd1306syp_update();
            sei();
            ect_p = inputs.ect;
        }
        _delay_ms(10);
    }

}

//ISR(INT7_vect) {
//}
