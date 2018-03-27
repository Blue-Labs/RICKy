#if 0
// the MOB CANCDMOB Rx command
#define MOB_RX_CMD ((1 << CONMOB1) | (0 << CONMOB0))
// the MOB CANCDMOB Tx command
#define MOB_TX_CMD ((0 << CONMOB1) | (1 << CONMOB0))

typedef struct
{
    uint32_t id;                //!< ID der Nachricht (11 Bit)
    struct
    {
        int rtr      :1;            //!< Remote-Transmit-Request-Frame?
        int extended :1;
    } flags;

    uint8_t length;             //!< Anzahl der Datenbytes
    uint8_t data[8];            //!< Die Daten der CAN Nachricht

    uint16_t timestamp;
} can_t;


can_t msg;

void callback_canbus(void)
{
    // check if this message is for us
    //   respond to it

}

void fan_pwm_set(int rpm)
{
    // set by value
}

int fan_pwm_get(void)
{
    int fan_rpm_speed=0;
    return fan_rpm_speed;
}

void can_tx (can_t *);
void chip_init (void);
uint8_t can_send_message(can_t *);


volatile int8_t data[8];

//***** Reception ISR **********************************
ISR ( CANIT_vect ){               // use interrupts

    int8_t length, savecanpage;

    savecanpage = CANPAGE;          // Save current MOB

    CANPAGE = CANHPMOB & 0xF0;      // Selects MOB with highest priority interrupt

    if ( CANSTMOB & ( 1 << RXOK) ){     // Interrupt caused by receive finished
        length = ( CANCDMOB & 0x0F );   // DLC, number of bytes to be received

        for ( int8_t i = 0; i < length; i++ ){
            data[i] = CANMSG;       // Get data, INDX auto increments CANMSG
            PORTC = data[i];
            msg.data[i] = data[i];
        }

        CANCDMOB = (( 1 << CONMOB1 ) | ( 1 << IDE ) | ( 8 << DLC0));  // Enable Reception 29 bit IDE DLC8
        // Note - the DLC field of the CANCDMO register is updated by the received MOb. If the value differs from expected DLC, an error is set

    } // if (CANSTMOB...

    CANSTMOB = 0x00;        // Reset reason on selected channel
    CANPAGE = savecanpage;      // Restore original MOB

    msg.id=0xffff;
    msg.length=length;
    can_send_message(&msg);
}


//***** CAN initialization *****************************************************
// see http://www.avrfreaks.net/forum/at90can128-interface-problem
void can_init(void){
    cli();                                // ensure we're not interrupted

    // The first time you run can_init() after an AVR hardware reset, the ENFG bit will always be cleared
    //    and this code will not execute. This only works on subsequent calls to can_init().
    // If the CAN controller is enabled....
    if (CANGSTA & (1 << ENFG))
    {
        CANGCON = 0;            // ....force the CAN hardware into standby mode
            // Wait for any on going CAN Tx to cease (it must cease, so no timeout backup is needed)
            //    The CAN state machine Rx state may be confused by this forced standby. However, the
            //    SWRES reset below will take care of this and reset the CAN state machine
        while (CANGSTA & (1 << ENFG));
    }

    CANGCON = ( 1 << SWRES );             // Software reset, initialize the CAN state machine and only the general CAN registers
    CANTCON = 0x00;                       // CAN timing prescaler set to 0;

    CANBT1 = 0x1E;                        // Baud Rate Prescaler,  Set baud rate to 125kbps, TQ=1.00 (assuming 16Mhz IOclk)
                                          //                       pg 267
    CANBT2 = 0x04;                        // Re-Synchronization Jump Width and Propagation
    CANBT3 = 0x13;                        // Phase Segment 1, Phase Segment 2, and Sample Point

    for ( int8_t mob=0; mob<15; mob++ ) { // on chip reset, ALL MOBs are undefined and must be zeroed
        CANPAGE = ( mob << 4 );           // Selects Message Object 0-14
        CANCDMOB = 0x00;                  // Disable mob
        CANSTMOB = 0x00;                  // Clear mob status register;
    }

    CANPAGE = ( 1 << MOBNB0 );            // Selecto MOB0
    CANIE2 = ( 1 << IEMOB0 );             // Enable interrupts on mob0 for reception and transmission
    CANGIE = ( 1 << ENIT ) | ( 1 << ENRX );   // Enable interrupts on receive /*| ( 1 << ENTX )*/
    CANIDM1 = 0x00;                       // Clear Mask, let all IDs pass
    CANIDM2 = 0x00;                       // ""
    CANIDM3 = 0x00;                       // ""
    CANIDM4 = 0x00;                       // ""
    CANCDMOB = ( 1 << CONMOB1) | ( 1 << IDE ) | ( 8 << DLC0);  // Enable Reception 29 bit IDE DLC8
    CANGCON |= ( 1 << ENASTB );           // Enable mode. CAN channel enters in enable mode once 11 recessive bits have been read

    sei();
}

// get next free MOb
uint8_t _find_free_mob(void)
{
    uint8_t i;
    for (i = 0;i < 15;i++)
    {
        CANPAGE = i << 4; // load MOb page

        // check if MOb is in use
        if ((CANCDMOB & ((1 << CONMOB1) | (1 << CONMOB0))) == 0)
            return i;
    }

    return 0xff;
}

void can_copy_message_to_mob(const can_t *msg)
{
    uint8_t i;

    // write DLC (Data Length Code)
    CANCDMOB = msg->length;

    if (0) {
        // standard CAN ID, 11bit
        CANIDT4 = 0;
        CANIDT3 = 0;
        CANIDT2 = (uint8_t)  msg->id << 5;
        CANIDT1 = (uint16_t) msg->id >> 3;
    } else {
        // extended CAN ID, 29bit
        //CANIDT1 = (unsigned char)(msg->id>>21);
        //CANIDT2 = (unsigned char)(msg->id>>13);
        //CANIDT3 = (unsigned char)(msg->id>>5);
        //CANIDT4 = (unsigned char)(msg->id<<3);
        CANIDT = msg->id<<3;
    }

    // Put message data into the CAN message
    // Dummy data
    for (i = 0;i < msg->length;i++)
    {
        CANMSG = msg->data[i];
    }
}

// enable interrupt of corresponding MOb
void _enable_mob_interrupt(uint8_t mob)
{
    if (mob < 8)
        CANIE2 |= (1 << mob);
    else
        CANIE1 |= (1 << (mob - 8));
}

uint8_t can_send_message(can_t *msg)
{
    // check if there is any free MOb
    uint8_t mob;

    cli();
    mob = _find_free_mob();

    // load corresponding MOb page ...
    CANPAGE = (mob << 4);

    // clear flags
    CANSTMOB = 0x00;

    // ... and copy the data
    can_copy_message_to_mob( msg );

    // enable interrupt
    _enable_mob_interrupt(mob);

    // enable transmission
    //CANCDMOB = MOB_TX_CMD | (1 << IDE) ; //| (msg->length & 0x0F); IDE includes an 18bit identifier
    CANCDMOB = MOB_TX_CMD | 1 << IDE | (msg->length & 0x0f) ; //| (msg->length & 0x0F);

    while(!(CANSTMOB &(1<<TXOK)));
    CANSTMOB = 0x00; // clear TXOK flag
    CANCDMOB = 0x00; // disable transmission
    sei();

    return (mob + 1);
}



//***** transmit *****************************************************
// not currently used
void can_tx(can_t *msg) {
    CANPAGE = 0x00;     // Select MOb0 for transmission

    while ( CANEN2 & ( 1 << ENMOB0 ) ); // Wait for MOb 0 to be free

    CANSTMOB = 0x00;        // Clear mob status register
    CANIDT4 = 0x00;         // Set can id to 0
    CANIDT3 = 0x00;     // ""
    CANIDT2 = 0x00;     // ""
    CANIDT1 = 0x00;     // ""

    for ( int8_t i = 0; i < 8; ++i ){
        CANMSG = 0x55;  // set message data for all 8 bytes to 55 (alternating 1s and 0s
    } // for

    CANCDMOB = ( 1 << CONMOB0 ) | ( 1 << IDE ) | ( 8 << DLC0 );     // Enable transmission, data length=1 (CAN Standard rev 2.0B(29 bit identifiers))

    while ( ! ( CANSTMOB & ( 1 << TXOK ) ) );   // wait for TXOK flag set
// todo: have this use interrupts
    CANCDMOB = 0x00;    // Disable Transmission
    CANSTMOB = 0x00;    // Clear TXOK flag
}

int can_main(void)
{
    uint32_t msgid = 0;

    chip_init();    // Chip initialization
    can_init();     // Can initialization

    // Create a test message
    //msg.id = 0x7ff; // max is 0x7ff for 11bit, or 0x1fffffff for 29bit
    // Bosch can spec dictates 0-0x7ef, 0x0x1fbfffff as certain ranges
    // are not allowed

    msg.flags.rtr = 0;
    msg.flags.extended = 1;

    msg.length = 8;
    msg.data[0] = 0xde;
    msg.data[1] = 0xad;
    msg.data[2] = 0xbe;
    msg.data[3] = 0xef;
    msg.data[4] = 0x00;
    msg.data[5] = 0x14;
    msg.data[6] = 0x30;
    msg.data[7] = 0x69;

    while(1) {
        PORTB |= _BV(PB7);
        _delay_ms(1000);

        PORTB &= ~ _BV(PB7);
        _delay_ms(1000);

        if (1 || data[0] == 0x01){   // if 1st byta of received data is 0x01

            // Send the message
            msg.id = msgid;
            can_send_message(&msg);
            //can_tx(msg);   // transmit

            data[0] = 0x00;
            msgid += 1;

            if (msgid > 65535) {
                msgid=0;
            }
        }
    }
}

#endif

