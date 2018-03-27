#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "usart_printf.h"

// ********************************************************************************
// Macros and Defines
// ********************************************************************************
#define UART_BAUD_RATE 57600
#define BAUD_PRESCALE (((F_CPU / (UART_BAUD_RATE * 16UL))) - 1)

// ********************************************************************************
// Global Variables
// ********************************************************************************
FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

// ********************************************************************************
// usart Related
// ********************************************************************************
void usart_init() {
    UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
    UCSR1C |= (0 << UMSEL1) | (1 << UCSZ10) | (1 << UCSZ11);

    UBRR1 = BAUD_PRESCALE;
    stdout = &mystdout;
}
void usart_putchar(char data) {
    // Wait for empty transmit buffer
    while ( !(UCSR1A & (_BV(UDRE1))) );
    // Start transmission
    UDR1 = data;
}
char usart_getchar(void) {
    // Wait for incoming data
    while ( !(UCSR1A & (_BV(RXC1))) );
    // Return the data
    return UDR1;
}
unsigned char usart_kbhit(void) {
    //return nonzero if char waiting polled version
    unsigned char b;
    b=0;
    if(UCSR1A & (1<<RXC1)) b=1;
    return b;
}
void usart_pstr(char *s) {
    // loop through entire string
    while (*s) {
        usart_putchar(*s);
        s++;
    }
}

// this function is called by printf as a stream handler
int usart_putchar_printf(char var, FILE *stream) {
    // translate \n to \r for br@y++ terminal
    if (var == '\n') usart_putchar('\r');
    usart_putchar(var);
    return 0;
}
