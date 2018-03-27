// ********************************************************************************
// Function Prototypes
// ********************************************************************************
void usart_init(void);
char usart_getchar( void );
void usart_putchar( char );
void usart_pstr (char *);
unsigned char usart_kbhit(void);
int usart_putchar_printf(char, FILE *);
