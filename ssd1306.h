#include <stdbool.h>

bool Adafruit_ssd1306syp_initialize(void);
void Adafruit_ssd1306syp_clear(bool);
void Adafruit_ssd1306syp_writeCommand(char);
void Adafruit_ssd1306syp_writeByte(unsigned char);
void Adafruit_ssd1306syp_drawPixel(int16_t, int16_t, uint16_t);
void Adafruit_ssd1306syp_startDataSequence(void);
void Adafruit_ssd1306syp_update(void);
void Adafruit_ssd1306syp_updateRow(int);
void Adafruit_ssd1306syp_updateRowSE(int, int);


#define BLACK 0
#define WHITE 1

unsigned char* m_pFramebuffer;
