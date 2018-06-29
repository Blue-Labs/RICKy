/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "i2c_master.h"
#include "ssd1306.h"
#include "graphics.h"

#define SSD1306_I2CADDR 0x78 // onboard is 0x78, but a 7bit i2c needs to shift this 1 right, so 0x3c

//common parameters
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_FBSIZE 1024 //128x8
#define SSD1306_MAXROW 8

//command macro
#define SSD1306_CMD_DISPLAY_OFF 0xAE//--turn off the OLED
#define SSD1306_CMD_DISPLAY_ON 0xAF//--turn on oled panel

bool Adafruit_ssd1306syp_initialize() {
   //malloc the framebuffer.
   m_pFramebuffer = (unsigned char*)malloc(SSD1306_FBSIZE);
   if(m_pFramebuffer == 0){
      return false;
   }
   memset(m_pFramebuffer,0,SSD1306_FBSIZE);//clear it.

   cli();
   Adafruit_ssd1306syp_writeCommand(SSD1306_CMD_DISPLAY_OFF);//display off
   Adafruit_ssd1306syp_writeCommand(0x00);//Set Memory Addressing Mode
   Adafruit_ssd1306syp_writeCommand(0x10);//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
   Adafruit_ssd1306syp_writeCommand(0x40);//Set Page Start Address for Page Addressing Mode,0-7
   Adafruit_ssd1306syp_writeCommand(0xB0);//Set COM Output Scan Direction
   Adafruit_ssd1306syp_writeCommand(0x81);//---set low column address
   Adafruit_ssd1306syp_writeCommand(0xCF);//---set high column address
   Adafruit_ssd1306syp_writeCommand(0xA1);//--set start line address
   Adafruit_ssd1306syp_writeCommand(0xA6);//--set contrast control register
   Adafruit_ssd1306syp_writeCommand(0xA8);
   Adafruit_ssd1306syp_writeCommand(0x3F);//--set segment re-map 0 to 127
   Adafruit_ssd1306syp_writeCommand(0xC8);//--set normal display
   Adafruit_ssd1306syp_writeCommand(0xD3);//--set multiplex ratio(1 to 64)
   Adafruit_ssd1306syp_writeCommand(0x00);//
   Adafruit_ssd1306syp_writeCommand(0xD5);//0xa4,Output follows RAM content;0xa5,Output ignores RAM content
   Adafruit_ssd1306syp_writeCommand(0x80);//-set display offset
   Adafruit_ssd1306syp_writeCommand(0xD9);//-not offset
   Adafruit_ssd1306syp_writeCommand(0xF1);//--set display clock divide ratio/oscillator frequency
   Adafruit_ssd1306syp_writeCommand(0xDA);//--set divide ratio
   Adafruit_ssd1306syp_writeCommand(0x12);//--set pre-charge period
   Adafruit_ssd1306syp_writeCommand(0xDB);//
   Adafruit_ssd1306syp_writeCommand(0x40);//--set com pins hardware configuration
   Adafruit_ssd1306syp_writeCommand(0x8D);//--set vcomh
   Adafruit_ssd1306syp_writeCommand(0x14);//0x20,0.77xVcc
   Adafruit_ssd1306syp_writeCommand(0xAF);//--set DC-DC enable
   Adafruit_ssd1306syp_writeCommand(SSD1306_CMD_DISPLAY_ON);
   _delay_ms(10);
   sei();

   return true;
}

void Adafruit_ssd1306syp_clear(bool isUpdateHW)
{
   memset(m_pFramebuffer,0,SSD1306_FBSIZE);//clear the back buffer.
   if(isUpdateHW)
      Adafruit_ssd1306syp_update();//update the hw immediately
}

void Adafruit_ssd1306syp_writeCommand(char cmd) {
   cli();
   i2c_start(SSD1306_I2CADDR);
   //i2c_write(0x78);  //Slave address,SA0=0
   i2c_write(0x00);  // set flag; write command
   i2c_write(cmd);
   i2c_stop();
   sei();
}

void Adafruit_ssd1306syp_writeByte(unsigned char b)
{
   i2c_write(b);
}

void Adafruit_ssd1306syp_drawPixel(int16_t x, int16_t y, uint16_t color)
{
   unsigned char row;
   unsigned char offset;
   unsigned char preData;//previous data.
   unsigned char val;
   int16_t  index;

   if ((x < 0) || (x > Adafruit_GFX_width()) || (y < 0) || (y > Adafruit_GFX_height()) || ( m_pFramebuffer==0)) {
       printf("beyond border, not drawing pixel: x:%i x:%i c:%i\r\n", x, y, color);
       return;
   }

   //get the previous data;
   row = y/8;
   offset =y%8;

   index = row*Adafruit_GFX_width() + x;
   preData = m_pFramebuffer[index];

   //set pixel;
   val = 1<<offset;
   if(color!=0)
   {//white! set bit.
      m_pFramebuffer[index] = preData | val;
   }else
   {//black! clear bit.
      m_pFramebuffer[index] = preData & (~val);
   }
}

void Adafruit_ssd1306syp_startDataSequence()
{
   cli();
   i2c_start(SSD1306_I2CADDR);
   Adafruit_ssd1306syp_writeByte(0x78);
   Adafruit_ssd1306syp_writeByte(0x40);  //write data
   // sei() will be done in caller
}

void Adafruit_ssd1306syp_update()
{
#if 1
   unsigned int  i=0;
   unsigned char m,n;
   for(m=0;m<8;m++)
   {
      Adafruit_ssd1306syp_writeCommand(0xb0+m);   //page0-page1
      Adafruit_ssd1306syp_writeCommand(0x00);     //low column start address
      Adafruit_ssd1306syp_writeCommand(0x10);     //high column start address

      Adafruit_ssd1306syp_startDataSequence();
      for(n=0;n<128;n++)
      {
         Adafruit_ssd1306syp_writeByte(m_pFramebuffer[i++]);
      }
      i2c_stop();
      sei();
   }
#else
   updateRow(0,SSD1306_MAXROW);
#endif
}

void Adafruit_ssd1306syp_updateRow(int rowID)
{
   unsigned char x;
   unsigned int  index;
   if(rowID>=0 && rowID<SSD1306_MAXROW && m_pFramebuffer)
   {//this part is faster than else.
      //set the position
      i2c_start(SSD1306_I2CADDR);
      Adafruit_ssd1306syp_writeByte(0x78);  //Slave address,SA0=0
      Adafruit_ssd1306syp_writeByte(0x00);  //write command

      Adafruit_ssd1306syp_writeByte(0xb0+rowID);

      #warning wtf is x supposed to be?
      Adafruit_ssd1306syp_writeByte(((x&0xf0)>>4)|0x10);//|0x10
      Adafruit_ssd1306syp_writeByte((x&0x0f)|0x01);//|0x01

      i2c_stop();
      sei();

      //start painting the buffer.
      Adafruit_ssd1306syp_startDataSequence();
      for(x=0;x<SSD1306_WIDTH;x++)
      {
         index = rowID*SSD1306_WIDTH+x;
         Adafruit_ssd1306syp_writeByte(m_pFramebuffer[index]);
      }
      i2c_stop();
      sei();
   }
}
void Adafruit_ssd1306syp_updateRowSE(int startID, int endID)
{
   unsigned char y =0;
   for(y=startID; y<endID; y++)
   {
      Adafruit_ssd1306syp_updateRow(y);
   }
}
