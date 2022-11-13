/*********************************************************************
This is an Arduino library for our Monochrome SHARP Memory Displays

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1393

These displays use SPI to communicate, 3 pins are required to
interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/
#ifndef LIB_ADAFRUIT_SHARPMEM
#define LIB_ADAFRUIT_SHARPMEM

#include <Adafruit_GFX.h>
#include <Arduino.h>


#define SHARPMEM_CMD_NORMAL (0x02)
#define SHARPMEM_CMD_STBY (0x04)
#define SHARPMEM_CMD_RDIMG (0x16)
#define SHARPMEM_CMD_RDIMGPTLL (0x18)
#define SHARPMEM_CMD_LDIMG (0x20)
#define SHARPMEM_CMD_LDIMGPTL (0x22)
#define SHARPMEM_CMD_PTLAREA (0x27)
#define SHARPMEM_CMD_DISPDIR (0x28)
//ROTATION [D1,D0]: [00] none [01] 90 degree turn [10] 180 degree turn [11] 270 degree turn 
#define SHARPMEM_CMD_DITHER (0x2B)
//DITHER [D0] ： [0]OFF [1]ON
#define SHARPMEM_CMD_PRTCLSEL (0x2C)
#define SHARPMEM_CMD_MOVIE (0x2D)
//MOVIE [D0] ： [0]OFF [1]ON 
#define SHARPMEM_CMD_FLOAD (0x2E)
//FLOAD [D0] ： [0]OFF [1]ON 
#define SHARPMEM_CMD_DISP (0x33)
#define SHARPMEM_CMD_DISPPTL (0x35)


#define SHARPMEM_1BIT (0x00)
#define SHARPMEM_2BIT (0x01)
#define SHARPMEM_4BIT (0x02)

/**
 * @brief Class to control a Sharp memory display
 *
 */
class Adafruit_SharpMem : public Adafruit_GFX {
public:
  Adafruit_SharpMem(uint8_t reset, uint8_t rs, uint8_t cs, uint8_t wr, uint8_t rd, uint8_t ack, uint8_t en, uint8_t sync, uint16_t w = 800,uint16_t h = 600);
  boolean begin(void);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  uint8_t getPixel(uint16_t x, uint16_t y);
  void clearDisplay();
  void refresh(void);
  void clearDisplayBuffer();
  void enableDisplay();
  void disableDisplay();
  void rotationMode(uint8_t rotation);
  void diteringMode(bool on);
  void panelBits(uint8_t bits);
  void movieMode(bool on);
  void transferPanel();
  void sendCmd(uint8_t cmd, uint8_t parameter);
  void sendCmd(uint8_t cmd);
  void sendCmdAndData(uint8_t cmd, uint8_t* data, uint32_t length);
  void writeParallelBus(uint8_t data);


private:
  uint8_t *sharpmem_buffer = NULL;
  uint8_t _reset;
  uint8_t _rs;
  uint8_t _cs;
  uint8_t _wr;
  uint8_t _rd;
  uint8_t _ack;
  uint8_t _en;
  uint8_t _sync;

};

#endif
