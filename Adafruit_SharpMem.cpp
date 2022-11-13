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

#include "Adafruit_SharpMem.h"
#include "Arduino.h"
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"


#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif
#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b)                                                   \
  {                                                                            \
    uint16_t t = a;                                                            \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

volatile uint32_t *wtsb = (volatile uint32_t *)0x40010094;
volatile uint32_t *wtcb = (volatile uint32_t *)0x4001009C;
volatile uint32_t *wtsa = (volatile uint32_t *)0x40010090;
volatile uint32_t *wtca = (volatile uint32_t *)0x40010098;

Adafruit_SharpMem::Adafruit_SharpMem(uint8_t reset, uint8_t rs, uint8_t cs, uint8_t wr, uint8_t rd, uint8_t ack, uint8_t en, uint8_t sync,
                                     uint16_t width, uint16_t height)
    : Adafruit_GFX(width, height) {
  _reset = reset; 
  _rs = rs; 
  _cs = cs; 
  _wr = wr; 
  _rd = rd; 
  _ack = ack; 
  _en = en; 
  _sync = sync;

}

/**
 * @brief Start the driver object, setting up pins and configuring a buffer for
 * the screen contents
 *
 * @return boolean true: success false: failure
 */
boolean Adafruit_SharpMem::begin(void) {

  sharpmem_buffer = (uint8_t *)malloc((WIDTH * HEIGHT) / 8);

  if (!sharpmem_buffer)
    return false;

  pinMode(_reset,OUTPUT);
  digitalWrite(_reset,HIGH);
  pinMode(_rs,OUTPUT);
  pinMode(_cs,OUTPUT);
  pinMode(_wr,OUTPUT);
  pinMode(_rd,OUTPUT);
  pinMode(_ack,INPUT_PULLUP);
  pinMode(_en,OUTPUT);
  pinMode(_sync,INPUT_PULLUP);

  digitalWrite(_rs,LOW);
  digitalWrite(_cs,HIGH);
  digitalWrite(_rd,HIGH);
  digitalWrite(_wr, HIGH);
  digitalWrite(_en,HIGH);

  for (int i = 0; i <= 7; ++i)
  {
    //Not sure why this is not working using Arduino HAL
    //pinMode(i,OUTPUT);
    //digitalWrite(i,LOW);
    am_hal_gpio_pinconfig(i,g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(i,AM_HAL_GPIO_OUTPUT_CLEAR);
    
  }

  digitalWrite(_reset,LOW);
  delay(100);
  digitalWrite(_reset,HIGH);
  delay(100);

  enableDisplay();

  rotationMode(3);
  panelBits(SHARPMEM_1BIT);

  return true;
}

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t PROGMEM set[] = {128, 64, 32, 16, 8, 4, 2, 1},
                             clr[] = {(uint8_t)~128,  (uint8_t)~64,  (uint8_t)~32,
                                      (uint8_t)~16,  (uint8_t)~8, (uint8_t)~4,
                                      (uint8_t)~2, (uint8_t)~1};

/**************************************************************************/
/*!
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)
    @param color The color to set:
    * **0**: Black
    * **1**: White
*/
/**************************************************************************/
void Adafruit_SharpMem::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
    return;

  switch (rotation) {
  case 1:
    _swap_int16_t(x, y);
    x = WIDTH - 1 - x;
    break;
  case 2:
    x = WIDTH - 1 - x;
    y = HEIGHT - 1 - y;
    break;
  case 3:
    _swap_int16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  if (color) {
    sharpmem_buffer[(y * WIDTH + x) / 8] |= pgm_read_byte(&set[x & 7]);
  } else {
    sharpmem_buffer[(y * WIDTH + x) / 8] &= pgm_read_byte(&clr[x & 7]);
  }
}

/**************************************************************************/
/*!
    @brief Gets the value (1 or 0) of the specified pixel from the buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)

    @return     1 if the pixel is enabled, 0 if disabled
*/
/**************************************************************************/
uint8_t Adafruit_SharpMem::getPixel(uint16_t x, uint16_t y) {
  if ((x >= _width) || (y >= _height))
    return 0; // <0 test not needed, unsigned

  switch (rotation) {
  case 1:
    _swap_uint16_t(x, y);
    x = WIDTH - 1 - x;
    break;
  case 2:
    x = WIDTH - 1 - x;
    y = HEIGHT - 1 - y;
    break;
  case 3:
    _swap_uint16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  return sharpmem_buffer[(y * WIDTH + x) / 8] & pgm_read_byte(&set[x & 7]) ? 1
                                                                           : 0;
}

/**************************************************************************/
/*!
    @brief Clears the screen
*/
/**************************************************************************/
void Adafruit_SharpMem::clearDisplay() {
  memset(sharpmem_buffer, 0xFF, (WIDTH * HEIGHT) / 8);
  refresh();
}

/**************************************************************************/
/*!
    @brief Renders the contents of the pixel buffer on the LCD
*/
/**************************************************************************/
void Adafruit_SharpMem::refresh(void) {

  sendCmdAndData(SHARPMEM_CMD_LDIMG,sharpmem_buffer,60000);

  transferPanel();

}

/**************************************************************************/
/*!
    @brief Clears the display buffer without outputting to the display
*/
/**************************************************************************/
void Adafruit_SharpMem::clearDisplayBuffer() {
  memset(sharpmem_buffer, 0xFF, (WIDTH * HEIGHT) / 8);
}

/**************************************************************************/
/*!
    @brief Start the display
*/
/**************************************************************************/
void Adafruit_SharpMem::enableDisplay() {
  sendCmd(SHARPMEM_CMD_NORMAL);
}

/**************************************************************************/
/*!
    @brief Shutdown the display
*/
/**************************************************************************/
void Adafruit_SharpMem::disableDisplay() {
  sendCmd(SHARPMEM_CMD_STBY);
}

/**************************************************************************/
/*!
    @brief Shutdown the display
*/
/**************************************************************************/
void Adafruit_SharpMem::rotationMode(uint8_t rotation) {
  sendCmd(SHARPMEM_CMD_DISPDIR,rotation);
}


/**************************************************************************/
/*!
    @brief Enable Ditering 
*/
/**************************************************************************/
void Adafruit_SharpMem::diteringMode(bool on) {
  sendCmd(SHARPMEM_CMD_DITHER,on);
}
/**************************************************************************/
/*!
    @brief Configure Panel Bits
*/
/**************************************************************************/
void Adafruit_SharpMem::panelBits(uint8_t bits) {
  sendCmd(SHARPMEM_CMD_PRTCLSEL,bits);
}

/**************************************************************************/
/*!
    @brief Enable Ditering 
*/
/**************************************************************************/
void Adafruit_SharpMem::movieMode(bool on) {
  sendCmd(SHARPMEM_CMD_MOVIE,on);
}

/**************************************************************************/
/*!
    @brief Transfer Data from TCON to Panel
*/
/**************************************************************************/
void Adafruit_SharpMem::transferPanel() {
  digitalWrite(_cs, LOW);
  digitalWrite(_rd, HIGH);
  digitalWrite(_rs, LOW);

  writeParallelBus(SHARPMEM_CMD_DISP);

  //Wait the panel transfer to complete
  while(digitalRead(_ack) == 0);

  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    @brief Send CMD with parameter
*/
/**************************************************************************/
void Adafruit_SharpMem::sendCmd(uint8_t cmd, uint8_t parameter) {
  digitalWrite(_cs, LOW);
  digitalWrite(_rd, HIGH);
  
  digitalWrite(_rs, LOW);
  writeParallelBus(cmd);

  digitalWrite(_rs, HIGH);
  writeParallelBus(parameter);
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    @brief Send CMD without parameter
*/
/**************************************************************************/
void Adafruit_SharpMem::sendCmd(uint8_t cmd) {
  digitalWrite(_cs, LOW);
  digitalWrite(_rd, HIGH);
  digitalWrite(_rs, LOW);

  writeParallelBus(cmd);

  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    @brief Send data array
*/
/**************************************************************************/
void Adafruit_SharpMem::sendCmdAndData(uint8_t cmd, uint8_t* data, uint32_t length) {
  digitalWrite(_cs, LOW);
  digitalWrite(_rd, HIGH);
  digitalWrite(_rs, LOW);

  writeParallelBus(cmd);

  digitalWrite(_rs, HIGH);
  for (uint32_t i = 0; i < length; ++i)
  {
    //Originally I used digital Write to control the WR pins, but it is too slow.
    //The commented out lines are here to describe the beheavor of the register write.
    //am_hal_gpio_fastgpio will not be faster because the caching nature of the MCU
    //am_hal_gpio_fastgpio_set might be faster then the data even though it is execute after bus data write
    //which will be a problem.

    //am_hal_gpio_fastgpio_clr(_wr);
    //digitalWrite(_wr, LOW);
    *wtca = 0x40000FF;
    //writeParallelBus(data[i]);
    *wtsa = (uint32_t) data[i];
    //am_hal_gpio_fastgpio_set(_wr);
    //digitalWrite(_wr, HIGH);
    *wtsa = 0x4000000;
  }

  digitalWrite(_cs, HIGH);
}


/**************************************************************************/
/*!
    @brief Write 8bit to parallel bus
*/
/**************************************************************************/



void Adafruit_SharpMem::writeParallelBus(uint8_t data) {
    //Originally I used digital Write to control the WR pins, but it is too slow.
    //The commented out lines are here to describe the beheavor of the register write.
    //am_hal_gpio_fastgpio will not be faster because the caching nature of the MCU
    //am_hal_gpio_fastgpio_set might be faster then the data even though it is execute after bus data write
    //which will be a problem.

  //digitalWrite(_wr,LOW);
  //am_hal_gpio_fastgpio_clr(_wr);
  *wtca = 0x40000FF; //Also clear bus data
  *wtsa = (uint32_t) data;
  *wtsa = 0x4000000; 
  //digitalWrite(_wr,HIGH);
  //am_hal_gpio_fastgpio_set(_wr);
}

