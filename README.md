# Modified Adafruit SHARP Memory Display for LS060S2UD01 6" Sharp Memory LCD

This is the modified version of Adafruit SHARP Memory Display Arduino Library for Arduino(Sparkfun Artemis Module) and LS060S2UD01
It is modified to work with 8bit data bus and 1 bit color (B/W only) mode.

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

# Dependencies
  The library uses 8-bit width parallel Bus to communicate with the display. The control signal pins uses Arduino HAL like pinMode() or digitalWrite() function, but for the performance sake, the 8-bit data bus and WR pin were controlled directly by pin registers which is highly customized for Sparkfun Artemis Module (Apoll3b MCU). It is assuming the bus are GPIO0 ~ 7 and WR is GPIO 28. 

* [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)

