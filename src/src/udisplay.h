/***************************************************************************
 *   Copyright (C) 2024 by DTU
 *   jcan@dtu.dk
 * 
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */


#ifndef UDISPLAY_H
#define UDISPLAY_H

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306_mod.h"
#include <Wire.h>

class UDisplay
{
public:
  /**
   * Display size */
  static const int SCREEN_WIDTH = 128; // OLED display width, in pixels
  static const int SCREEN_HEIGHT = 32; // OLED display height, in pixels
  static const int OLED_RESET    =  -1; // Reset pin # (or -1 if sharing Arduino reset pin)
  static const int SCREEN_ADDRESS = 0x3C; ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  /**
   * setup */
  void setup();
  /**
   * Do sensor processing - at tick time */
  void tick();  
  /**
   * Show this line as the first on the small display */
  void setLine(const char * line);
  /**
   * Flad to stop display update during time critical missions
   * and fast sample times.
   * Update takes about 1.5ms */
  bool useDisplay = true;
  /** time since last boot in seconds */
  float time = 0; // sec
  const char * versionh = "$Id: udisplay.h 28 2024-11-15 12:18:05Z jcan $";
  const char * versioncpp = nullptr;

protected:
  /**
   * simple tic counter */
  int tickCnt;
  /**
   * Object to handle the text font and graphics */
  Adafruit_SSD1306 * dss = nullptr;
  /** Buffer for display content */
  static const int MAX_LINE_LENGTH = 40;
  char lineName[MAX_LINE_LENGTH];
  char lineFree[MAX_LINE_LENGTH];
  char lineState[MAX_LINE_LENGTH];
  int updFastCol = 0;
  int updSlowLine = 0;
  int updSlowCol = 0;
};

extern UDisplay display;

#endif
