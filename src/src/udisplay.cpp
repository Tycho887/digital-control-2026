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


#include "main.h"
#include "udisplay.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uimu2.h"
#include "umotor.h"

/// Small display handling
UDisplay display;

void UDisplay::setup()
{
  if (dss == nullptr)
  {
    dss = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET); //, 1000000, 1000000);
  }  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!dss->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    usb.send("%% UDisplay::setup: SSD1306 allocation failed\n");
  }
  Wire.setClock(1000000);
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //
  // display all 16 elements (4 lines each 32 pixels wide (32 bytes send in each block)
  for (int i=0; i < 4; i++)
  {
    for (int c = 0; c < 8; c++)
      dss->display(i, c);
  }
  versioncpp = "$Id: udisplay.cpp 47 2025-10-28 15:05:02Z jcan $";
}


void UDisplay::tick()
{
  tickCnt++;
  time = millis() / 1000.0;
  if (useDisplay)
  {
    // usb.send("# display update\n");
    bool fast = int(millis()/10) % 3 == 0;
    bool slow = int(millis()/10) % 16 == 0;
    if (fast or slow)
    {
      dss->clearDisplay();
      dss->setCursor(42,16);             // Start at row 8 (line 2), 16 = row 3
      dss->setCursor(0,8);             // Start at row 8 (line 2), 16 = row 3

  //     dss->setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  //     dss->println(3.141592);
      
      dss->setTextSize(2);             // Draw 1x or 2X-scale text
      dss->setTextColor(SSD1306_WHITE);
      snprintf(lineName, MAX_LINE_LENGTH, "%d %s\n", robot.deviceID, robot.getRobotName());
      lineName[10] = '\0';
      dss->print(F(lineName));
      dss->setTextSize(1);             // Normal 1:1 pixel scale
      dss->setTextColor(SSD1306_WHITE);        // Draw white text
      dss->setCursor(0,24);             // Start at row 24
      char useImu = 'I';
      if (imu2.imuAvailable > 0)
        useImu = 'i';
      char m1ok = 'm', m2ok = 'm';
      if (not motor.m1ok)
        m1ok = 'M';
      if (not motor.m2ok)
        m2ok = 'M';
      char usbConn = 'U';
      if (usb.usbIsUp)
        usbConn = 'u';
      snprintf(lineState, MAX_LINE_LENGTH, "%5.1f %4.1fV %c%c%c%c %2d",
               float(millis() % 100000) / 100.0,
               robot.batteryVoltage,
               m1ok, m2ok, useImu, usbConn,
               robot.rev);
      lineState[MAX_LINE_LENGTH-1] = '\0';
      dss->println(F(lineState));    
      dss->setCursor(0,0);             // Start at top-left corner
      dss->println(F(lineFree));
      if (slow)
      {
        // usb.send("# slow disp update\n");
        dss->display(updSlowLine, updSlowCol++);
        if (updSlowCol >= 8)
        {
          updSlowCol = 0;
          updSlowLine = (updSlowLine + 1) % 3;
        }
      }
      else if (fast)
      {
        // usb.send("# fast disp update\n");
        dss->display(3, updFastCol);
        updFastCol = (updFastCol + 1) % 8;
      }
    }
  }
}

void UDisplay::setLine(const char* line)
{
  strncpy(lineFree, line, MAX_LINE_LENGTH);
  lineFree[MAX_LINE_LENGTH-1] = '\0';
}

