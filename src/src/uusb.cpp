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

#include <core_pins.h>
#include <usb_serial.h>
#include <wiring.h>
#include "main.h"
#include "uusb.h"
#include "urobot.h"

UUSB usb;

void UUSB::setup()
{ // init USB connection (parameter is not used - always 12MB/s)
//   Serial.begin ( 115200 ); // USB init serial
  versioncpp = "$Id: uusb.cpp 33 2024-12-22 16:08:11Z jcan $";
}

void UUSB::tick()
{ // check for messages
  usbIsUp = not bitRead(USB1_PORTSC1,7);
  bool gotLine = handleIncoming();
  if (gotLine)
  {
    robot.decode(usbRxBuf);
  }
}

bool UUSB::send(const char* str)
{ // just a shorthand
  if (usbIsUp)
    Serial.print(str);
  return usbIsUp;
}

void UUSB::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("%% USB -------\r\n");
  snprintf(reply, MRL, "%% -- \techo A \tSend received characters back as received. A=0 not, A=1 yes (is=%d)\r\n", localEcho);
  usb.send(reply);
}

void printLog();


bool UUSB::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "echo", 4) == 0)
  {
    const char * p1 = &buf[4];
    if (*p1 < ' ')
      // no parameter
      localEcho = true;
    else
      // get value
      localEcho = strtol(p1, (char**)&p1, 10) != 0;
  }
  else
    used = false;
  return used;
}

////////////////////////////////////////////////////////////////

bool UUSB::handleIncoming()
{
  int n = 0, m;
  bool dataReceived = false;
  // get number of available chars in USB buffer
  m = usb_serial_available();
  if (m > 0)
  { // get characters
    for (int i = 0; i < m; i++)
    { // get pending characters, one at a time
      n = usb_serial_getchar();
      if (localEcho)
      { // echo characters back
        if (usbRxBufCnt == 0)
          Serial.print("% ");
        Serial.print(char(n));
      }
      if (n < 0)
        break;
      // limit to usable part of 7-bit ASCII
      if (n >= '\n' and n < 0x80)
      { // could echo received chars to sender
        // Serial.print(char(n));
        usbRxBuf[usbRxBufCnt++] = n;
        dataReceived = n == '\n' or n == '\r';
      }
    }
    if (usbRxBufCnt >= RX_BUF_SIZE - 2)
    { // buffer too full, remove
      Serial.println("%% USB receive buffer overflow - discarded");
      usbRxBufCnt = 0;
      dataReceived = false;
    }
    if (dataReceived)
    { // command is in buffer, terminate C string
      usbRxBuf[usbRxBufCnt] = '\0';
      // prepare for next command
      usbRxBufCnt = 0;
    }
  }
  return dataReceived;
}


