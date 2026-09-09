#line 1 "/Users/michaeljohansen/Autumn2026/digital-control/week-01/basebot_6/src/urobot.cpp"
 /***************************************************************************
 * 
 *   Copyright (C) 2024 by DTU                             *
 *   jcan@dtu.dk                                                    *
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
 
#include <stdio.h>
#include <core_pins.h>
#include "urobot.h"
#include "ueeconfig.h"
#include "uusb.h"
#include "umotor.h"
#include "uimu2.h"
#include "udisplay.h"
#include "uencoder.h"

// #define REV_ID "$Id: urobot.cpp 37 2025-08-28 18:52:47Z jcan $"
// adjust REV_MINOR to save new SVN revision number
#define REV_MINOR 3

/**
 * create state object */
URobot robot;

void URobot::setup()
{ // hold power on
  pinMode ( PIN_POWER_ROBOT, OUTPUT );
  powerOn();
  pinMode ( PIN_START_BUTTON, INPUT );
  pinMode ( PIN_LED_STATUS, OUTPUT );
  eeConfig.eePromLoadStatus(false);
  display.setup();
//   ad.setup();
  // Using 3.3V reference, resistor divider 2x47k and 4.7k Ohm, thus:
  batVoltIntToFloat = 3.3 / 4096 * (47.0/2 + 4.7)/4.7;
  //
  analogReadResolution(useADCresolution);
  // AD converter settings
//   adc.adc0->calibrate();
//   adc.adc0->wait_for_cal();
//   adc.adc1->calibrate();
//   adc.adc1->wait_for_cal();
//   //
//   adc.adc0->setResolution ( useADCresolution);
//   adc.adc1->setResolution ( useADCresolution);
//   // support for 3.3V only  and HW63 is designed for 3.3V ref
//   adc.adc0->setReference ( ADC_REFERENCE::REF_3V3);
//   adc.adc1->setReference ( ADC_REFERENCE::REF_3V3);
//   adc.adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED);
//   adc.adc1->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED);
  versioncpp = "$Id: urobot.cpp 37 2025-08-28 18:52:47Z jcan $";
}

void URobot::setVersion()
{ // find the library with the newest SVN revision number
  rev = 1;
  setRevisionNumber(versionh);
  setRevisionNumber(versioncpp);
  setRevisionNumber(encoder.versionh);
  setRevisionNumber(encoder.versioncpp);
  setRevisionNumber(imu2.versionh);
  setRevisionNumber(imu2.versioncpp);
  setRevisionNumber(usb.versionh);
  if (true)
  {
    setRevisionNumber(usb.versioncpp);
    setRevisionNumber(motor.versionh);
    setRevisionNumber(motor.versioncpp);
    setRevisionNumber(display.versionh);
    setRevisionNumber(display.versioncpp);
  }
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "%% Robot %d %s (PCB version %s, SVN version %d, battery %.1f V)\r\n",
           deviceID,
           getRobotName(),
           pcbVersion,
           rev,
           batteryVoltage);
  usb.send(s);
}


void URobot::tick()
{ // safety things
  if (tickCnt == 20)
    setVersion();
  // blink "alive" LED
  uint32_t ms = millis() % 1000;
  if (ms < 10)
    digitalWriteFast(PIN_LED_STATUS, HIGH);
  else
    digitalWriteFast(PIN_LED_STATUS, LOW);
  //
  // monitor battery voltage
  int batteryADvalue = analogRead(PIN_BATTERY_VOLTAGE);
  batteryVoltage = float(batteryADvalue) * batVoltIntToFloat;
  // check battery (and turn off if needed)
  if ((batteryVoltage < lowBatteryLevel and batteryVoltage > 6.5))
  {
    batLowCnt++;
    if (batLowCnt > 10000)
    {
      powerOff();
      // delay for power to drop
      batLowCnt = -800;
    }
  }
  else
    batLowCnt = 0;
  if (missionStart)
    missionStart--;
  tickCnt++;
}

// printLog is called from here, but is defined in 'basebot_6.cpp'
void printLog();

void doReboot()
{ // disconnect USB - to inform PC/Raspberry of the reboot
  // maybe a bad idea? // chr
  USB1_USBCMD = 0; // disconnect USB
  delay(50);       // enough time for USB hubs/ports to detect disconnect
  SCB_AIRCR = 0x05FA0004;
}

bool URobot::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "setidx ", 7) == 0)
  {
    const char * p1 = &buf[7];
    deviceID = strtol(p1, NULL, 10);
    if (not robotIDvalid() and deviceID != 0)
      deviceID = 0;
  }
  else if (strncmp(buf, "start", 5) == 0)
  { // stop motors now
    //     userMission.missionStop = false;
    missionStart = 5;
    //     usb.send("start\n");
    usb.send("%% starting\r\n");
  }
  else if (strncmp(buf, "sethw ", 6) == 0)
  { // set hardware type
    const char * p1 = &buf[6];
    robotHWversion = strtol(p1, NULL, 10);
  }
  else if (strncmp(buf, "reboot", 6) == 0)
  { // reboot Teensy
    // _reboot_Teensyduino_(); // reloads software - require running loader
    // _restart_Teensyduino_(); // fails to compile
    // call the global function
    //::setup(); // problems with loop still running
    doReboot();
  }
  else if (strncmp(buf, "power", 5) == 0)
  { // reboot Teensy
    const char * p1 = &buf[5];
    char * p2;
    int v = strtol(p1, &p2, 10);
    if (p1 == p2 or v > 0)
    {
      powerOn();
      usb.send("% power on\n\t");
    }
    else
    {
      powerOff();
      usb.send("% power off\n\t");
    }
  }
  else if (strncmp(buf, "help", 4) == 0)
  { // stop motors now
    sendHelp();
  }
  else if (strncmp(buf, "version", 7) == 0)
  { // stop motors now
    sendVersion();
  }
  else if (strncmp(buf, "log", 3) == 0)
  {
    printLog();
  }
  //   else if (eeConfig.decode(buf)) {}
  else if (motor.decode(buf)) {}
  else if (imu2.decode(buf)) {}
  else if (encoder.decode(buf)) {}
  else if (usb.decode(buf)) {}
  else if (eeConfig.decode(buf)) {}
  else
    used = false;
  if (not used)
  {
    Serial.print("%% noone used ");
    Serial.println(buf);
  }
  return used;
}

void URobot::sendHelp()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "%% Available commands from %s:\r\n", deviceName);
  usb.send(reply);
  usb.send("%% Robot settings -------\r\n");
  snprintf(reply, MRL, "%% -- \tsetidx N \tSet ID to N (sets robot name) (id=%d)\r\n", deviceID);
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tsethw P \tSet PCB version to P (is=%d) 9=PCB6.3\r\n", robotHWversion);
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tstart \tStart (same as start button)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tversion \tGet version in SVN (is %d)\r\n", getLatestRevisionNumber());
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tpower 0/1 \t12V power 1=on (is %d)\r\n", not batteryOff);
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \treboot \tReboot Teensy\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tlog \tPrint log\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \thelp \tSend on-line help\r\n");
  usb.send(reply);
  // from other modules
//   eeConfig.sendHelp();
  usb.sendHelp();
  motor.sendHelp();
  encoder.sendHelp();
  imu2.sendHelp();
  eeConfig.sendHelp();
}

void URobot::sendVersion()
{
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "version %d %s\n", getLatestRevisionNumber(), getRevisionDate());
  usb.send(s);
}

void URobot::powerOn()
{ // no warning, just off
  digitalWriteFast(PIN_POWER_ROBOT, HIGH);
  batteryOff = false;
  display.setLine(deviceName);
//   usb.send("%% URobot:: power on\r\n");
}

void URobot::powerOff()
{ // no warning, just off
  digitalWriteFast(PIN_POWER_ROBOT, LOW);
  batteryOff = true;
  display.setLine("Power off");
}

/**
 * Get and set SVN revision number */
void URobot::setRevisionNumber(const char * versionString)
{ // find space after filename
  if (versionString != nullptr)
  {
    const char * p1 = strchr(&versionString[7], ' ');
    if (p1 != nullptr)
    {
      int vn = strtol(p1, (char **)&p1, 10);
      if (vn > rev and p1 != nullptr)
      {
        rev = vn;
        strncpy(revDate, ++p1, 22);
        revDate[22] = '\0';
      }
    }
  }
  else
    usb.send("% a Version string is not populated\n");
}

const char * URobot::getRevisionDate()
{
  return revDate;
}


void URobot::eePromLoad()
{
  deviceID = eeConfig.readWord();
  if (not robotIDvalid() and deviceID != 0)
    deviceID = 0;
  robotHWversion = eeConfig.readByte();
  switch (robotHWversion)
  {
    case 6: strncpy(pcbVersion, "4.2", 32); break;
    case 7: strncpy(pcbVersion, "5.1", 32); break;
    case 8: strncpy(pcbVersion, "8.5", 32); break;
    case 9: strncpy(pcbVersion, "6.3", 32); break;
    case 10: strncpy(pcbVersion, "Scorpi", 32); break;
    default: /*strncpy(pcbVersion, "??", 32);*/ break;
  }
  // eeConfig.skipAddr(MAX_NAME_LENGTH);
  //
  // const int MSL = 100;
  // char s[MSL];
  // snprintf(s, MSL, "%% Robot %d %s (PCB version %s, SW %d)\r\n", deviceID, getRobotName(), pcbVersion, rev);
  // usb.send(s);
}

void URobot::eePromSave()
{
  eeConfig.pushWord(deviceID);
  eeConfig.pushByte(robotHWversion);
}

