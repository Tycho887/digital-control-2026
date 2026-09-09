#line 1 "/Users/michaeljohansen/Autumn2026/digital-control/week-01/basebot_6/src/ueeconfig.cpp"
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
#include "ueeconfig.h"
#include "avr_functions.h"
#include "umotor.h"
#include "uusb.h"

#include "urobot.h"
#include "uencoder.h"
#include "uimu2.h"
#include "udisplay.h"

const int EEPROM_SIZE = 4284;

/**
 * Global configuration */
EEConfig eeConfig;


/** initialize */
EEConfig::EEConfig()
{
  sbufCnt = 0;
  stringConfig = false;
  config = NULL;
}

void EEConfig::setup()
{
  eePromLoadStatus (false);
  versioncpp = "$Id: ueeconfig.cpp 28 2024-11-15 12:18:05Z jcan $";
}


bool EEConfig::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "eer", 3) == 0)
  {  // load from flash to ccurrent configuration
    eePromLoadStatus(false);
  }
  else if (strncmp(buf, "eew", 3) == 0)
  {  // load from flash to ccurrent configuration
    eePromSaveStatus(false);
  }
  else if (strncmp(buf, "eeusb", 5) == 0)
  {  // load from flash to ccurrent configuration
    const int MSL = 2048;
    uint8_t s[MSL];
    // set buffer where to save the the configuration (binary)
    setStringBuffer(s, false);
    // save the config to this buffer
    eePromSaveStatus(true);
    // convert binary configuration to string and send to USB
    stringConfigToUSB(nullptr, 0);
    // clear the buffer (to avoid invalid pointer)
    clearStringBuffer();
  }
  else
    used = false;
  return used;
}

void EEConfig::stringConfigToUSB(const uint8_t * configBuffer, int configBufferLength)
{
  int length = configBufferLength;
  const uint8_t * cfg = configBuffer;
  if (cfg == NULL)
  {
    cfg = config;
    length = configAddrMax;
  }
  if (cfg == NULL)
  {
    usb.send("%% error: configuration not generated as string\n");
  }
  else
  {
    const int MSL = 110;
    char s[MSL];
    char * p1 = s;
    int n = 0;
    int line = 0;
    int i = 0;
    while (i < length)
    {
      snprintf(s, MSL, "#cfg%02d", line++);
      n += strlen(p1);
      p1 = &s[n];
      for (int j = 0; j < 32; j++)
      {
        snprintf(p1, MSL-n, " %02x", cfg[i]);
        n += strlen(p1);
        p1 = &s[n];
        i++;
        if (i >= length)
          break;
      }
      if (i < length)
      { // not finished, so add a linefeed escape character
        // to make it easier to copy-paste into code
        *p1++ = '\\';
      }
      *p1++ = '\n';
      *p1++ = '\0';
      usb.send(s);
      if (n > MSL - 4)
        usb.send("%% stringConfigToUSB error\n");
      p1 = s;
      n = 0;
    }
  }
}
//
void EEConfig::sendHelp()
{
  const int MRL = 250;
  char reply[MRL];
  usb.send("%% EE (configuration flash) --------\r\n");
  snprintf(reply, MRL, "%% -- \teew \tSave configuration to EE-Prom (flash)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \teer \tRead configuration from EE-Prom\r\n");
  usb.send(reply);
}
//
//
  
void EEConfig::push32(uint32_t value)
{
  //   const int MSL = 100;
  //   char s[MSL];
  //   snprintf(s, MSL, "%% ee saved: at %lu, value %lu\r\n", eePushAdr, value);
  //   usb.send(s);
  //
  if (stringConfig)
  {
    if (config != NULL)
      memcpy(&config[configAddr], &value, 4);
  }
  else
  {
    eeprom_busy_wait();
    eeprom_write_dword((uint32_t*)configAddr, value);
  }
  configAddr += 4;
  if (configAddr > configAddrMax)
    configAddrMax = configAddr;
}

////////////////////////////////////////////////

void EEConfig::pushByte(uint8_t value)
{ // save one byte
  if (stringConfig)
  {
    if (config != NULL)
      config[configAddr] = value;
  }
  else
  {
    eeprom_busy_wait();
    eeprom_write_byte((uint8_t*)configAddr, value);
  }
  configAddr++;
  if (configAddr > configAddrMax)
    configAddrMax = configAddr;
}

////////////////////////////////////////////////

void EEConfig::pushWord(uint16_t value)
{ // save one byte
  if (stringConfig)
  {
    if (config != NULL)
      memcpy(&config[configAddr], &value, 2);
  }
  else
  {
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)configAddr, value);
  }
  configAddr += 2;
  if (configAddr > configAddrMax)
    configAddrMax = configAddr;
}

//////////////////////////////////////////////

uint32_t EEConfig::read32()
{
  uint32_t b;
  if (stringConfig)
  {
    if (config != NULL)
      b = *(uint32_t *)&config[configAddr];
    else
      b = 0;
  }
  else
  {
    b = eeprom_read_dword((uint32_t*)configAddr);
  }
  configAddr += 4;
  return b;
}

/////////////////////////////////////////////////

uint8_t EEConfig::readByte()
{
  uint8_t b;
  if (stringConfig)
  {
    if (config != NULL)
      b = config[configAddr];
    else
      b = 0;
  }
  else
  {
    b = eeprom_read_byte((uint8_t*)configAddr);
  }
  configAddr++;
  return b;
}

/////////////////////////////////////////////////

uint16_t EEConfig::readWord()
{
  uint16_t b;
  if (stringConfig)
  {
    if (config != NULL)
      b = *(uint16_t *)&config[configAddr];
    else
      b = 0;
  }
  else
  {
    b = eeprom_read_word((uint16_t*)configAddr);
  }
  configAddr += 2;
  return b;
}
  
///////////////////////////////////////////////////

void EEConfig::eePromSaveStatus(bool toUSB)
{ // reserve first 4 bytes for dword count
  const int MSL = 100;
  char s[MSL];
  // debug
  // debug end
  stringConfig = toUSB;
  // save space for used bytes in configuration
  configAddr = 4;
  configAddrMax = 4;
  // save revision number
  push32(robot.getLatestRevisionNumber());
  // main values
  robot.eePromSave();
  // save gyro zero offset
  imu2.eePromSave();
  // encoder calibration values
  encoder.eePromSave();
  motor.eePromSave();
  // then save length
  uint32_t cnt = configAddr;
  configAddr = 0;
  if (not robot.robotIDvalid())
  {
    // ignore ee-prom at next reboot
    push32(0);
    snprintf(s, MSL, "%% EE-prom D set to default values at next reboot\r\n");
  }
  else
  {
    push32(cnt);
    if (toUSB)
      snprintf(s, MSL, "%% Send %lu config bytes (of %d) to USB\r\n", cnt, EEPROM_SIZE);
    else
      snprintf(s, MSL, "%% Saved %lu bytes (of %d) to EE-prom D\r\n", cnt, EEPROM_SIZE);
  }
  configAddr = cnt;
  // tell user
  usb.send(s);
}

//////////////////////////////////////////////////

void EEConfig::eePromLoadStatus(bool from2Kbuffer)
{ 
  const int MSL = 100;
  char s[MSL]; 
  //eePushAdr = 0;
  stringConfig = from2Kbuffer;  
  configAddr = 0;
  uint32_t cnt = read32();
  loadedRrev = read32();
  if (cnt == 0 or cnt >= uint32_t(maxEESize) or loadedRrev < 10)
  { // > 15000 means that it is an old Regbot configuration
    snprintf(s, MSL, "%%# No or old configuration - save with eew (config size=%lu, rev=%lu)\r\n", cnt, loadedRrev);
    usb.send(s);
    return;
  }
  if (false and loadedRrev != robot.getLatestRevisionNumber())
  {
    snprintf(s, MSL, "%%# configuration from old SW version now:%d != ee:%d - continues (use eew to update)\r\n", robot.getLatestRevisionNumber(), (int)loadedRrev);
    usb.send(s);
  }
  robot.eePromLoad();
  if (robot.robotIDvalid())
  { // gyro zero value
    imu2.eePromLoad();
    encoder.eePromLoad();
    motor.eePromLoad();
    motor.setup();
    // note changes in ee-prom size
    if (cnt != (uint32_t)configAddr)
    {
      snprintf(s, MSL, "%%# configuration size has changed! %lu != %d used bytes\r\n", cnt, configAddr);
      usb.send(s);
    }
  }
  else
  {
    usb.send("%% skipped major part of ee-load, as ID == 0\n");
  }
}
  
/////////////////////////////////////////////////

bool EEConfig::pushBlock(const char * data, int dataCnt)
{
  if (getAddr() + dataCnt < 2048 - 2)
  {
    busy_wait();
    write_block(data, dataCnt);
    return true;
  }
  else
    return false;
}

bool EEConfig::readBlock(char * data, int dataCnt)
{
  if (getAddr() + dataCnt < 2048 - 2)
  {
    busy_wait();
    for (int n = 0; n < dataCnt; n++)
    {
      data[n] = readByte();
    }
    return true;
  }
  else
    return false;
}
