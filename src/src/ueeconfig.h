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

#ifndef REGBOT_EESTRING_H
#define REGBOT_EESTRING_H

#include <string.h>
#include <stdlib.h>
#include <HardwareSerial.h>
#include "main.h"
// #include "ucommand.h"
#include "uusb.h"



class EEConfig //: public USubss
{
public:
  // constructor
  EEConfig();
  /**
   * set PWM port of frekvens */
  void setup();
  /**
   * Send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf);
  /** send configuration to USB
   * \param configBuffer set to NULL to use the just saved configuration or to another configuration to fetch.
   * Sends the configuration as byte hex code  */
  void stringConfigToUSB(const uint8_t * configBuffer, int configBufferLength);
  /** set in use flag and clear buffer */
  inline void setStringBuffer(uint8_t * string2kBuffer,  bool initializeEEprom)
  {
    config = string2kBuffer;
    configAddr = 0;
    configAddrMax = 0;
    if (initializeEEprom)
      eeprom_initialize();
  }
  inline void clearStringBuffer()
  {
    config = NULL;
  }
  /**
   * Load configuration from either a config string or from eeProm (flash) 
   * dependent on the "stringConfig" flag
   * \param from2Kbuffer (not used - inherited from regbot code).
   * */
  void eePromLoadStatus(bool from2Kbuffer);
  /**
   * Save configuration to eeProm (flash) or sent configuration to USB in hex format.
   * \param toUSB set to true to read to USB, or false to save to eeProm */
  void eePromSaveStatus(bool toUSB);
  
public:
  uint32_t loadedRrev = 0;
  /** save a 32 bit value */
  void push32(uint32_t value);
  /** save a byte */
  void pushByte(uint8_t value);
  /** save a word in configuration stack */
  void pushWord(uint16_t value);
  /** get a 32 bit integer from configuration stack */
  uint32_t read32();
  /** get a byte from configuration stack */
  uint8_t readByte();
  /** get a 16 bit integer from configuration stack */
  uint16_t readWord();
  /**
   * Add a block of data to ee-Prom area 
   * \param data is the byte data block,
   * \param dataCnt is the number of bytes to write 
   * \returns true if space to write all. */
  bool pushBlock(const char * data, int dataCnt);
  /**
   * Read a number of bytes to a string 
   * \param data is a pointer to a byte array with space for at least dataCnt bytes.
   * \param dataCnt is number of bytes to read
   * \returns true if data is added to data array and false if 
   * requested number of bytes is not available */
  bool readBlock(char * data, int dataCnt);
  
  /** save a 32 bit float to configuration stack */
  inline void pushFloat(float value)
  {
    union {float f; uint32_t u32;} u;
    u.f = value;
    push32(u.u32);
  }
  // read 32 bit as float from configuration stack
  inline float readFloat()
  {
    union {float f; uint32_t u32;} u;
    u.u32 = read32();
    return u.f;  
  }
  /** write a word to a specific place in configuration stack
   * typically a size that is not known before pushing all the data */
  inline void write_word(int adr, uint16_t v)
  {
    if (not stringConfig)
      eeprom_write_word((uint16_t*)adr, v);
    else if (config != NULL)
    {
      memcpy(&config[adr], &v, 2);
    }
    else
      usb.send("# failed to save word\n");
    if (adr > configAddr - 2)
      configAddr = adr + 2;
  }
  /**
   * a busy wait if the flash write system is busy */
  inline void busy_wait()
  {
    if (not stringConfig)
    {
      eeprom_busy_wait();
    }
  }
  /** push a block of data to the configuration stack */
  inline void write_block(const char * data, int n)
  {
    if (not stringConfig)
    {
      eeprom_write_block(data, (void*)configAddr, n);
    }
    else
    {
      memcpy(&config[configAddr], data, n);
    }
    configAddr += n;
  }
  /** set the adress for the next push or read operation on the configuration stack */
  void setAddr(int newAddr)
  {
    configAddr = newAddr;
  }
  /** skip some bytes from the configuration stack
   * \param bytes is the number of bytes to skib. */
  void skipAddr(int bytes)
  {
    configAddr+=bytes;
  }
  /** get the address of the next push or read operation on the configuration stack */
  int getAddr()
  {
    return configAddr;
  }
  /**
   * Implement one of the hard-coded configurations 
   * \param hardConfigIdx is index to the hardConfig array, as defined in eeconfig.h and set in the constructor.
   * \param andToUsb is a debug flag, that also will return the just loaded configuration to the USB
   * */
  bool hardConfigLoad(int hardConfigIdx, bool andToUsb);

  /** current read/write adress in config array */
  int configAddr;
  const char * versionh = "$Id: ueeconfig.h 28 2024-11-15 12:18:05Z jcan $";
  const char * versioncpp = nullptr;

protected:
  /**
   * Get hard coded configuration string and load it into buffer in binary form
   * like for the real flash configuration memory. */
   int getHardConfigString(uint8_t * buffer, int configIdx);
  
  const int maxEESize = 4096;
  
private:
  /** full configuration buffer, as real eeProm 
   * is either NULL or points to a 2048 byte array */
  uint8_t * config;
  /** number of bytes written to string buffer */
  int sbufCnt;
  /** is string buffer in use - else flash is in use */
  bool stringConfig;
  /** highest number used in configuration in config array */
  int configAddrMax;
};

/**
 * Instans af ee og string config */
extern EEConfig eeConfig;


#endif
