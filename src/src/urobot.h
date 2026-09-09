 /***************************************************************************
 * 
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
 
#ifndef USTATE_H
#define USTATE_H

#include <HardwareSerial.h>
#include <stdint.h>
#include "main.h"

#define MAX_DATE_LENGTH 30

// #include <ADC.h> // better ADC package, but not needed here

class URobot // : public USubss
{
public:
  /**
   * Setup */
  void setup();
  /**
   * decode command for this unit */
  bool decode(const char * buf); // override;
  /**
   * Send list of available commands */
  void sendHelp();
  /**
   * Checks for state change.
   * e.g. alive LED and battery voltage */
  void tick();
  /**
   * load from flash */
  void eePromLoad();
  /**
   * save to flash */
  void eePromSave();
  /**
   * Turn DC power off after a while
   * \param after is suspend tile in seconds
   * also triggers state change to shut down PC */
  void powerOff(float after);
  /**
   * Turn DC power off (now) */
  void powerOff();
  /**
   * Turn DC power on */
  void powerOn();
  /**
   * Get SVN revision number for this file */
  int getLatestRevisionNumber()
  {
    return rev;
  };
  /**
   * Get SVN revision number for this file */
  void setRevisionNumber(const char * versionString);
  /**
   * find collective version for all stationary class-based library files */
  void setVersion();
  /**
   * Get SVN revision number for this file */
  const char * getRevisionDate();
  /**
   * send SVN revision number and date to USB */
  void sendVersion();
  /**
   * get robot name */
  const char * getRobotName()
  {
    return robotname[deviceID];
  }
//
  bool robotIDvalid()
  {
    return deviceID > 0 and deviceID < MAX_ROBOT_NAMES;
  }
  
  float batteryVoltage;

private:
    float batVoltIntToFloat;

public:
  /// device ID (gives also nickname)
  int deviceID = 0;
  int robotHWversion = 9;
  char pcbVersion[32] = "6.3";
  /// hardware version
  /* hw 6 is for version 4.0 (teensy 3.5) (red PCB)
   * hw 7 is for version 5.1 (teensy 4.1) (purple PCB)
   * hw 8 is for version 5.0 (teensy 4.1) (green PCB with build-in display)
   * hw 9 is for version 6.3 (teensy 4.1) (blue PCB with build-in display)
   * */
  /// and robot type
  static const int MAX_NAME_LENGTH = 32;
  char deviceName[MAX_NAME_LENGTH] = "Basebot";
  char revDate[32] = "none";
  int rev = 0;
  int missionStart = 0; // used as a boolean, but set to true (!=0) in 5 samples, when activated
  float lowBatteryLevel = 10.2; // NB! should be 10.2 to keep LiPo battery safe
private:
  int tickCnt = 0;
  const char * versionh = "$Id: urobot.h 37 2025-08-28 18:52:47Z jcan $";
  const char * versioncpp = nullptr;
  // used on Teensy 4.1 only
  void setBatVoltageScale();
  // AD mode (10 or 12 bit)
  const int useADCresolution = 12;

  
private:
  static const int MAX_ROBOT_NAMES = 151;
  const char * robotname[MAX_ROBOT_NAMES] = 
  { "invalid", // 0
    "Emma",
    "Sofia",
    "Ida",
    "Freja",
    "Clara", // 5
    "Laura",
    "Anna",
    "Ella",
    "Isabella",
    "Karla", // 10
    "Alma",
    "Josefine",
    "Olivia",
    "Alberte",
    "Maja", // 15
    "Sofie", // 16
    "Mathilde",
    "Agnes",
    "Lily",
    "Caroline", // 20
    "Liva",
    "Emily",
    "Sara",
    "Victoria",
    "Emilie", // 25
    "Mille",
    "Frida",
    "Marie",
    "Ellen",
    "Rosa", // 30
    "Lea",
    "Signe",
    "Filippa",
    "Julie",
    "Nora", // 35
    "Liv",
    "Vigga",
    "Nanna",
    "Naja",
    "Alba", // 40
    "Astrid",
    "Aya",
    "Asta",
    "Luna",
    "Malou", // 45
    "Esther",
    "Celina",
    "Johanne",
    "Andrea", // 49
    "Silje", // 50
    "Thea", 
    "Adriana", 
    "Dicte", 
    "Silke", 
    "Eva", // 55
    "Gry", 
    "Tania", 
    "Susanne", 
    "Augusta",  // 59
    "Birte", // 60
    "Dagmar",
    "Leonora",
    "Nova",
    "Molly",
    "Ingrid", // 65
    "Sigrid",
    "Nicoline",
    "Tilde",
    "Ronja",
    "Saga", // 70
    "Viola",
    "Emilia",
    "Cecilie",
    "Kim",
    "Clara", // 75
    "Mie",
    "Alex",
    "Melina",
    "Amanda",
    "Hannah", // 80
    "Jasmin",
    "Kaya",
    "Sally",
    "Cleo",
    "Solvej", // 85
    "Nadia",
    "Ronja",
    "Vera",
    "Mary", // 89
    "Hans", // 90 
    "Joe",
    "Stub",
    "Rumle",
    "Viking",
    "Ada", // 95
    "Falk",
    "Atlas",
    "Cuba",
    "Gia", // 99
    "Alfred", // 100
    "Oscar",
    "Coco",
    "Jack",
    "William",
    "Oliver", // 105
    "Aksel",
    "Arthur",
    "Kai",
    "Juniper",
    "Noa", //110
    "Emil",
    "August",
    "June",
    "Sky",
    "Nat", // 115
    "Birdie",
    "Gandalf",
    "Hugo",
    "Newton",
    "Theo", // 120
    "Liam",
    "Bode",
    "Verdi",
    "Dot",
    "Theodor", // 125
    "Lauge",
    "Frederik",
    "Liz",
    "Anker",
    "Adam", // 130
    "Loui",
    "Storm",
    "Navy",
    "Johan",
    "Konrad", // 135
    "Flora",
    "Erik",
    "Albert",
    "Murphy", // 139
    "Mars", // 140
    "Uranus",
    "Saturn",
    "Venus",
    "Pluto",
    "Neptune", // 145
    "Titan",
    "Orion",
    "Mercury",
    "Psyche", // 149
    "Tietgen" // 150
  };
  
private:
  bool batteryOff = false;
  int batLowCnt = 0;
};




extern URobot robot;

#endif
