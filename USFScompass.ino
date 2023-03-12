/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include <Wire.h>

#include "I2Cdev.h"
#include "USFSMAX.h"
#include "Sensor_cal.h"
#include "IMU.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"
#include "bitmaps.h"

#include <EEPROM.h>

#include <RotaryEncoder.h>
//#include "STM32encoder.h"

#include "MovingAverage.h"

//#define HAL_LPTIM_MODULE_ENABLED



int16_t position = 0;

RotaryEncoder encoder(ENC_A, ENC_B, BUTTONENC);

//STM32encoder enc(TIM2);

MovingAverage<float,32> filter;  //powers of 2 only

// Instantiate class objects
I2Cdev i2c_0(&SENSOR_0_WIRE_INSTANCE);
USFSMAX USFSMAX_0(&i2c_0, 0);
IMU imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Declare global scope utility functions

void GoToSleep();
void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM);
void DRDY_handler_0();
void SerialInterface_handler();
void encoderISR();
void encoderButtonISR();

int scroll = 0;


unsigned long previousMillis = 0;
const long interval = 100;

int Declination = 11;  // substitute your magnetic declination
//int declinationInt = 11;

unsigned long previousMillisScroll;
int scanLocation = 0;
int brightness = 175;
bool toggleButton = 1;
volatile bool pressed = 0;
int lastPosition = 0;

int letter = 0;
unsigned char displayBuffer[400];
unsigned char displayText[60] = { "                                                           " };
int invertedChars[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int selectedStart = 0;
unsigned char menuArray[6][20] = {
  { "Heading" }, { "Declination" }, { "Brightness" }, { "Altitude" }, { "Marker" }, { "Drive in circles!!" }
};

int menuArrayLengths[6] = { 7, 11, 10, 8, 6, 19 };

int displayedMenu = 0;  // 0 for compass  1 = Menu   2 = Calibrate

int hoveredMenuItem = 0;  // 1 = Heading   2 = Declination    3 = Brightness     4 = Calibrate
int selectedMenuItem = 0;
int menuItem = 0;  // 1 = Heading   2 = Declination    3 = Brightness     4 = Calibrate   5 = Marker
int marker = 180;
int markerFlag = 0;
int markerSetFlag = 0;
int oldPos = 0;

int calibratingFlag = 0;
int calConfirm = 0;

int showHeading = 1;
int showAltitude = 0;

int declinationSet = 0;

unsigned long delayTimer = 0;
unsigned long lastDelayTimer = 0;
int updateTime = 0;
int maxUpdateTime = 10000;
int readsBetweenMax = 0;
int lastReadsBetweenMax = 0;

unsigned long calTimer = 0;
unsigned long lastCalTimer = 0;

unsigned long readTime = 0;
unsigned long lastReadTime = 0;
int readRate = 0;

float pressureOffset = 5.2;

void setup() {

  // Open serial port
#ifdef SERIALSTUFF
  Serial.blockOnOverrun(true);
  Serial.begin(115200);
  delay(200);


  // Initialize USFSMAX_0 I2C bus

#endif
  Wire.setClock(10000);  // Set I2C clock speed to 100kHz cor configuration
  delay(50);
  Wire.begin();


// Do I2C bus scan if serial debug is active
#ifdef SERIAL_DEBUG  // Should see MAX32660 slave bus address (default is 0x57)
  i2c_0.I2Cscan();
#endif
  delay(10);

// Initialize USFSMAX_0
#ifdef SERIAL_DEBUG
  Serial.print("Initializing USFSMAX_0...");
  Serial.println("");
#endif



  USFSMAX_0.init_USFSMAX();
  // Configure USFSMAX and sensors
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);  // Set the I2C clock to high speed for run-mode data collection
  delay(20);




#ifdef SERIAL_DEBUG
  //Serial.println("USFXMAX_0 successfully initialized!");
  //Serial.println("");
  //sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the results
#endif

  // Calculate geomagnetic calibration parameters for your location (set in "config.h")
  Mv_Cal = M_V;                                     // Vertical geomagnetic field component
  Mh_Cal = M_H;                                     // Horizontal geomagnetic field component
  M_Cal = sqrt(Mv_Cal * Mv_Cal + Mh_Cal * Mh_Cal);  // Geomagnetic field strength
  Del_Cal = atan(Mv_Cal / Mh_Cal);                  // Geomagnetic inclination or "Dip" angle
#ifdef SERIALSTUFF
#if !defined(SERIAL_DEBUG) && !defined(MOTION_CAL_GUI_ENABLED)  // Print header for spreadsheet data collection
  Serial.print("Time");
  Serial.print(",");
  Serial.print("Heading (deg)");
  Serial.print(",");
  Serial.print("Pitch (deg)");
  Serial.print(",");
  Serial.print("Roll (deg)");
  Serial.print(",");
  Serial.print("Cal Status");
  Serial.println("");
#endif
#endif


  calibratingG[0] = 1;

  Start_time = micros();  // Set sketch start time

  sensor_cal.GyroCal();
  pinMode(INT_PIN, INPUT);

  pinMode(scan1, OUTPUT);
  pinMode(scan2, OUTPUT);
  pinMode(scan3, OUTPUT);

  pinMode(row0, OUTPUT);
  pinMode(row1, OUTPUT);
  pinMode(row2, OUTPUT);
  pinMode(row3, OUTPUT);
  pinMode(row4, OUTPUT);
  pinMode(row5, OUTPUT);
  pinMode(row6, OUTPUT);

  pinMode(blank, OUTPUT);


  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  pinMode(BUTTONENC, INPUT_PULLUP);



  encoder.begin();  //set encoders pins as input & enable built-in pullup resistors
                    //attachInterrupt(digitalPinToInterrupt(BUTTONENC), pressedEncoder, FALLING);



  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);  //call encoderISR()    every low->high change
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);  //call encoderISR()    every low->high change

  
  EEPROM.get(40, pressureOffset);

  if (pressureOffset >= 300 || pressureOffset < -300) {
   pressureOffset = 5.2;
    EEPROM.put(40, pressureOffset);
  }

  EEPROM.get(0, Declination);

  //if (Declination >= 360 || Declination < 0) {
    //Declination = 11.2;
   // EEPROM.put(0, Declination);
//  }


  EEPROM.get(10, brightness);
  if (brightness > 500 || brightness < 50) {
    brightness = 160;
    EEPROM.put(10, brightness);
  }

  EEPROM.get(20, marker);
  if (marker > 360 || marker < 0) {
    marker = 180;
    EEPROM.put(20, marker);
  }

  EEPROM.get(30, showHeading);
  if (showHeading != 1 || showHeading != 0) {
    showHeading = 1;
    EEPROM.put(30, showHeading);
  }


  //Serial.begin(115200);

#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  delay(100);
  Serial.println("Do the gyro bias offset calibration. Make sure the USFS is sitting still...");
  Serial.println("");
  //sensor_cal.sendOneToProceed();                                                                                   // Wait for user input to proceed
#endif

  attachInterrupt(digitalPinToInterrupt(INT_PIN), DRDY_handler_0, RISING);
}

unsigned long holdButton = 0;
unsigned long holdButtonFirstPressed = 0;
unsigned long lastPress = 0;
int pressFlag = 0;

void loop() {

int buttonState = digitalRead(BUTTONENC);

  readTime = millis();
  // interrupts();
  if (pressed == 1) encoderButtonISR();


  // Manage gyro cal
  if (calibratingG[0] == 1) {
    calibratingG[0] = 0;
   // sensor_cal.GyroCal();
   // delay(50);
  }

  if (data_ready[0] == 1) {
    data_ready[0] = 0;
    ProcEventStatus(&i2c_0, 0);                // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
    FetchUSFSMAX_Data(&USFSMAX_0, &imu_0, 0);  // USFSMAX instance 0, IMU calculation instance 0 and Sensor instance 0
  }


  if (buttonState == 0) {

    holdButton = millis();
    if (holdButton - holdButtonFirstPressed > 7000) {
      calibratingFlag = 1;
      USFSMAX_0.Reset_DHI();

    } else {
    }


  } else {
    holdButtonFirstPressed = millis();
  }


#if defined(IGV1)



  loadDisplayBuffer();

  writeNeon();



  if (calibratingFlag == 1) {
    calTimer = millis();

    if (calTimer - lastCalTimer > 500)  // Update the serial monitor every "UPDATE_PERIOD" ms
    {
      if ((i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS) & 0x80) >= 1) {
        calibratingFlag = 0;
        ///EEPROM.put(40,calibratingFlag);
      }

      lastCalTimer = calTimer;
    }
  }

#endif

//if (displayedMenu == 0) hoveredMenuItem = 0;

  //Serial.println((int)heading[0]);






#if defined(SERIALSTUFF)
  // Update serial output
  //Serial.begin(115200);
  delt_t = millis() - last_refresh;
  if (delt_t > UPDATE_PERIOD)  // Update the serial monitor every "UPDATE_PERIOD" ms
  {
    last_refresh = millis();

    //delay(100);
#ifdef SERIAL_DEBUG
    SerialInterface_handler();
    USFSMAX_0.GetMxMy();  // Get Horizontal magnetic components
    if (ENABLE_DHI_CORRECTOR) {
      cal_status[0] = i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS);  // Poll calibration status byte
      USFSMAX_0.getDHI_Rsq();                                                 // Get DHI R-square
      Serial.print("Dynamic Hard Iron Correction Valid = 0x");
      Serial.println(cal_status[0], HEX);  // DHI correction status
      Serial.print("Dynamic Hard Iron Fit R-square = ");
      Serial.println(Rsq, 4);
      if (USE_2D_DHI_CORRECTOR) {
        Serial.println("Using the 2D Corrector");
      } else {
        Serial.println("Using the 3D Corrector");
      }
      Serial.println("");
    } else {
      Serial.print("Dynamic Hard Iron Correction Disabled!");
      Serial.println("");
      Serial.println("");
    }


    float pressure = (float)((baroADC[0] / 4096.0f));
    //pressure -= 92.0f;

    float pressureExp = pow((pressure / (1013.25f + pressureOffset)), 0.190284);

    float elevation = 145366.45f * (1 - pressureExp);

    int elevationInt = (int)elevation;

    Serial.print("altitude = ");
    Serial.println(elevation);

    // USFSMAX_0 sensor and raw quaternion outout
    Serial.print("ax = ");
    Serial.print((int)(1000.0f * accData[0][0]));
    Serial.print(" ay = ");
    Serial.print((int)(1000.0f * accData[0][1]));
    Serial.print(" az = ");
    Serial.print((int)(1000.0f * accData[0][2]));
    Serial.println(" mg");
    Serial.print("gx = ");
    Serial.print(gyroData[0][0], 1);
    Serial.print(" gy = ");
    Serial.print(gyroData[0][1], 1);
    Serial.print(" gz = ");
    Serial.print(gyroData[0][2], 1);
    Serial.println(" deg/s");
    Serial.print("mx = ");
    Serial.print(magData[0][0], 1);
    Serial.print(" my = ");
    Serial.print(magData[0][1], 1);
    Serial.print(" mz = ");
    Serial.print(magData[0][2], 1);
    Serial.println(" uT");
    Serial.print("Tomasch Xh, Yh: ");
    Serial.print(Mx[0], 2);
    Serial.print(", ");
    Serial.print(My[0], 2);
    Serial.println(" uT");
    Serial.print("Baro pressure = ");
    Serial.print(((float)baroADC[0]) / 4096.0f);
    Serial.println(" hPa");
    Serial.println("");
    Serial.print("USFSMAX Quat: ");
    Serial.print("q0 = ");
    Serial.print(qt[0][0], 4);
    Serial.print(" qx = ");
    Serial.print(qt[0][1], 4);
    Serial.print(" qy = ");
    Serial.print(qt[0][2], 4);
    Serial.print(" qz = ");
    Serial.print(qt[0][3], 4);
    Serial.println("");

    // Euler angles
    Serial.print("USFSMAX Yaw, Pitch, Roll: ");
    Serial.print(heading[0], 2);
    Serial.print(", ");
    Serial.print(angle[0][0], 2);
    Serial.print(", ");
    Serial.println(angle[0][1], 2);

    // Critical time deltas
    //Serial.println(""); Serial.print("Loop CT:"); Serial.print(cycleTime); Serial.println(" us");
    Serial.print("Sensor Acq Time:");
    Serial.print(Acq_time);
    Serial.println(" us");
    Serial.println("");
#endif

// Spreadsheet output when "SERIAL_DEBUG" and "MOTION_CAL_GUI_ENABLED" are not defined in config.h
#if !defined(SERIAL_DEBUG) && !defined(MOTION_CAL_GUI_ENABLED)
    Serial.print(TimeStamp, 2);
    Serial.print(",\t");
    Serial.print(heading[0], 2);
    Serial.print(",\t");
    Serial.print(angle[0][0], 2);
    Serial.print(",\t");
    Serial.print(angle[0][1], 2);
    Serial.print(",\t");
    Serial.print(cal_status[0]);
    Serial.println(" ");
#endif

// Output formatted MotionCal GUI magnetometer data message when "MOTION_CAL_GUI_ENABLED" is defined and "SERIAL_DEBUG" is not defined in config.h
// https://www.pjrc.com/store/prop_shield.html
#if defined(MOTION_CAL_GUI_ENABLED) && !defined(SERIAL_DEBUG)
    //noInterrupts();
    Serial.begin(115200);
    Serial.print("Raw:");
    Serial.print(0);  // MotionCal GUI doesn't act upon accel/gyro input; send null data
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');


    Serial.print((int32_t)(magData[0][0] * 10.0f), 10);
    // The MotionCal GUI is expecting 0.1uT/LSB
    Serial.print(',');
    Serial.print((int32_t)(magData[0][1] * 10.0f), 10);
    Serial.print(',');
    Serial.print((int32_t)(magData[0][2] * 10.0f), 10);
    Serial.println();
    //interrupts();
#endif


  }


#endif

  if (buttonState == 0 && (millis() - lastPress > 200) && pressFlag == 0) {
    pressed = 1;
    lastPress = millis();
    pressFlag = 1;

  } else if (buttonState == 1) {
    pressFlag = 0;
    pressed = 0;
  }
}


void pressedEncoder() {
  //holdButtonFirstPressed = millis();
  pressed = 1;
}

void encoderISR() {
  encoder.readAB();

  int newPositionEncoder = encoder.getPosition();

  if (position != newPositionEncoder) {
    position = newPositionEncoder;



    if (declinationSet == 1) {
      
      Declination = (position % 360);
      EEPROM.put(0,Declination);
    }

    if (selectedMenuItem == 3) {

      int brightnessIncrement = 5;

      switch (brightness) {
        case 0 ... 99:
          brightnessIncrement = 1;
          break;

        case 100 ... 179:
          brightnessIncrement = 1;
          break;

        case 180 ... 270:
          brightnessIncrement = 1;
          break;

        case 271 ... 350:
          brightnessIncrement = 2;
          break;

        case 351 ... 500:
          brightnessIncrement = 15;
          break;

        case 501 ... 2000:
          brightness /= 10;
          brightness *= 10;
          brightnessIncrement = 50;
          break;

        case 2001 ... 2000000:
          brightness /= 10;
          brightness *= 10;
          brightnessIncrement = 200;
          break;
      }


      if (oldPos < position) {
        brightness += brightnessIncrement;
        EEPROM.put(10,brightness);

      } else {
        brightness -= brightnessIncrement;
        EEPROM.put(10,brightness);
      }
      oldPos = position;
    }

    if (selectedMenuItem == 5) {

      if (oldPos < position) {
        marker += 1;
        if (marker >= 360) marker = 0;
      } else {
        marker -= 1;
        if (marker < 0) marker = 359;
      }
      oldPos = position;

      marker = marker % 360;
      EEPROM.put(20,marker);
    }


if (showAltitude == 1){

      if (oldPos < position) {
        pressureOffset += 0.05;
        
      } else {
        pressureOffset -= 0.05;
      }
      oldPos = position;
EEPROM.put(40,pressureOffset);
}


    letter = abs(position);
    hoveredMenuItem = abs(position % 7);
    //position = Declination;
  }
}


void encoderButtonISR() {
  pressed = 0;
  

  encoder.readPushButton();

  
  
  EEPROM.put (40, pressureOffset);

  
  if (selectedMenuItem > 5) selectedMenuItem = 0;
  //while(1);
  //noInterrupts();
  toggleButton = !toggleButton;




  if (selectedMenuItem == 5) {
    declinationSet = 0;
    showAltitude = 0;
    showHeading = 1;
    EEPROM.put(20, marker);
    markerSetFlag = 0;

    hoveredMenuItem = 0;
    selectedMenuItem = hoveredMenuItem;
    displayedMenu = !displayedMenu;
    //interrupts();
    return;
  }
selectedMenuItem = hoveredMenuItem;

  if (selectedMenuItem == 1) {
    //showHeading = !showHeading;

    declinationSet = 0;
encoder.setPosition (0);
    if (showHeading == 1) {
      showAltitude = 0;
      showHeading = 0;
    } else {
      showAltitude = 0;
      showHeading = 1;
    }
    //selectedMenuItem = 0;
    EEPROM.put(30, showHeading);
    hoveredMenuItem = 0;
    selectedMenuItem = 0;
    //displayedMenu = 0;
    displayedMenu = !displayedMenu;
    
    return;
  }

  if (selectedMenuItem == 2) {
    if (declinationSet == 0)
    {
      encoder.setPosition(Declination);
      declinationSet = 1;
      showHeading = 1;
        hoveredMenuItem = 2;
    selectedMenuItem = 2;
      displayedMenu = 0;
EEPROM.put(0, Declination);
    }
    else {
      declinationSet = 0;
    EEPROM.put(0, Declination);
    EEPROM.get(30, showHeading);
   hoveredMenuItem = 0;
    selectedMenuItem = 0;
    //selectedMenuItem = hoveredMenuItem;
    displayedMenu = 0;
    //interrupts();
    
    }
return;
  } else {
    declinationSet = 0;
  }

  if (selectedMenuItem == 3) {
    EEPROM.put(10, brightness);
    declinationSet = 0;
    //hoveredMenuItem = 0;

    //displayedMenu = !displayedMenu;
  }


  if (selectedMenuItem == 5) {

declinationSet = 0;
    if (markerFlag == 0) {
      markerFlag = 1;
      markerSetFlag = 1;


    } else if (markerFlag == 1) {
      markerFlag = 0;
      markerSetFlag = 0;
       hoveredMenuItem = 0;
      selectedMenuItem = hoveredMenuItem;
    }
  }


  if (selectedMenuItem == 4) {
    declinationSet = 0;
    encoder.setPosition (0);
    if (showAltitude == 0) {
      showAltitude = 1;
      showHeading = 0;
    } else {
      showAltitude = 0;
      
      //EEPROM.get(30, showHeading);
      //showHeading = 1;
    }
    hoveredMenuItem = 0;
    selectedMenuItem = 0;
  }


  displayedMenu = !displayedMenu;


  //hoveredMenuItem = 0;
  //interrupts();
}



void showHeadingNumbers(int headingToShow) {

 // if (displayedMenu == 1) return;

  int numberCount = 0;
  int rowCount = 0;
  int scrollValue = scroll;

  int lastDigit = 1;
  int menuReminderAscii = 78;
  int asciiInt[12] = { -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16 };

if ((showHeading == 1 || declinationSet == 1 || markerSetFlag == 1 && showAltitude != 1 )&& selectedMenuItem != 3)
{
 headingToShow = (headingToShow + 280)%360;
 
}


  if (headingToShow < 10) {

    asciiInt[1] = headingToShow;


  } else if (headingToShow >= 10 && headingToShow < 100) {
    //asciiInt[3] = headingToShow % 10;

    asciiInt[2] = headingToShow % 10;
    asciiInt[1] = headingToShow / 10;

  } else if (headingToShow >= 100 && headingToShow < 1000) {

    asciiInt[3] = headingToShow % 10;
    asciiInt[2] = (headingToShow / 10) % 10;
    asciiInt[1] = (headingToShow / 100) % 10;

  } else if (headingToShow >= 1000 && headingToShow < 10000) {

    asciiInt[4] = headingToShow % 10;
    asciiInt[3] = (headingToShow / 10) % 10;
    asciiInt[2] = (headingToShow / 100) % 10;
    asciiInt[1] = (headingToShow / 1000) % 10;
  } else if (headingToShow >= 10000) {

    asciiInt[5] = headingToShow % 10;
    asciiInt[4] = (headingToShow / 10) % 10;
    asciiInt[3] = (headingToShow / 100) % 10;
    asciiInt[2] = (headingToShow / 1000) % 10;
    asciiInt[1] = (headingToShow / 10000) % 10;
  }


  for (int i = 1; i < 9; i++) {
    if (asciiInt[i] == -16) {
      lastDigit = i;
      break;
    }
  }
  switch (selectedMenuItem) {
    case 0:
      menuReminderAscii = 78;
      break;

    case 1:
      break;

    case 2:
      menuReminderAscii = 79;
      break;

    case 3:
      menuReminderAscii = 81;
      break;

    case 5:
      menuReminderAscii = 80;
      break;
  }

  if (calibratingFlag == 1) {
    menuReminderAscii = 82;
  }

  if (showAltitude == 1 && showHeading == 0)
  {
    menuReminderAscii = 83;
  }

  if (declinationSet == 1)
  {
    menuReminderAscii = 79;
  }

  asciiInt[lastDigit] = menuReminderAscii;

  if (asciiInt[1] == 1) rowCount -= 1;

  for (int j = 1; j <= 11; j++) {

    for (int k = 0; k < 5; k++) {

      if (k >= 5) {
        displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
      } else {
        displayBuffer[(scrollValue + rowCount) % 360] = ~charBitmaps[asciiInt[j] + 16][k];
      }
      rowCount++;
    }


    displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
    rowCount++;

    if (j + 1 == lastDigit && menuReminderAscii != 78) {
      displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
      rowCount++;
    }

    if (asciiInt[j] == 1) rowCount -= 1;

    if (asciiInt[j + 1] == -16) {
      if (menuReminderAscii != 78 && menuReminderAscii != 82) {
        displayBuffer[(scrollValue + rowCount) % 360] = 0b11111111;
        displayBuffer[(scrollValue + rowCount + 1) % 360] = 0b11111111;
        displayBuffer[(scrollValue + rowCount + 2) % 360] = 0b11111111;
      }
      return;
    }
  }
}

void loadDisplayBuffer(void) {


  switch (displayedMenu) {
    case 0:  //compass
      {

        hoveredMenuItem = 0;

        for (int i = 0; i < 360; i++) {

          displayBuffer[i] = compass[i];
          if (marker == i && markerFlag == 1) {
            //displayBuffer[(i-1)%360] = 0b00011111;
            displayBuffer[i] = 0b0000000;
            //displayBuffer[(i+1)%360] = 0b00010101;
          }
            if (declinationSet == 1 && i == ((scroll + 55) % 360)) displayBuffer[i] = 0b0000000;

        }
      }
      break;

    case 1:

      int letterCount = 0;
      int menuCount = 0;

      int displayTextLocation = 0;


      for (int k = 0; k < 5; k++) {
        for (int i = 0; i < menuArrayLengths[k]; i++) {
          displayText[displayTextLocation] = menuArray[k][i];
          if (hoveredMenuItem == k + 1) {
            invertedChars[displayTextLocation] = 1;

          } else {

            invertedChars[displayTextLocation] = 0;
          }

          displayTextLocation++;
        }
        displayText[displayTextLocation] = ' ';
        displayTextLocation++;
      }



      for (int i = 0; i < 40; i++) {
        displayBuffer[i] = 0b01111111;
      }

      for (int i = 0; i < 360; i += 6) {
        for (int j = 0; j < 6; j++) {

          int letterInt = displayText[i / 6] - 32;

          if (letterInt > 92 || letterInt < 0) letterInt = 0;


          if (invertedChars[i / 6] == 1) {
            if (j == 0) {
              displayBuffer[i + j - 1 + 40] = 0b00000000;
            }
            if (j < 5) {

              displayBuffer[i + j + 40] = charBitmaps[letterInt][j];

            } else {
              displayBuffer[i + j + 40] = 0b00000000;
            }
          } else {
            if (j < 5) {

              displayBuffer[i + j + 40] = ~charBitmaps[letterInt][j];

            } else {
              displayBuffer[i + j + 40] = 0b11111111;
            }
          }
        }
        letterCount++;
      }
      break;
  }
}
int totalDelayTimer = 0;

int refreshesPerLoop = 0;
float lastHeading = 0;

void writeNeon(void) {

digitalWrite(blank, LOW);

  if (displayedMenu == 0) {

    scroll = (int)heading[0];


    scroll = (scroll + Declination);
    if (scroll < 0) scroll += 360;  // Allow for under|overflow
    if (scroll >= 360) scroll -= 360;


    if (markerSetFlag == 1) {
      scroll = marker + 305;
      scroll = scroll % 360;
    }

////showHeadingNumbers(showHeading);

    if (selectedMenuItem == 3) {
      showHeadingNumbers(brightness);


    } else if (showHeading == 1) {

      showHeadingNumbers(scroll);

    } else if (showAltitude == 1 && declinationSet == 0) {

      float pressure = (float)((baroADC[0] / 4096.0f));
      //pressure -= 92.0f;

      float pressureExp = pow((pressure / (1013.25f + pressureOffset)), 0.190284);

      float elevation = 145366.45f * (1 - pressureExp);

      

      int elevationInt = (int)filter.add(elevation);

      showHeadingNumbers(elevationInt);
    }
  } else {


    for (int i = 0; i < 60; i++) {
      if (invertedChars[i] == 1) {
        selectedStart = i;
        break;
      }
    }
    scroll = (selectedStart * 6) + 20;
    //scroll = 20;
  }

  for (int i = scroll; i < (111 + scroll); i++) {
  

    unsigned char row = displayBuffer[i % 360];


    digitalWrite(row0, row & 0b01000000);
    digitalWrite(row1, row & 0b00100000);
    digitalWrite(row2, row & 0b00010000);
    digitalWrite(row3, row & 0b00001000);
    digitalWrite(row4, row & 0b00000100);
    digitalWrite(row5, row & 0b00000010);
    digitalWrite(row6, row & 0b00000001);

    delayMicroseconds(2);

    scanLocation++;
    if (scanLocation >= 3) scanLocation = 0;

    if (scanLocation == 0) {
      digitalWrite(scan1, HIGH);
      digitalWrite(scan2, LOW);
      digitalWrite(scan3, LOW);
    } else if (scanLocation == 1) {
      digitalWrite(scan1, LOW);
      digitalWrite(scan2, HIGH);
      digitalWrite(scan3, LOW);

    } else if (scanLocation == 2) {
      digitalWrite(scan1, LOW);
      digitalWrite(scan2, LOW);
      digitalWrite(scan3, HIGH);
    }

    delayMicroseconds(brightness);

    digitalWrite(row0, HIGH);
    digitalWrite(row1, HIGH);
    digitalWrite(row2, HIGH);
    digitalWrite(row3, HIGH);
    digitalWrite(row4, HIGH);
    digitalWrite(row5, HIGH);
    digitalWrite(row6, HIGH);
  }
  //interrupts();

  digitalWrite(blank, HIGH);
//scanLocation = 0;
  //delayMicroseconds(100);
  //digitalWrite(blank, LOW);
}


void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM) {
  uint8_t temp[1];

  // Read algorithm status and event status
  i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
  eventStatus[sensorNUM] = temp[0];

  // Decode the event status to determine what data is ready and set the appropriate DRDY fags
  if (eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
  if (eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM] = 1;
  if (eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM] = 1;
  if (eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
  if (eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}



void GoToSleep() {
  /*
  detachInterrupt(INT_PIN);
  delay(10);
  USFSMAX_0.GoToSleep();
  Serial.println("Going to sleep... Press the Dragonfly 'BOOT' button to wake.");
  Serial.flush();
  //USBDevice.detach();
  delay(1000);
  data_ready[0] = 0;
  awake = 0;
  STM32.stop();
  WakeUp();
  */
}

void WakeUp() {
  /*
  //USBDevice.attach();                                                                                                // Re-attach the USB port and re-open
  Serial.begin(115200);
  delay(100);
  //Serial.blockOnOverrun(false);
  attachInterrupt(INT_PIN, DRDY_handler_0, RISING);
  data_ready[0] = 0;
  awake = 1;
  digitalWrite(USFS_WAKE, HIGH);                                                                                     // Pulse USFSMAX wakeup pin (1ms)
  delay(1);
  digitalWrite(USFS_WAKE, LOW);
  while(1)                                                                                                           // Loop until DRDY interrupt signals USFSMAX is ready
  {
    if(data_ready[0])
    {
      data_ready[0] = 0;
      break;
    }
  }
  */
}

void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM) {
  //if (digitalRead(INT_PIN) == 1)
  //{
  uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

  Acq_time = 0;
  Begin = micros();

  // Optimize the I2C read function with respect to whatever sensor data is ready
  switch (call_sensors) {
    case 0x01:
      usfsmax->GyroAccel_getADC();
      break;
    case 0x02:
      usfsmax->GyroAccel_getADC();
      break;
    case 0x03:
      usfsmax->GyroAccel_getADC();
      break;
    case 0x07:
      usfsmax->GyroAccelMagBaro_getADC();
      break;
    case 0x0B:
      usfsmax->GyroAccelMagBaro_getADC();
      break;
    case 0x0F:
      usfsmax->GyroAccelMagBaro_getADC();
      break;
    case 0x0C:
      usfsmax->MagBaro_getADC();
      break;
    case 0x04:
      usfsmax->MAG_getADC();
      break;
    case 0x08:
      usfsmax->BARO_getADC();
      break;
    default:
      break;
  };
  Acq_time += micros() - Begin;

  if (Mag_flag[sensorNUM]) {
    if (ScaledSensorDataFlag)  // Calibration data is applied in the coprocessor; just scale
    {
      for (uint8_t i = 0; i < 3; i++) {
        magData[sensorNUM][i] = ((float)magADC[sensorNUM][i]) * UT_per_Count;
      }
    } else  // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count, mag_calData[sensorNUM]);
      sensor_cal.apply_adv_calibration(final_magcal[sensorNUM], mag_calData[sensorNUM], 1.0f, sensor_point);
      MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Mag_flag[sensorNUM] = 0;
  }
  if (Acc_flag[sensorNUM]) {
    if (ScaledSensorDataFlag)  // Calibration data is applied in the coprocessor; just scale
    {
      for (uint8_t i = 0; i < 3; i++) {
        accData[sensorNUM][i] = ((float)accADC[sensorNUM][i]) * g_per_count;
      }
    } else  // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(accelcal[sensorNUM], accADC[sensorNUM], g_per_count, sensor_point);
      ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Acc_flag[sensorNUM] = 0;
  }
  if (Gyro_flag[sensorNUM] == 1) {
    if (ScaledSensorDataFlag)  // Calibration data is applied in the coprocessor; just scale
    {
      for (uint8_t i = 0; i < 3; i++) {
        gyroData[sensorNUM][i] = ((float)gyroADC[sensorNUM][i]) * dps_per_count;
      }
    } else  // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count, sensor_point);
      GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }

    // Call alternative (Madgwick or Mahony) IMU fusion filter
    IMu->compute_Alternate_IMU();
    Gyro_flag[sensorNUM] = 0;
  }
  if (Quat_flag[sensorNUM] == 1) {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }
  // }
}

// Host DRDY interrupt handler
void DRDY_handler_0() {
  data_ready[0] = 1;
  //delayMicroseconds(5000);
  //while(1);
}

// Serial interface handler
void SerialInterface_handler() {
  serial_input = 0;
  if (Serial.available()) serial_input = Serial.read();
  if (serial_input == 49) { calibratingG[0] = 1; }  // Type "1" to initiate USFSMAX_0 Gyro Cal
  if (serial_input == 50)                           // Type "2" to list current sensor calibration data
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);  // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_full_gyrocal();
    delay(100);
    USFSMAX_0.Retreive_full_accelcal();
    delay(100);
    USFSMAX_0.Retreive_ellip_magcal();
    delay(100);
    USFSMAX_0.Retreive_final_magcal();
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);  // Resume high-speed I2C operation
    delay(100);

    // Print the calibration results
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(gyrocal[0].V[0], 4);
    Serial.println(gyrocal[0].V[1], 4);
    Serial.println(gyrocal[0].V[2], 4);
    Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(gyrocal[0].invW[0][0], 4);
    Serial.print(",");
    Serial.print(gyrocal[0].invW[0][1], 4);
    Serial.print(",");
    Serial.println(gyrocal[0].invW[0][2], 4);
    Serial.print(gyrocal[0].invW[1][0], 4);
    Serial.print(",");
    Serial.print(gyrocal[0].invW[1][1], 4);
    Serial.print(",");
    Serial.println(gyrocal[0].invW[1][2], 4);
    Serial.print(gyrocal[0].invW[2][0], 4);
    Serial.print(",");
    Serial.print(gyrocal[0].invW[2][1], 4);
    Serial.print(",");
    Serial.println(gyrocal[0].invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(accelcal[0].V[0], 4);
    Serial.println(accelcal[0].V[1], 4);
    Serial.println(accelcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(accelcal[0].invW[0][0], 4);
    Serial.print(",");
    Serial.print(accelcal[0].invW[0][1], 4);
    Serial.print(",");
    Serial.println(accelcal[0].invW[0][2], 4);
    Serial.print(accelcal[0].invW[1][0], 4);
    Serial.print(",");
    Serial.print(accelcal[0].invW[1][1], 4);
    Serial.print(",");
    Serial.println(accelcal[0].invW[1][2], 4);
    Serial.print(accelcal[0].invW[2][0], 4);
    Serial.print(",");
    Serial.print(accelcal[0].invW[2][1], 4);
    Serial.print(",");
    Serial.println(accelcal[0].invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(ellipsoid_magcal[0].V[0], 4);
    Serial.println(ellipsoid_magcal[0].V[1], 4);
    Serial.println(ellipsoid_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(ellipsoid_magcal[0].invW[0][0], 4);
    Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[0][1], 4);
    Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[0][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[1][0], 4);
    Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[1][1], 4);
    Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[1][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[2][0], 4);
    Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[2][1], 4);
    Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(final_magcal[0].V[0], 4);
    Serial.println(final_magcal[0].V[1], 4);
    Serial.println(final_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(final_magcal[0].invW[0][0], 4);
    Serial.print(",");
    Serial.print(final_magcal[0].invW[0][1], 4);
    Serial.print(",");
    Serial.println(final_magcal[0].invW[0][2], 4);
    Serial.print(final_magcal[0].invW[1][0], 4);
    Serial.print(",");
    Serial.print(final_magcal[0].invW[1][1], 4);
    Serial.print(",");
    Serial.println(final_magcal[0].invW[1][2], 4);
    Serial.print(final_magcal[0].invW[2][0], 4);
    Serial.print(",");
    Serial.print(final_magcal[0].invW[2][1], 4);
    Serial.print(",");
    Serial.println(final_magcal[0].invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    sensor_cal.sendOneToProceed();  // Halt the serial monitor to let the user read the calibration data
  }
  if (serial_input == 51) { USFSMAX_0.Reset_DHI(); }  // Type "3" to reset the DHI corrector
  if (serial_input == 52)                             // Type "4" to list copro config
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);  // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_cfg();  // Get the current USFSMAX config
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);  // Resume high-speed I2C operation
    delay(100);
    Serial.print("Accel scale = ");
    Serial.println(Cfg[1].Ascale);
    Serial.print("Accel ODR   = ");
    Serial.println(Cfg[1].AODR);
    Serial.print("Accel LPF   = ");
    Serial.println(Cfg[1].Alpf);
    Serial.print("Accel HPF   = ");
    Serial.println(Cfg[1].Ahpf);
    Serial.print("Gyro scale  = ");
    Serial.println(Cfg[1].Gscale);
    Serial.print("Gyro ODR    = ");
    Serial.println(Cfg[1].GODR);
    Serial.print("Gyro LPF    = ");
    Serial.println(Cfg[1].Glpf);
    Serial.print("Gyro HPF    = ");
    Serial.println(Cfg[1].Ghpf);
    Serial.print("Quat div    = ");
    Serial.println(Cfg[1].quat_div);
    Serial.print("Mag scale   = ");
    Serial.println(Cfg[1].Mscale);
    Serial.print("Mag ODR     = ");
    Serial.println(Cfg[1].MODR);
    Serial.print("Mag LPF     = ");
    Serial.println(Cfg[1].Mlpf);
    Serial.print("Mag HPF     = ");
    Serial.println(Cfg[1].Mhpf);
    Serial.print("Baro scale  = ");
    Serial.println(Cfg[1].Pscale);
    Serial.print("Baro ODR    = ");
    Serial.println(Cfg[1].PODR);
    Serial.print("Baro LPF    = ");
    Serial.println(Cfg[1].Plpf);
    Serial.print("Baro HPF    = ");
    Serial.println(Cfg[1].Phpf);
    Serial.print("AUX_1 scale = ");
    Serial.println(Cfg[1].AUX1scale);
    Serial.print("AUX_1 ODR   = ");
    Serial.println(Cfg[1].AUX1ODR);
    Serial.print("AUX_1 LPF   = ");
    Serial.println(Cfg[1].AUX1lpf);
    Serial.print("AUX_1 HPF   = ");
    Serial.println(Cfg[1].AUX1hpf);
    Serial.print("AUX_2 scale = ");
    Serial.println(Cfg[1].AUX2scale);
    Serial.print("AUX_2 ODR   = ");
    Serial.println(Cfg[1].AUX2ODR);
    Serial.print("AUX_2 LPF   = ");
    Serial.println(Cfg[1].AUX2lpf);
    Serial.print("AUX_2 HPF   = ");
    Serial.println(Cfg[1].AUX2hpf);
    Serial.print("AUX_3 scale = ");
    Serial.println(Cfg[1].AUX3scale);
    Serial.print("AUX_3 ODR   = ");
    Serial.println(Cfg[1].AUX3ODR);
    Serial.print("AUX_3 LPF   = ");
    Serial.println(Cfg[1].AUX3lpf);
    Serial.print("AUX_3 HPF   = ");
    Serial.println(Cfg[1].AUX3hpf);
    Serial.print("Vert FS     = ");
    Serial.println(Cfg[1].m_v, 5);
    Serial.print("Horiz FS    = ");
    Serial.println(Cfg[1].m_h, 5);
    Serial.print("Declination = ");
    Serial.println(Cfg[1].m_dec, 5);
    Serial.print("Cal points  = ");
    Serial.println(Cfg[1].cal_points);
    Serial.println("");
    Serial.println("");
    sensor_cal.sendOneToProceed();  // Halt the serial monitor to let the user read the calibration data
  }
  if (serial_input == 53)  // Type "5" to go to sleep
  {
    go_to_sleep = 1;
  }
  serial_input = 0;

  // Hotkey messaging
  Serial.println("'1' Gyro Cal");
  Serial.println("'2' List Cal Data");
  Serial.println("'3' Reset DHI Corrector");
  Serial.println("'4' List USFSMAX Config");
  Serial.println("'5' Go to sleep");
  Serial.println("");
}
