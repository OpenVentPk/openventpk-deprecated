/*
  SFM3x00.cpp - Library for reading values from flowmeter Sensirion SFM3x00
  https://www.sensirion.com/en/flow-sensors/mass-flow-meters-for-high-precise-measurement-of-gases/low-pressure-drop-mass-flow-meter/
  Created by WeDo, Zuerich 20170616
  
  Released into the public domain.
  
  
  Pay attention to Scaling!!
  int offset = 32000; // Offset for the sensor
  float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
  Flow = (readvalue - offset) / scale
  
*/

#ifndef SFM3x00_h
#define SFM3x00_h
 
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif
 

class SFM3x00 {
  public:
    //SFM3x00(uint8_t i2cAddress);
	SFM3x00(int i2cAddress);
    byte init();
    float getvalue();
    
 
  private:
	//uint8_t mI2cAddress;
	int mI2cAddress;
	uint8_t crc8(const uint8_t data, uint8_t crc);
};
    // unsigned long _startTime;
    // float _startValue; // Start from this value
 
    // unsigned long _stopTime;
    // float _stopValue;   // Stop at this value
 
    // float lerp(float m1, float M1, float m2, float M2, float v1);
 
#endif
