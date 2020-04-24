#ifndef __FLOWSENSOR_H
#define __FLOWSENSOR_H

#if defined(SFM3200AW)
#include "sfm3x00.h"
#define I2C_ADDR_SFM 64 // 0x40 //Default for SFM3000, SFM3200, SFM3200AW and SFM3300
#endif

//#define APPLY_Q_CORRECTION_FACTOR

void initFlowSensor();

float getFlowValue();

float getCorrectedFlowValue(int phase);

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

struct Flow_Sensor
{
  byte connectionStatus = 127;
  unsigned long offsetFlow = 32768; // Offset for the sensor
  float scaleFactorFlow_Air = 120.0; // Scale factor for Air
  float scaleFactorFlow_O2 = 142.8; // Scale factor for O2 is SFM3000 Only
  unsigned int offsetTemperature = 20000; // Offset for the sensor for SFM3200AW Only
  float scaleFactorTemperature = 100.0; // Scale factor for Temperature for SFM3200AW Only
  float Q_SLM = 0.0;
};

#endif