#include "header.h"
#include "flowSensor.h"

#if defined(SFM3200AW)
SFM3x00 measflow(I2C_ADDR_SFM);
#endif

struct Flow_Sensor FS;

void initFlowSensor()
{
    #if defined(SFM3200AW)
    FS.offsetFlow = 32768; // Offset for the sensor
    FS.scaleFactorFlow_Air = 120.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
    //float scaleFactorFlow_O2 = Not Applicable; // Scale factor for Air and N2 is 140.0, O2 is 142.8
    // For compensation for O2 and humidity see application note: 
    // “GF_AN_SFM3200_SFM3300_Effects_Humidity_Gas_mixtures”
    // https://www.repcomsrl.com/wp-content/uploads/2016/05/GF_AN_SFM3200_SFM3300_Effects_Humidity_Gas_mixtures_V1_D2.pdf

    FS.offsetTemperature = 20000; // Offset for the sensor
    FS.scaleFactorTemperature = 100.0; // Scale factor for Temperature

    FS.connectionStatus = measflow.init();
#ifndef TX_SERIAL_TELEMETRY
//    Serial.print("Flow Sensor Error Code: "); Serial.println(FS.connectionStatus);
//    delay(20000); //for testing
    #endif
    #endif
}

float getFlowValue()
{
    float volFlowRate = 0.0;
    unsigned int rawVal = 0.0;
    rawVal = measflow.getvalue();
    volFlowRate = ((float)rawVal - FS.offsetFlow) / (FS.scaleFactorFlow_Air);
    volFlowRate = (-1.0) * volFlowRate;
    return volFlowRate;
}

float getCorrectedFlowValue(int phase)
{
    float volFlowRate = 0.0;
    float correctedFlowRate = 0.0;
    volFlowRate = getFlowValue();

    #ifndef APPLY_Q_CORRECTION_FACTOR
    correctedFlowRate = volFlowRate;
    #else
    float qMeas, qMeasAbs, qMin, qMax, factorMin, factorMax, corrFactor;
    qMeas = volFlowRate;
    qMeasAbs = abs(qMeas);
    switch (phase)
    {
    case INSPIRATION_PHASE:
        if (qMeasAbs <= 30.0)
        {
            qMin = 0.0;
            qMax = 30.0;
            factorMin = 0.0;
            factorMax = 2.5;
            corrFactor = (mapFloat(qMeasAbs, qMin, qMax, factorMin, factorMax)) / 100.0f;
        }
        else    corrFactor = 2.5 / 100.0f;                
        break;
    case EXPIRATION_PHASE:
        if (qMeasAbs <= 30.0)
        {
            qMin = 0.0;
            qMax = 30.0;
            factorMin = 1.80;
            factorMax = 0.64;
            corrFactor = (mapFloat(qMeasAbs, qMin, qMax, factorMin, factorMax)) / 100.0f;
        }
        else
        {
            qMin = 30.0;
            qMax = 250.0;
            factorMin = 0.64;
            factorMax = 0.20;
            corrFactor = (mapFloat(qMeasAbs, qMin, qMax, factorMin, factorMax)) / 100.0f;
        }    
        break;
    default:
        corrFactor = 0.0;
        break;
    }
//    if (qMeas >= 0.0)
        correctedFlowRate = qMeas * (1 - corrFactor);
    //else
    // Seperate handling for negative values reqd or not
    #endif

    return correctedFlowRate;
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow; 
}