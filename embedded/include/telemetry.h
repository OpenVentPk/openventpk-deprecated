#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#define TEL_PACKET_LENGTH 48
struct TEL_TYPE {

byte FDCB;

unsigned int txPktCtr;
unsigned int txUpdateRate;

unsigned long Time;
float mTV;
float mTVinsp;
float mTVexp;
float mPressure;
float mFlowRate;
float mPEEP;
float mPltPress;
float mFiO2;
float minuteVentilation;
float staticCompliance;
float mRR;
float mPeakPressure;



int spTV;
int spInsPressure;
int spExpPressure;
int spFiO2;
int spBPM;
int spIE_Inhale;
int spIE_Exhale;
int spPEEP;
int patientWeight;
float spTrigger;

unsigned char statusByteError;
unsigned char statusByte1;
}; 

void Prepare_Tx_Telemetry();

#endif
