#include "header.h"


#ifdef PID_CONTROL

extern struct TidalVolume TV;
struct PID_TYPE pid;

PID myPID(&pid.input, &pid.output, &pid.setpoint, pid.Kp, pid.Ki, pid.Kd, DIRECT);

//----------------------------------------------------------------------
//PID calculations for stepper motors
//----------------------------------------------------------------------
void PID_setup()
{
  myPID.SetMode(AUTOMATIC);

  myPID.SetOutputLimits(min_stepCmd, max_stepCmd);

  myPID.SetSampleTime(PID_sample_time);
}

void calc_PID()
{
  pid.input = TV.measured;
  
  myPID.Compute();
  
  //add deadbamd that the motors do nothing
//  if ( pid.input - pid.setpoint < deadband && pid.input - pid.setpoint > -deadband) pid.output = 0;
  if ( abs(pid.input - pid.setpoint) < deadband) pid.output = 0;
}

#endif