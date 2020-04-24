/*
 * All the #define are in Plateformio.ini file with -D name 
  OpenVentPK Prototype 1 OVPD-1 - Source Code
    //TODO:
        ADD Version History Here
        In Future;
        Implement Watchdog
        Address Wire.h long reported bug
        Implement Assist Control
        Implement Pressure Controlled Mode
        Interface O2 Sensor
        Interface Flow Rate Sensor

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Author: OpenVentPK.
    Created March 28, 2020
    EMAIL : <sohaib_ashraf@hotamail.com>
*/
#include "header.h"
#include "gui.h"
#include "flowSensor.h"

LiquidCrystal_I2C lcd(0x27, 20, 4); // Address of LCD will be finalized from LCD Datasheet
//Other Address could be 0x20, 0x27 , 0x3F
// Run I2C_Scanner Script to discover device

#ifdef TX_SERIAL_TELEMETRY
#include "telemetry.h"
extern struct TEL_TYPE TEL;
#endif
 
boolean debug;             // True if a debugging mode is used, false otherwise
float plateauPressure, /* Plateau pressure is the pressure that is applied by the ventilator to the small airways and alveoli.
                           It is measured at end-inspiration with an inspiratory hold maneuver.*/
      PEEPressure,       // Positive end-expiratory pressure (PEEP)
      ambientPressure,   // Calculated ambiant air pressure (averaged)
      peakInspPressure,      // high pass filtered value of the pressure. Used to detect patient initiated breathing cycles
      avgPressure,       // Averaged pressure (used to limit the motor speed with short spikes filtered out
      relPressure;       // relative pressure (measured pressure - ambiant pressure)
unsigned long tick1,            // counter used to trigger sensor measurements //loop 1
         tick2,             // counter used to trigger sensor measurements // loop 2
         breathLength;      // duration of the current breathing cycle in milliseconds. This is 60000/BPM.

boolean PeepValid = false;
boolean PltPrsValid = false;

char  tempChar[20];


//double CodeVer;
// Parameters saved to / recovered from EEPROM

float reqBPM;                // respiratory frequency
float reqVolume;             // respiratory volume in milliliters
float reqPressure;           // compression for the ambu-bag in Pa
int reqExpirationRatioIndex; // The proportion of each breathing cycle that is spent breathing in compared to breathing out

float bpmSetpoint;             // respiratory frequency
float volumeSetpoint;          // respiratory volume in milliliters
float pressureSetpoint;        // compression for the ambu-bag in Pa
float expirationRatioSetpoint; // The proportion of each breathing cycle that is spent breathing in compared to breathing out

int breathPhase = WAIT_PHASE;
int selfTestProg = ST_NOT_INIT; // Selft Test is Implemented
int selfTestStatus = ST_PASS;   // Selft Test is Implemented

#ifdef AUTO_HOME
int Homing_Done_F = 0;
#endif

int ErrorNumber = 0;
int devMode = 0;
int activateVentilatorOperation = 0;

int WarmUpFlag = 1;
int DevModeDetectionInProg = 0;
int PEEPMesaureFlag = 0;

int CVmode = VOL_CONT_MODE;// CV or CP mode indicator;
int assistControl = 0;
boolean scanBreathingAttempt = false;
boolean patientTriggeredBreath = false;
boolean holdManeuver = false;
boolean setpointAchieved = false;
int holdDur_ms = 150;
//int triggerVariable = FLOW_VAR;
//int cyclingVariable = TIME_VAR;
float flowTrigger = 0.5; //lpm

int VentilatorOperationON = 0;
char PressedKey = 0;
char keyRead = 1;

#define debounceDelay 600 //ms
int OkButton = 0;
int SnoozeButton = 0;

int pressSnsrInUse = MS4525_IN_USE;

//#ifdef StepGen
int motorInUse = STEPPER_IN_USE;
//#endif

int patientWeight = 50; //kg

int spStatusAllowChange = 0;
int I_E_InpsFactor[5] = {2, 1, 1, 1, 1};
int I_E_ExpFactor[5] = {1, 1, 2, 3, 4};
float I_E_SampleSet[5] = {2.0, 1.0, 0.5, 0.33, 0.25}; // 2:1, 1:1, 1:2, 1:3, 1:4

extern struct Flow_Sensor FS;

struct setpointStatus spStatus;
struct P_Sensor p_sensor;
struct TidalVolume TV;
struct Slave slave;

#ifdef Beeper
struct Alarm alarm;
#endif
#ifdef PID_CONTROL
extern struct PID_TYPE pid;
#endif

void (*resetFunction)(void) = 0; // Self reset (to be used with watchdog)

boolean checkValues()
{
  boolean isOk = (reqBPM >= minBPM);                  // BPM in allowed range ?
  if (reqBPM > maxBPM) isOk = false;
  if (reqVolume < minVolume) isOk = false;            // Volume in allowed range ?
  if (reqVolume > maxVolume) isOk = false;
  if (reqPressure < minPressure) isOk = false;  // Compression in allowed range ?
  if (reqPressure > maxPressure) isOk = false;
  if (isnan(reqBPM)) isOk = false;                    // Check for malformed floating point values (NaN)
  if (isnan(reqVolume)) isOk = false;
  if (isnan(reqPressure)) isOk = false;

  return isOk;
}

#ifdef Beeper

void beep() // Launch a beep
{
  static unsigned int currentToneFreq = SNOOZE_ALARM;
  static unsigned long t_millis = 0;

#ifdef ActiveBeeper
  if (alarm.action == SNOOZE_ALARM)
  {
    noTone(pin_Beep);
    currentToneFreq = SNOOZE_ALARM;
    alarm.toneFreq = SNOOZE_ALARM;
    alarm.timePeriod = SNOOZE_ALARM;
  }
  else
  {
    if (alarm.toneFreq > currentToneFreq)
    {
      currentToneFreq = alarm.toneFreq; //High Severity Alarm Has priority
      t_millis = 0;
    }

    if ((millis() - t_millis) >= alarm.timePeriod)
    {
      t_millis = millis();
      tone(pin_Beep, currentToneFreq, (int)(alarm.timePeriod / 2)); //Duration in milliseconds
    }    
  }
#endif

#ifdef PassiveBeeper //Call Inside Interrupt if want to use Passive Beeper //NEEDS REVISION
  static unsigned long old_millis = 0;
  static unsigned long buzzerToggleTime = 0; //ms
  static int buzzer = HIGH;
  if (alarm.action == SNOOZE_ALARM)
  {
    digitalWrite(pin_Beep, LOW);
    currentToneFreq = SNOOZE_ALARM;
  }
  else
  {
    if (alarm.toneFreq > currentToneFreq)
    {
      currentToneFreq = alarm.toneFreq;
      buzzerToggleTime = (unsigned long)(500 / currentToneFreq); //ms
    }
    if ((millis() - old_millis) >= buzzerToggleTime)
    {
      old_millis = millis();
      digitalWrite(pin_Beep, buzzer);
      buzzer = !buzzer;
    }
  }
#endif
}
#endif

void eeput(int n) // records to EEPROM (only if values are validated)
{
#ifdef E2PROM
  int eeAddress = eeStart;
  boolean isOk = checkValues();

  if (n == 1) isOk = true; // override (for debug testing)
  if (isOk)
  {
    EEPROM.put(eeAddress, reqBPM);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, reqVolume);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, reqPressure);
    eeAddress += sizeof(float);
  }
#endif
}

void eeget()
{
  reqBPM = defaultBPM;
  reqVolume = defaultVolume;
  reqPressure = defaultPressure;
  reqExpirationRatioIndex = defaultExpirationRatioIndex;
//  Serial.print("Read Default Settings\n");  //Arduino gets stuck if comment this line
}

int homePosHitMotorPos = 0;
int OP2HitMotorPos = 0;

#ifdef StepGen
void Timer()
{
/*  if (CVmode == VOL_CONT_MODE) {
  if ((abs(TV.inspiration - volumeSetpoint) <= 20.0) && (breathPhase == INSPIRATION_PHASE) && (!setpointAchieved))
  {
    Serial.println("STOP: Volume Achieved");
//    Serial2.print("#STOP\r");
    txSlaveCMD(STOP);
    slave.lastCMD_ID = STOP; 
    setpointAchieved = true;
  }
  else    setpointAchieved = false;  
  }
  else // PRESS_CONT_MODE
  {
    if ((abs(p_sensor.pressure_gauge_CM - (pressureSetpoint*Pa2cmH2O)) <= 1.0) && (breathPhase == INSPIRATION_PHASE) && (!setpointAchieved))
    {
    Serial.println("STOP: Pressure Achieved");
//    Serial2.print("#STOP\r");
      txSlaveCMD(STOP);
      slave.lastCMD_ID = STOP; 
      setpointAchieved = true;
    }
    else    setpointAchieved = false;  
  }
*/
}
#endif

//Self Test and Auto Calibrate Routines
void selfTest()
{
  static boolean ctr = 0;
  ErrorNumber = 0;
  selfTestStatus = ST_PASS;
  selfTestProg   = ST_IN_PROG;

  if (FS.connectionStatus != 0)
  {
    ErrorNumber = FLOW_SENSOR_DISCONNECTED;
    #ifndef TX_SERIAL_TELEMETRY
    Serial.print("Flow Sensor Error Code: "); Serial.println(FS.connectionStatus);  
    #endif
    return;
  }

#ifdef StepGen
#ifdef AUTO_HOME

Homing_Done_F = slave.homeAck;

  if (Homing_Done_F == 0)
  {    
    if (ctr == 0) { 
      //Serial2.print("#HOME 2000"); //2000us
      txSlaveCMD(HOME, 2000);
      slave.lastCMD_ID = HOME; }
    else {ctr++; if (ctr == (1000/samplePeriod1)) ctr = 0;}        

    ErrorNumber     = HOMING_NOT_DONE_ERROR;
    selfTestStatus  = ST_FAIL;
    return;
  }
  else if (Homing_Done_F == 1)
  {
    ErrorNumber     = HOMING_NOT_DONE_ERROR;
    selfTestStatus  = ST_FAIL;
    return;
  }
  else if (Homing_Done_F == 2)
  {
    //HOMING COMPLETE and SUCCESSFUL
  }
  else if (Homing_Done_F == 3)
  {
    ErrorNumber     = MECH_INTEGRITY_FAILED;
    selfTestStatus  = ST_FAIL;
    return;
  }
#endif
#endif
/*  if (activateVentilatorOperation == 1)
  {
    //    Serial.println("Self Test FAIL");

    ErrorNumber     = START_SWT_ERROR;
    selfTestStatus  = ST_FAIL;
//    selfTestProg    = ST_COMPLETE;
    return;
  }
  else
  {
    //    Serial.println("Self Test PASS");
    selfTestProg    = ST_COMPLETE;
    return;
  }*/
selfTestProg    = ST_COMPLETE;
return;
}
#ifdef MS4525DO
/*
   used to get values from MS4525 sensor
   and should be called after 10ms
   due to sensor update or refresh rate
*/

inline static void get_sensor_data(uint16_t *raw) {
  Wire.beginTransmission(I2C_ADDRESS_MS4525DO);
  Wire.write(1);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS_MS4525DO, 2); //request for two pressure bytes from sensor
  *raw = (Wire.read() & 0x3F) << 8; // read the msb from the I2C device
  *raw |= Wire.read();//read the lsb from the device
}
#endif

#ifdef BMP_180
void Bmp180Read()
{
  char status;
  double T, P, p0, a;
#ifdef __DEBUG
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE, 0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE * 3.28084, 0);
  Serial.println(" feet");
#endif
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp180.getTemperature(T);
    if (status != 0)
    {
      p_sensor.bmp_temperature = T;
#ifdef __DEBUG
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T, 2);
      Serial.print(" deg C, ");
      Serial.print((9.0 / 5.0)*T + 32.0, 2);
      Serial.println(" deg F");
#endif
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp180.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp180.getPressure(P, T);
        if (status != 0)
        {
          p_sensor.bmp_pressure = P;
#ifdef __DEBUG
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P, 2);
          Serial.print(" mb, ");
          Serial.print(P * 0.0295333727, 2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = bmp180.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0, 2);
          Serial.print(" mb, ");
          Serial.print(p0 * 0.0295333727, 2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = bmp180.altitude(P, p0);
          Serial.print("computed altitude: ");
          Serial.print(a, 0);
          Serial.print(" meters, ");
          Serial.print(a * 3.28084, 0);
          Serial.println(" feet");
#endif
        }
#ifdef __DEBUG
        else Serial.println("error retrieving pressure measurement\n");
#endif
      }
#ifdef __DEBUG
      else Serial.println("error starting pressure measurement\n");
#endif
    }
#ifdef __DEBUG
    else Serial.println("error retrieving temperature measurement\n");
#endif
  }
#ifdef __DEBUG
  else Serial.println("error starting temperature measurement\n");
#endif
  //delay(5000);  // Pause for 5 seconds.
}
#endif

float readVcc() {
  long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert while (bit_is_set(ADCSRA,ADSC));
  result = ADCL; result |= ADCH << 8;
  result = 1126400L / result; // Ba,,ck-calculate AVcc in mV
  return result;
}


void voltage_correction(float &diff_press_pa)
{
  const float slope = 65.0f;
  /*
    apply a piecewise linear correction, flattening at 0.5V from 5V
  */
  float voltage_diff = readVcc() - 5.0f;
  if (voltage_diff > 0.5f) {
    voltage_diff = 0.5f;
  }

  if (voltage_diff < -0.5f) {
    voltage_diff = -0.5f;
  }
  diff_press_pa -= voltage_diff * slope;
}

void readSensors() // Read Values from Installed Sensor
{
#ifdef MPXV7002DP
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  /* pressure = 1000 * (((5 * voltageRead) / 5) - (5 / 2));
     https://makersportal.com/blog/2019/02/06/arduino-pitot-tube-wind-speed-theory-and-experiment */
  p_sensor.diff_press_pa = (1000 * voltage_read) - 2500;
#elif defined (MPX4250)
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  // Typical MPV is 0.204v@20KPa
  // Maximum pressure is 250KPa
  // pressure = voltage * (250 / (MPV + 4.692))
  p_sensor.diff_press_pa = voltage_read * 51.06209150326797f;
#elif defined (MPX2010DP)
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  //changing voltage to pressure 100kPa/3.5v
  p_sensor.diff_press_pa = voltage_read * 2857.14285714286f;
#elif defined (MPX10DP)
  float voltage_read = analogRead(MPX_IN) * ADC_TO_VOLTS;
  //changing voltage to pressure 100kPa/3.5v
  p_sensor.diff_press_pa = voltage_read * 2857.14285714286f;
#elif defined (MS4525DO)
  // Calculate differential pressure. As its centered around 8000
  // and can go positive or negative
  uint16_t raw_pressure = 0;
  get_sensor_data(&raw_pressure);

  /*this equation is an inversion of the equation in the
    pressure transfer function figure on page 4 of the datasheet
    We negate the result so that positive differential pressures
    are generated when the bottom port is used as the static
    port on the pitot and top port is used as the dynamic port*/
  p_sensor.diff_press_PSI = -((raw_pressure - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
  p_sensor.diff_press_pa = p_sensor.diff_press_PSI * PSI_to_Pa;

  voltage_correction(p_sensor.diff_press_pa); //Recommended by Hamza
#ifdef MS4525_AS_Gauge_Pressure
  p_sensor.pressure_gauge = p_sensor.diff_press_pa;

  static float oldY = 0.0;
  static float avgP = 0.8;
  float y = p_sensor.pressure_gauge * Pa2cmH2O;
  y = (y * avgP) + ( oldY * (1.0 - avgP));
  oldY = y;
  p_sensor.pressure_gauge_CM = oldY;

  //  p_sensor.pressure_gauge_CM = p_sensor.pressure_gauge * Pa2cmH2O; //Unfiltered

#else
  float diff_press_bar = (p_sensor.diff_press_PSI * 0.0689476);
  //liter/min flow rate 60000 where is K is 520 constant
  p_sensor.q = ( K * diff_press_Bar ) * 60000;
#endif
#endif
 
#ifdef BMP_180
  Bmp180Read();
#endif

  //FLOW SENSORS
  #if defined(FLOW_SENSOR_INSTALLED)
  FS.Q_SLM = getFlowValue();
  #endif


}

void Monitoring()
{
  static bool initInsp = true;
  static bool initHold = true;
  static bool initExp = true;
  static bool initMeasurePEEP = true;
  static float minuteVentilationSum = 0.0;

  static unsigned long T_old_us = millis();

  static unsigned long pre_millis_1min = 0;

  if ((millis() - pre_millis_1min) >= 60000)
  {
    pre_millis_1min = millis();
    TV.minuteVentilation = minuteVentilationSum;
  }  

  float delta_t = ((float)(millis() - T_old_us)); //ms
// Plateau Pressure & PEEP and Set Breathing Flags
  switch (breathPhase)
  {
    case INSPIRATION_PHASE:
      if (initInsp) {
        TV.inspiration = 0.0; TV.measured = 0.0;
        setpointAchieved = false;
        peakInspPressure = p_sensor.pressure_gauge_CM * cmH2O_to_Pa;
        initInsp = false;
      }
      TV.inspiration += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);
      TV.measured = TV.inspiration;   

      minuteVentilationSum += TV.inspiration;

      if (peakInspPressure < p_sensor.pressure_gauge_CM)
        peakInspPressure = p_sensor.pressure_gauge_CM;


      // reset init Flags
      initHold = true;
      initExp = true;
      initMeasurePEEP = true;
      scanBreathingAttempt = false;
      patientTriggeredBreath = false;
    break;
    case HOLD_PHASE:
      TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);

      PltPrsValid = true;
      if (initHold)
      {
        plateauPressure = (p_sensor.pressure_gauge_CM * cmH2O_to_Pa);
        initHold = false;
      }
      else
      {
        plateauPressure = (plateauPressure + (p_sensor.pressure_gauge_CM * cmH2O_to_Pa)) * 0.5;
      }
    
      TV.staticCompliance = (TV.inspiration / (plateauPressure - PEEPressure));

      // reset init Flags
      initInsp = true;
      initExp = true;
      initMeasurePEEP = true;
      scanBreathingAttempt = false;
      patientTriggeredBreath = false;
      break;
    case EXPIRATION_PHASE:
      if (initExp) {TV.expiration = 0.0; initExp = false;}
      TV.expiration += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);      
      TV.measured += (((FS.Q_SLM * 1000.0f) / 60000.0f) * delta_t);

      if (assistControl == 1) {
        if (scanBreathingAttempt)
        {
          if (FS.Q_SLM >= flowTrigger) {
          patientTriggeredBreath = true; }          
        }
      }
      if (PEEPMesaureFlag == 1)
      {
        PeepValid = true;
        if (initMeasurePEEP)
        {
          PEEPressure = (p_sensor.pressure_gauge_CM * cmH2O_to_Pa);
          initMeasurePEEP = false;
        }
        else
        {
          PEEPressure = (PEEPressure + (p_sensor.pressure_gauge_CM * cmH2O_to_Pa)) * 0.5;
        }
//        TV.staticCompliance = (TV.inspiration / (plateauPressure - PEEPressure));
      }

      // reset init Flags
      initInsp = true;
      initHold = true;
      break;
    default: //WAIT PHASE
      // reset init Flags
      initInsp = true;
      initHold = true;
      initExp = true;
      initMeasurePEEP = true;
      scanBreathingAttempt = false;
      patientTriggeredBreath = false;
      break;
  }

  T_old_us = millis();

#ifndef TX_SERIAL_TELEMETRY
  Serial.print("$");
  Serial.print(FS.Q_SLM, 5);
  Serial.print(" ");
  Serial.print(TV.measured, 5);
//  Serial.print(" ");
//  Serial.print(p_sensor.pressure_gauge_CM, 5);
  Serial.print(" ");
  Serial.print(breathPhase);
  Serial.print(" ");
  Serial.print(delta_t, 5);
  Serial.print(";");
#endif


}


/*
     This is the main Alarm Control.
        Sensor Monitoring.
        Alarm Triggering.
*/
void alarmControl() // Read Values from Installed Sensor
{

  #define COUNT_10ms  10/samplePeriod1
  #define COUNT_50ms  50/samplePeriod1
  #define COUNT_100ms 100/samplePeriod1

  //1 count = 10ms
  static unsigned int HighPeakPressAlarmCnt = 0;
  static unsigned int HighPltPressAlarmCnt = 0;
  static unsigned int LowPEEPAlarmCnt = 0;

  //ADD Power Related Alarms here
  // Low Battery
  // Battery Power In Use  

  if ((VentilatorOperationON == 1) && (breathPhase != WAIT_PHASE)) //Ventilation Related Alarms
  {
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////FAULT DETECTION////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////


    if (p_sensor.pressure_gauge_CM > pressureSetpoint) //Pressure Setpoint in Control Loop (User Input Based)
    {
      HighPeakPressAlarmCnt++;
    } else HighPeakPressAlarmCnt--;

    if (PltPrsValid == true) //To Prevent False Alarms
    {
      if (plateauPressure > (35.0 * cmH2O_to_Pa)) //FOR COVID-19 : NHS Requirement
      {
        HighPltPressAlarmCnt++;
      } else HighPltPressAlarmCnt--;
    } else HighPltPressAlarmCnt = 0;


    if (PeepValid == true) //To Prevent False Alarms
    {
      if (PEEPressure < (5.0 * cmH2O_to_Pa)) //FOR COVID-19 : NHS Requirement
      {
        LowPEEPAlarmCnt++;
      } else LowPEEPAlarmCnt--;
    } else LowPEEPAlarmCnt = 0;

    
    ////////////////////////////////////////////////////////////////////////////////
    /////////////                   SET ALARMS                      ////////////////
    /////////////Order: LOW SEVERITY (TOP) -> HIGH SEVERITY (BOTTOM)///////////////
    ///////////////////////////////////////////////////////////////////////////////

  //HIGH SEVERITY ALARMS HAVE PREFEREENCE
    if (alarm.toneFreq <= SEVERITY_LOW_FREQ) //ONLY RING IF PREVIOUSLY MUTE
    {
      if (HighPltPressAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        alarm.action = RING_ALARM;
    //    alarm.toneFreq = SEVERITY_HIGH_FREQ;
    //    alarm.timePeriod = SEVERITY_HIGH_TP;
        alarm.toneFreq = SEVERITY_LOW_FREQ; //TESTING ONLY
        alarm.timePeriod = SEVERITY_LOW_TP; //TESTING ONLY
        ErrorNumber = HIGH_PIP;
      }
    }

    if (alarm.toneFreq <= SEVERITY_MED_FREQ) //ONLY RING IF PREVIOUSLY MUTE OR PREVIOUS ALARM IS LOWER SEVERITY
    {
      if (HighPltPressAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        alarm.action = RING_ALARM;
        alarm.toneFreq = SEVERITY_MED_FREQ;
        alarm.timePeriod = SEVERITY_MED_TP;
        ErrorNumber = HIGH_PLT;
      }
    }
    
    if (alarm.toneFreq <= SEVERITY_HIGH_FREQ)  //ONLY RING IF PREVIOUSLY MUTE OR PREVIOUS ALARM IS LOWER SEVERITY
    {
      if (LowPEEPAlarmCnt >= ((int)(COUNT_100ms))) //10 loop counts = 100ms;
      {
        alarm.action = RING_ALARM;
        alarm.toneFreq = SEVERITY_HIGH_FREQ;
        alarm.timePeriod = SEVERITY_HIGH_TP;
        ErrorNumber = LOW_PEEP;
      }
    }
  }
  else
  {
    HighPeakPressAlarmCnt = 0;
    HighPltPressAlarmCnt = 0;
    LowPEEPAlarmCnt = 0;

    PeepValid = false; //To Prevent False Alarms
    PltPrsValid = false; //To Prevent False Alarms
  }
}

bool timer3InterruptTriggered = false;

void timer3ISR()
{
  timer3InterruptTriggered = true;
}
void setup()
{
  // put your setup code here, to run once:

  unsigned long pre_millis = 0;
  unsigned long prePressedTimestamp = 0;
  unsigned int isPressedTime = 0;

  plateauPressure = 0.0;
  PEEPressure = 0.0;
  peakInspPressure = 0.0;

  CVmode = VOL_CONT_MODE; //Volume Controlled Mode


  Timer3.initialize(100000);   //microseconds //0.10sec
  //Timer3.attachInterrupt(userInterface);
  Timer3.attachInterrupt(timer3ISR);

#ifdef StepGen

  /*Stepper Motor Detailed Guide with Driver: https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial */

//  Timer1.initialize(200);
//  Timer1.attachInterrupt(Timer);

#endif
  pinMode(pin_Button_OK, INPUT);
  pinMode(pin_Button_SNZ, INPUT);
  pinMode(pin_Switch_START, INPUT);
  pinMode(pin_Switch_MODE, INPUT);

  //    pinMode(pin_LmtSWT_CL1, INPUT);
  //    pinMode(pin_LmtSWT_CL2, INPUT);

  pinMode(pin_Knob_1, INPUT);
  pinMode(pin_Knob_2, INPUT);
  pinMode(pin_Knob_3, INPUT);
  pinMode(pin_Knob_4, INPUT);
#ifdef Beeper
  pinMode(pin_Beep, OUTPUT);
#endif
#ifdef Keypad_4x3
  //     keypad.addEventListener(keypadEvent);  // Add an event listener.
  keypad.setHoldTime(500);               // Default is 1000mS
  keypad.setDebounceTime(250);           // Default is 50mS
#endif
  Wire.begin();
  lcd.init(); //intializing lcd
  //lcd.begin(20, 4);
  lcd.backlight(); //setting lcd backlight

  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL_BAUD);


  initFlowSensor();

#ifdef BMP_180
  bmp_180.begin();//initialiazing BMP180
#ifdef __DEBUG
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail\n\n");
  }
#endif
#endif

  // reserve 200 bytes for the inputString:
  slave.AckStr.reserve(50);

  //#ifdef TX_SERIAL_TELEMETRY
  //    Serial1.begin(SERIAL_BAUD);
  //#endif
  //Serial.print("Open Source Ventilator Pakistan Prottype 1 SW Ver: "); Serial.println(CodeVer);
  //sprintf(tempChar, "*OpenVentPk Ver %1d.%1d*",CODE_VER_MAJOR,CODE_VER_MINOR);;Serial.print(tempChar);


  //    noInterrupts();
  pre_millis = millis();
  //  Serial.println("Entering Warmpup");

  while ((millis() - pre_millis) < WARM_UP_TIME)
  {
//    Serial.println(millis() - pre_millis);
    prePressedTimestamp = millis();
    isPressedTime = millis() - prePressedTimestamp;

    //    Serial.print("Motor Current Pos: "); Serial.println(stepper.currentPosition());


    while (OkButton == HIGH && SnoozeButton == HIGH && isPressedTime < 2000)
    {
      isPressedTime = millis() - prePressedTimestamp;
      if (timer3InterruptTriggered)
      {
        DevModeDetectionInProg = 1;
        userInterface();
        timer3InterruptTriggered = false;
      }
    }
    DevModeDetectionInProg = 0;

    if (isPressedTime >= 2000)
    {
      devMode = 1;
      WarmUpFlag = 0;
      devModeFunc(); //MOVE LCD_DISPLAY to Userinterface
    }
    if (timer3InterruptTriggered)
    {
      userInterface();
      timer3InterruptTriggered = false;
    }
  }
  //    Serial.println("Exiting Warmpup");

  //    interrupts();

  noInterrupts();
  //Fetch Motor Speed and Volume displace SF from EEPROM
  eeget();    // read startup parameters (either from EEPROM or default value)

  spStatus.curI_E_Section = reqExpirationRatioIndex;
  spStatus.curBPM = reqBPM;
  spStatus.curTV = reqVolume;
  spStatus.curOP = reqPressure;

  bpmSetpoint = reqBPM;                 // Start from these values without sweep
  volumeSetpoint = reqVolume;
  pressureSetpoint = reqPressure;
  expirationRatioSetpoint = I_E_SampleSet[reqExpirationRatioIndex - 1];

  interrupts();

  tick1 = millis();
  tick2 = millis();
}

void devModeFunc() //Developer Mode
{
  bpmSetpoint = reqBPM; // Start from these values without sweep
  volumeSetpoint = reqVolume;
  pressureSetpoint = reqPressure;
  expirationRatioSetpoint = I_E_SampleSet[reqExpirationRatioIndex - 1];

  while (devMode == 1)
  {
    userInterface();
    delay(1000);
  }
}

void Ventilator_Control()
{
  static boolean initIns = true;
  static boolean initHld = true;
  static boolean initExp = true;
  static boolean initWait = true;
  static boolean runMotor = true;
  static unsigned int Tin = 0;
  static unsigned int Tex = 0;
  static unsigned int Th = 0; //ms
  static unsigned int Ttrigger = 300; //ms
  static unsigned long Tcur = 0;
  static unsigned long BreathStartTimestamp = 0;

  static float reqMotorPos = 0.0; //mm
  static float Vin = 0.0; //mm/s
  static float Vex = 0.0;  //mm/s
  static float RPMin   = 0.0;
  static float RPMex   = 0.0;

  static long stepIn = 0;
  static long stepEx = 0;
  static long periodIn = 0; //us
  static long periodEx = 0; //us

  static boolean init = true;

  //    noInterrupts();
//  CVmode = VOL_CONT_MODE; //Proto-1
//  assistControl = 0;

  if (init)
  {
    breathLength = (int)(60000 / bpmSetpoint);
    // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
    Tex = (int)((breathLength - Th) / (1 + expirationRatioSetpoint)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
    Tin = (int)(Tex * expirationRatioSetpoint);
    if (holdManeuver) Th = holdDur_ms; else Th = 0;
    slave.runAck = 0;
    init = false;
    Tcur = breathLength;
  }

  if (activateVentilatorOperation == 1)
  {
    VentilatorOperationON = 1;
    initWait = true;

    if (assistControl == 1) {
      if (patientTriggeredBreath)
      {
        Tcur = breathLength; //Start a new inhale Cycle 
      }
    }
    

    if (Tcur >= breathLength)
    {
      Tcur = 0;
      bpmSetpoint = reqBPM;                 // Load Fresh User Settings
      volumeSetpoint = reqVolume;
      pressureSetpoint = reqPressure;
      expirationRatioSetpoint = I_E_ExpFactor[(int)reqExpirationRatioIndex - 1];

      breathLength = (int)(60000 / bpmSetpoint);
      if (holdManeuver) Th = holdDur_ms; else Th = 0;
      // Take the hold time out of the exhale cycle. Do this to ensure respitory rate is correct.
      Tin = (int)((breathLength - Th) / (1 + expirationRatioSetpoint)); // if I/E ratio = 0.5 ; it means expiration is twice as long as inspiration
      Tex = (int)(breathLength - Th - Tin);
      if (CVmode == VOL_CONT_MODE) {
        reqMotorPos = volumeSetpoint / LINEAR_FACTOR_VOLUME; //mm
        Vin = reqMotorPos / ((float)Tin / 1000.0f); // mm/s
        Vex = reqMotorPos / ((float)Tex / 1000.0f); // mm/s
        RPMin = (Vin / LIN_MECH_mm_per_rev) * 60.0;
        RPMex = (Vex / LIN_MECH_mm_per_rev) * 60.0;
        stepIn = (long)((reqMotorPos / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV);
        stepEx = (long)(stepIn + ((2.0 / LIN_MECH_mm_per_rev) * STEPPER_MICROSTEP * STEPPER_PULSES_PER_REV));
        periodIn = (long)((((float)Tin / 1000.0) / stepIn) * 1000000); //us
        periodEx = (long)((((float)Tex / 1000.0) / stepIn) * 1000000); //us
    }
    else //PRESS_CONT_MODE 
    {
    }
      BreathStartTimestamp = millis();

#ifdef __DEBUG
          static int i = 0;
           Serial.print("In Ventilator Control: "); Serial.println(i++);
           Serial.print("Breathing Length:      "); Serial.println(breathLength);
           Serial.print("Inspiration Time:      "); Serial.print(Tin); Serial.println(" ms");
           Serial.print("Expiration Time:       "); Serial.println(Tex); Serial.println(" ms");
           Serial.print("targetPosition:        "); Serial.println(reqMotorPos); Serial.println(" mm");
           Serial.print("Motor Speed Insp:      "); Serial.println(Vin); Serial.println(" mm/s");
           Serial.print("Motor Speed Exp:       "); Serial.println(Vex); Serial.println(" mm/s");
           Serial.print("RPM Insp:              "); Serial.println(RPMin);
           Serial.print("RPM Exp:               "); Serial.println(RPMex);
           Serial.print("Steps Insp:            "); Serial.println(stepIn);
           Serial.print("Steps Exp:             "); Serial.println(stepEx);
           Serial.print("Period Insp:           "); Serial.println(periodIn); Serial.println(" us");
           Serial.print("Period Exp:            "); Serial.println(periodEx); Serial.println(" us");
#endif
    }
    Tcur = millis() - BreathStartTimestamp;

      if (Tcur <= Tin)
      {
        if (initIns)
        {
        //  Serial.println("Inspiration Cycle");
//          slave.runAck = 0;
          runMotor = true;
          initHld = true;
          initIns = false;
          initExp = true;
        }        
        breathPhase = INSPIRATION_PHASE;
//        if (TV.measured < volumeSetpoint)
        if (runMotor && (slave.runAck == 0 || slave.runAck == 2)) //!setpointAchieved && //CMD NOT RECEIVED
        {
/*          Serial2.print("#RUN ");
          Serial2.print(stepIn);Serial2.print(" ");
          Serial2.print(periodIn);Serial2.print(" ");
          Serial2.print("1\r"); //1 for towards Ambu Bag Dir
          */
         txSlaveCMD(RUN, periodIn, stepIn, "1");
         runMotor = false;

          slave.lastCMD_ID = RUN;
        }
        PEEPMesaureFlag = 0;
      }
      else if ((Tcur > Tin) && (Tcur <= (Tin + Th)))
      {
        if (initHld)
        {
         //   Serial.println("HOLD Cycle");
//          slave.stopAck = 0;
          initHld = false;
          initIns = true;
          initExp = true;
        }        
        breathPhase = HOLD_PHASE;
//        if (slave.stopAck == 0) //CMD NOT RECEIVED
//          Serial2.print("#STOP\r");
//            txSlaveCMD(STOP);
//        Serial.println("STOP: HOLD MANEUVER");
//        slave.lastCMD_ID = STOP;
      }
      else if ((Tcur > (Tin + Th)) && (Tcur < (Tin + Th + Tex)))
      {
        if (initExp)
        {
    //  Serial.println("Exp Cycle");

         // slave.runAck = 0;
         runMotor = true;
          initHld = true;
          initIns = true;
          initExp = false;
        }                
        breathPhase = EXPIRATION_PHASE;

        if (runMotor && (slave.runAck == 0 || slave.runAck == 2)) //CMD NOT RECEIVED
        {
/*          Serial2.print("#RUN ");
          Serial2.print(stepEx);Serial2.print(" ");
          Serial2.print(periodEx);Serial2.print(" ");
          Serial2.print("0\r"); //0 for away from Ambu Bag Dir
*/
         txSlaveCMD(RUN, periodEx, stepEx, "0");
         runMotor = false;
          slave.lastCMD_ID = RUN;
        }

        if ((Tcur >= (Tin + Tex)) && (Tcur < (Tin + Th + Tex)))
        {
          PEEPMesaureFlag = 1;
        }

        if (assistControl == 1) {
        if ((Tin + Th + Tex - Tcur) < Ttrigger) 
        {
          scanBreathingAttempt = true;
        }        }
      }
  }
  else
  {
      //      Serial.println("Ventilator Operation Halt");
    if (initWait) {slave.homeAck = 0; initWait = false;}
    initHld = true;
    initIns = true;
    initExp = true;
    VentilatorOperationON = 0;
    breathPhase = WAIT_PHASE;
    Tcur = breathLength; // This will always start inspiration breath cycle on setting switch to start position
    PeepValid = false; //To Prevent False Alarms
    PltPrsValid = false; //To Prevent False Alarms


    if (slave.homeAck == 0)
    {
//        Serial2.print("#HOME 2000\r");
        txSlaveCMD(HOME, 2000);
        slave.lastCMD_ID = HOME;
    }
    PEEPMesaureFlag = 0;
  }
  //    interrupts();
}

#ifdef TX_SERIAL_TELEMETRY
void GetTelData()
{

  static boolean init = true;
  unsigned int TEL_BYTE = 0x00;
  if (init)
  {
    TEL.Time = 0;
    TEL.txUpdateRate = 0;
    TEL.txPktCtr = 0;
    TEL.FDCB = 0xFF;
    init = false;
  }

  TEL.Time += (samplePeriod1);

  if ((TEL.Time % 20) == 0)
  {
    TEL.mTV = TV.measured; //ml
    //TEL.mTV = constrain(TEL.mTV, 0.0, 1000.0);
    TEL.mTVinsp = TV.inspiration; //ml
    TEL.mTVexp = TV.expiration; //ml
    TEL.mPressure = p_sensor.pressure_gauge_CM; //cmH2O
    TEL.mFlowRate = FS.Q_SLM; //SLPM
    TEL.mPEEP = PEEPressure * Pa2cmH2O;
    TEL.mPltPress = plateauPressure * Pa2cmH2O;
    TEL.mFiO2 = 0.0;
    TEL.minuteVentilation = TV.minuteVentilation;
    TEL.mPeakPressure = peakInspPressure;
    TEL.mRR = 0.0;
    TEL.staticCompliance = TV.staticCompliance;
    TEL.spTrigger = flowTrigger;

  //  TEL.spTV = (int)(reqVolume);
  //  TEL.spInsPressure = int(reqPressure * Pa2cmH2O);
    TEL.spTV = (int)(spStatus.newTV);
    TEL.spInsPressure = int(spStatus.newOP  * Pa2cmH2O);
    TEL.spExpPressure = 5 * Pa2cmH2O; //0;

    TEL.spFiO2 = 0;
  //  TEL.spBPM = (int)(reqBPM);
  //  TEL.spIE_Inhale = I_E_InpsFactor[reqExpirationRatioIndex - 1];
  //  TEL.spIE_Exhale = I_E_ExpFactor[reqExpirationRatioIndex - 1];
    TEL.spBPM = (int)(spStatus.newBPM);
    TEL.spIE_Inhale = I_E_InpsFactor[spStatus.newI_E_Section - 1];
    TEL.spIE_Exhale = I_E_ExpFactor[spStatus.newI_E_Section - 1];
    TEL.patientWeight = patientWeight;

    TEL.statusByteError = ErrorNumber;
    TEL_BYTE = 0x00;
    TEL_BYTE |= breathPhase & 0x03;
    if (CVmode == PRESS_CONT_MODE)  TEL_BYTE |= 0x04;
    if (assistControl == 1)  TEL_BYTE |= 0x08;
    if (VentilatorOperationON == 1)  TEL_BYTE |= 0x10;
    if (selfTestProg == ST_IN_PROG)
      TEL_BYTE |= 0x20;
    else if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_FAIL))
      TEL_BYTE |= 0x40;
    else if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_PASS))
      TEL_BYTE |= 0x60;
    else //NOT INIT
      TEL_BYTE &= 0x9F; //Clear D5 and D6
    if (spStatusAllowChange == 1) TEL_BYTE |= 0x80; //D7 High if Settings Unsaved

    TEL.statusByte1 = TEL_BYTE;
    TEL.FDCB = 0xCC;
  }
}
#endif



void loop()
{

  unsigned long start_Ts = 0;
  WarmUpFlag = 0;
#ifdef StepGen
//  digitalWrite(pin_Stepper_SLP, HIGH);
#endif
  if (millis() > (tick1 + samplePeriod1))
  {
    start_Ts = micros();
    tick1 = millis();
    readSensors();
    Monitoring();

//    if (slave.strComplete == true)
//    {
//      decodeSlaveTel();
//      slave.AckStr = "";
//      slave.strComplete = false;
//    }
    

    if (selfTestProg != ST_COMPLETE)
      selfTest();
    else
    {
//      alarmControl();
    }
#ifdef ActiveBeeper
    beep(); //alarmAction = RING_ALARM, SNOOZE_ALARM; alarmSeverity = SEVERITY_HIGH, SEVERITY_MED, SEVERITY_LOW, SEVERITY_MUTE
#endif

#ifdef TX_SERIAL_TELEMETRY
    GetTelData(); //Called at 100Hz
    Prepare_Tx_Telemetry(); //Called at 100Hz
#endif
#ifndef TX_SERIAL_TELEMETRY
Serial.print("Busy Time 1: "); Serial.println(micros()-start_Ts);
#endif
  }
  if ((selfTestProg == ST_COMPLETE) && (selfTestStatus == ST_PASS)) // I am not writing control loop inside 100Hz loop to keep both loop rates independant
  {
    if (millis() > (tick2 + samplePeriod2))
    {
      start_Ts = micros();
      tick2 = millis();
      Ventilator_Control(); //Mandatory Volume Controlled Mode Only
      #ifndef TX_SERIAL_TELEMETRY
Serial.print("Busy Time 2: "); Serial.println(micros()-start_Ts);
#endif

    }
  }
  if (timer3InterruptTriggered)
  {
    userInterface();
    timer3InterruptTriggered = false;
  }
}


/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent2() {
    while (Serial2.available()) {
      // get the new byte:
      char inChar = (char)Serial2.read();
      // add it to the inputString:
      slave.AckStr += inChar;
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      if (inChar == '\r') {
     #ifndef TX_SERIAL_TELEMETRY
        Serial.println(slave.AckStr);
     #endif
        slave.strComplete = true;
      }
    }
    if (slave.strComplete == true)
    {
      decodeSlaveTel();
      slave.AckStr = "";
      slave.strComplete = false;
    }
}

void decodeSlaveTel()
{
  String message = "";

    for (int i = 0; i < slave.AckStr.length(); i++)
    {
      if (slave.AckStr[i] == '#')
      {
        if ((slave.AckStr[i+1] == 'R') && (slave.AckStr[i+2] == 'U') && (slave.AckStr[i+3] == 'N'))
        {
          message = slave.AckStr[i+5];
          slave.runAck = message.toInt();
    //      Serial.print("runAck = ");Serial.println(slave.runAck);
          break;
        }
        else if ((slave.AckStr[i+1] == 'S') && (slave.AckStr[i+2] == 'T') && (slave.AckStr[i+3] == 'O') && (slave.AckStr[i+4] == 'P'))
        {
          message = slave.AckStr[i+6];
          slave.stopAck = message.toInt();
      //    Serial.print("stopAck = ");Serial.println(slave.stopAck);
          break;
        }
        else if ((slave.AckStr[i+1] == 'H') && (slave.AckStr[i+2] == 'O') && (slave.AckStr[i+3] == 'M') && (slave.AckStr[i+4] == 'E'))
        {
          message = slave.AckStr[i+6];
          slave.homeAck = message.toInt();  
//          Homing_Done_F = slave.homeAck;
     //     Serial.print("homeAck = ");Serial.println(slave.homeAck);
          break;        
        }
      }
    }
}


void txSlaveCMD(int CMD_ID, unsigned int period=0, unsigned int pulses=0, String dir="0")
{
  String cmdString = "";
  switch (CMD_ID)
  {
  case RUN:
    cmdString = "#RUN " + String(pulses) + " " + String(period) + " " + dir + "\r";
    break;
  case HOME:
    cmdString = "#HOME " + String(period) + "\r";
    break;
  case STOP:
    cmdString = "#STOP\r";
    break;
  default:
    break;
  }
  Serial2.print(cmdString);
#ifndef TX_SERIAL_TELEMETRY
  Serial.println(cmdString);
#endif
}