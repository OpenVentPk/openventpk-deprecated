#ifndef __HEADER_H
#define __HEADER_H

//#include <Arduino.h> //For PlatformIO

#define CODE_VER_MAJOR 2
#define CODE_VER_MINOR 7 //(Took hold time out of expiration, Homing Done moved to interrupt, Plateau Pressure and PEEP measurements moved to readsensors function)

//************************   DEVICE OPERATIONAL PARAMETERS   ************************/
/*
       WARNING : When changing min and max value, manually change the text in the SerialCommand procedure accordingly

       The values are taken from various documents including :
         https://www.gov.uk/government/publications/coronavirus-covid-19-ventilator-supply-specification/rapidly-manufactured-ventilator-system-specification
         https://docs.google.com/document/d/1h77FkKXqPOwVqfOj-PIJSjYX9QiEx69Av2gYuzqZolw/edit#

       some changes have been made to havee a lower default volume in case the machine is used
       with an infant to prevent damaging their lungs with an adult setting.*/

#define minBPM 8.0             // minimum respiratory speed
#define defaultBPM 12.0         // default respiratory speed
#define stepBPM 1.0             // adjustment step for respiratory speed
#define maxBPM 35.0             // maximum respiratory speed
#define maxBPMchange 0.2        // maximum respiratory speed change in proportion of final value per beat (1=100%)
#define minVolume 200.0         // minimum respiratory volume in milliliters
#define defaultVolume 500.0     // default respiratory volume in milliliters
#define stepVolume 100.0        // adjustment step for respiratory volume in milliliters
#define maxVolume 800.0         // maximum respiratory volume in milliliters
#define maxVolumeChange 0.25    // maximum respiratory volume change in proportion of final value per beat (1=100%)
#define minPressure 0.00        // minimum compression for the ambu-bag in Pa
#define stepPressure 500.00     // adjustment step for compression for the ambu-bag in Pa
#define defaultPressure 2942.00 // default compression for the ambu-bag in Pa
#define maxPressure 3922.66     // maximum compression for the ambu-bag in Pa //approx 40cH20
#define maxPressureChange 0.5   // maximum compression for the ambu-bag change in proportion of final value per beat (1=100%)
#define minWeight 2.00          // minimum compression for the ambu-bag in Pa
#define maxWeight 150.00        // minimum compression for the ambu-bag in Pa
#define minPot 0
#define maxPot 1023
#define defaultExpirationRatioIndex 3 //Corresponds to 1:2 see definition: I_E_SampleSet

#define ADC_TO_VOLTS 0.004887585532746823 //0.004887585532746823 is from 5v/1023

#define ambientPressureFilter 0.002       // IIR filtering ratio (lower value produce longer time constant)
#define avgPressureFilter 0.1             // IIR filtering ratio (lower value produce longer time constant)
#define defaultMeasuredPressure 105000.00 // Pressure in Pa returned when no sensor is found
#define defaultAmbientPressure 105000.00  // assumed ambiant pressure in Pa returned when no sensor is found
/*******************************   MOTOR PARAMETERS FOR STEPPER MOTOR   *******************************

       These values will be highly dependant on mechanical design.
       When modifying the values, always check with an oscilloscope that the movements completes
       without overlaps and without too much idle time.
       Also check that the motor can properly follow the speed acceleration required under load.
       Wrong values can cause unpredictables moves and motor stalls.
       The worst case scenario is at max BPM, with max volume, max sync period and max ratio
       With default parameters, the whole compression can become as short as 250 ms
*/

#define FULL_SCALE_VOLUME             800.0f  //ml
#define FULL_SCALE_LENGTH             35.0f  //mm
#define LINEAR_FACTOR_VOLUME          22.86f
#define LIN_MECH_mm_per_rev           5.0f
#define STEPPER_MICROSTEP             2.0f
#define STEPPER_PULSES_PER_REV        200.0f
/*******************************   HARDWARE OPTIONS   *******************************
   It's normal for the program to not compile if some of these are undefined as they need an alternative*/

//******************************   IMPIED DEFINITIONS  ********************************
#ifdef ActiveBeeper
#define Beeper
#endif
#ifdef PassiveBeeper
#define Beeper
#endif

#ifdef MS4525DO
#define I2C_ADDRESS_MS4525DO 0x28 /**< 7-bit address. Depends on the order code (this is for code "I") */
#endif



#define RING_ALARM 1
#define SNOOZE_ALARM 0

//Frequency in HERTZ
#define SEVERITY_HIGH_FREQ 3000
#define SEVERITY_MED_FREQ 2000
#define SEVERITY_LOW_FREQ 1000

//Durations in milliseconds
#define SEVERITY_HIGH_TP 250 
#define SEVERITY_MED_TP 750
#define SEVERITY_LOW_TP 1500

//******************************   ERROR NUMBERS  ********************************
#define BATTERY_IN_USE 1
#define CIRCUIT_INTEGRITY_FAILED 2
#define HIGH_RR 3
#define HIGH_FIO2 4
#define HIGH_PEEP 5
#define HIGH_PLT 6
#define HIGH_PIP 7
#define LOW_PIP 8
#define LOW_FIO2 9
#define LOW_PEEP 10
#define LOW_PLT 11
#define OXYGEN_FAILURE 12
#define LOW_TV 13
#define HIGH_TV 14
#define START_SWT_ERROR 15
#define LOW_MV 16
#define HIGH_MV 17
#define VENT_CKT_DC 18
#define MECH_INTEGRITY_FAILED 19
#define HOMING_NOT_DONE_ERROR 20
#define OPS_96_HRS 21
#define FLOW_SENSOR_DISCONNECTED 22

//******************************   MACROS  ********************************

#define WAIT_PHASE 0
#define INSPIRATION_PHASE 1
#define HOLD_PHASE 2
#define EXPIRATION_PHASE 3

#define ST_NOT_INIT 0
#define ST_IN_PROG 1
#define ST_COMPLETE 2
#define ST_FAIL 0
#define ST_PASS 1
#define WARM_UP_TIME 10 * 1000

#define DC_MOTOR_IN_USE 1
#define STEPPER_IN_USE 0

#define VOL_CONT_MODE 0
#define PRESS_CONT_MODE 1

/**********Pressure sensors parameters*************/
#define BMP180_IN_USE 1
#define BMP280_IN_USE 2

#define MPXV7002DP_IN_USE 0
#define MPX2010D_IN_USE 1
#define MPX10DP_IN_USE 2
#define MS4525_IN_USE 3

#define K 520 //constant cofficent for flow rate q
#define P_min -1.0f
#define P_max 1.0f
#define PSI_to_Pa 6894.757f
/***************end***********************/

#define Pa2cmH2O 0.0101972
#define cmH2O_to_Pa 98.0665

//********************************   CONNECTION PINS   ********************************
// ATmega2560-Arduino Pin Mapping: https://www.arduino.cc/en/Hacking/PinMapping2560
#ifdef I2C
#define pin_SDA 20
#define pin_SCL 21
#endif

#define MPX_IN A4

#ifdef ActiveBeeper
#define pin_Beep 37 //Any Pin
#endif
#ifdef PassiveBeeper
#define pin_Beep 37 //PWM Pin
#endif

#ifdef Led
#define pin_LED1 22
#endif

#define pin_Button_OK 42
#define pin_Button_SNZ 43
#define pin_Switch_START 44
#define pin_Switch_MODE 45

#define pin_Knob_1 A12 //VR1 // I/E Ratio
#define pin_Knob_2 A13 //VR2 // BPM
#define pin_Knob_3 A14 //VR3 // Tidal Volume
#define pin_Knob_4 A15 //VR4 // Max Pressure

//#define pin_LmtSWT_OP1 46 //HOME POSITION
//#define pin_LmtSWT_OP2 47
#define pin_LmtSWT_CL1 48
#define pin_LmtSWT_CL2 49

#ifdef stepDirMotor
//#define pin_Stepper_DIR 4
//#define pin_Stepper_STEP 3
//#define pin_Stepper_SLP 2 //Active Low
//#define pin_Stepper_RST      15 //Active Low
//#define pin_Stepper_MS3 25
//#define pin_Stepper_MS2 24
//#define pin_Stepper_MS1 23
//#define pin_Stepper_EN1     23 //Active Low
//#define pin_Stepper_EN2     24 //Active Low

//Instructions for using moveTo function of Accel Stepper Class
// moveTo() also recalculates the speed for the next step.
// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().

#define FULL_STEP 0 //For Use with Port Registers: Not Yet Implemented
#define HALF_STEP 1
#define QUARTER_STEP 2
#define EIGHTH_STEP 3
#define SIXTEENTH_STEP 7

#endif

#define pin_M1_POT A11 //VR5

#define SERIAL_BAUD 115200 // Serial port communication speed

#ifdef E2PROM
#define eeStart 48 // EEPROM Offset for data
#define ee_reqBPM eeStart
#define ee_reqVol ee_reqBPM + (sizeof(float));
#define ee_reqPres ee_reqVol + (sizeof(float));
#define ee_MSpdSF ee_reqPres + (sizeof(float));
#define ee_MVolSF ee_MSpdSF + (sizeof(float));
#endif

#define samplePeriod1 10 // 5 ms sampling period
#define samplePeriod2 10 // 10 ms Control Loop

#define highPressureAlarmDetect 10 // delay before an overpressure alarm is triggered (in samplePeriod increments)

//*******************************   REQUIRED LIBRARIES   *******************************
#ifdef I2C
#include <Wire.h>              // I2C Library 2 wire Protocol
#include <LiquidCrystal_I2C.h> // Library for LCD //Liquid Crystal I2C by Frank de Brabander

#ifdef BMP_180
#include <SFE_BMP180.h> // BMP180 library to read bmp180
#define ALTITUDE 1655.0 //this should be according to the altitude of the user
SFE_BMP180 bmp180;
#endif
#endif

#ifdef E2PROM
#include <EEPROM.h> // read / write to the processor's internal EEPROM
#endif


#ifdef StepGen
#include "TimerOne.h" // Timer component
//  By Jesse Tane, Jérôme Despatis, Michael Polli, Dan Clemens, Paul Stroffregen
//  https://playground.arduino.cc/Code/Timer1/
#endif

#include "TimerThree.h" // Timer3 component

#ifdef PID_CONTROL

//----------------------------------------------------------------------
//PID stuff for stepper motors
//----------------------------------------------------------------------
#include <PID_v1.h>

#define max_stepCmd 20000.0  //max output value from PID
#define min_stepCmd -20000.0  //min output value from PID
#define PID_sample_time 10  //how often the PID algorithm evaluates in ms
#define deadband 3        // +/- number of counts to not respond 6

#endif
//***************************************   FUNCTION PROTOTYPES   ***************************************
#ifdef StepGen
void Timer();
void setMicroSteps(int MicrostepResolution);
#endif

void selfTest();
void readSensors();
void Monitoring();
void alarmControl();
void devModeFunc();
void Ventilator_Control();

#ifdef Beeper
void beep();
#endif

#ifdef TX_SERIAL_TELEMETRY
void GetTelData();
#endif
//#ifdef E2PROM
//#endif
void eeput(int n); // records to EEPROM (only if values are validated)
void eeget();

#ifdef PID_CONTROL
void PID_setup();
void calc_PID();
#endif

void txSlaveCMD(int CMD_ID, unsigned int period=0, unsigned int pulses=0, String dir="0");
void decodeSlaveTel();
//***************************************   END   ***************************************

struct P_Sensor
{
  float bmp_pressure = 0,
        bmp_temperature = 0;
  float diff_press_PSI = 0,    //differential pressure from sensor in psi
      diff_press_pa = 0,       //differential pressure form sensor in pa
      q = 0;                   //flow rate in lit/min
  float pressure_gauge = 0;    // Pressure from the sensor in Pa
  float pressure_gauge_CM = 0; // Pressure from the sensor in cmH2O
};

struct setpointStatus
{
  int curI_E_Section; //Patient Weight
  int curBPM;         // BPM
  int curTV;          //Tidal Volume Setpoint
  int curOP;          //Overpressure limit
  int newI_E_Section; //Patient Weight
  int newBPM;         // BPM
  int newTV;          //Tidal Volume Setpoint
  int newOP;          //Overpressure limit
};

struct Alarm
{
  unsigned int action = SNOOZE_ALARM;
  unsigned int toneFreq = SNOOZE_ALARM;
  unsigned int timePeriod = SNOOZE_ALARM; //milliseconds
};

struct TidalVolume
{
  float measured = 0.0;
  float inspiration = 0.0;
  float expiration = 0.0;
  float minuteVentilation = 0.0;
  float staticCompliance = 0.0; // (ml / cmH2O)

};

struct Slave
{
  int lastCMD_ID = 0;
  int runAck = 0;
  int stopAck = 0;
  int homeAck = 0;
  boolean strComplete = false;
  String AckStr = "";
};

#define TIME_VAR 0
#define FLOW_VAR 1
#define PRESS_VAR 2
#define VOL_VAR 3

#define RUN   1
#define STOP  2
#define HOME  3

#define HOMING_CMD_NOT_SENT 0
#define CMD_RECEIVED 1
#define CMD_COMPLETE 2
#define CMD_ERROR    3

#ifdef PID_CONTROL

struct PID_TYPE
{
  double setpoint = 500.0;
  double output = 0.0;
  double input = 0.0;
  double Kp = 0.03;  //proportional gain
  double Ki = 0.0;
  double Kd = 0.0;
};
#endif
//***************************************   END   ***************************************
#endif
/* NOTE*****************************************************************************************************************8
    Minor version change required in Future.
         This version allows the replacement of the buggy Wire.h arduino library  (can hang
         the controller) with acorrect version known as jm_Wire
         https://github.com/ermtl/Open-Source-Ventilator/blob/master/OpenSourceVentilator/README.md
         'Wire.h' is still the default library to prevent compiler errors.
         The processor's hardware watchdog can now be enabled (off by default, use with care, you
         risk bricking your processor.
         Modularisation is getting better (work in progress)

  #include <jm_Wire.h>              // I2C protocol, contains a workaround for a longstanding issue with the Wire library
                                  // by Jean-Marc Paratte - https://github.com/jmparatte/jm_Wire
                                  // The library is optional during developpement (unless you encounter lockups)
                                  // but must be used for any production software. To use it, you need to change
                                  // the #include "Wire.h" line in all libraries that use the I2C bus.


******************************************************************************************************************************/