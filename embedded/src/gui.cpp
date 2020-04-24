#include "header.h"
#include "gui.h"

#define debounceDelay 600 //ms

/**********************************Global********************************/
float WUTimeStart_LCD = 0.0; // This need to be Global
int WUTimeStartF_LCD = 0;    // This need to be Global
int DisplayPageNum = 2;      // This need to be Global
int SnoozKeyPressedCtr = 0;
int LCDDisplayCtr = 0;
char  tempChar1[3], tempChar2[17];
/*************************extern From main.cpp***************************/

extern LiquidCrystal_I2C lcd;
extern struct setpointStatus spStatus;
extern struct P_Sensor p_sensor;
#ifdef Beeper
extern struct Alarm alarm;
#endif

extern int CVmode; // CV or CP mode indicator;
extern int selfTestStatus;
extern int spStatusAllowChange;
extern int reqExpirationRatioIndex; // The proportion of each breathing cycle that is spent breathing in compared to breathing out
extern int activateVentilatorOperation;
extern int WarmUpFlag;
extern int devMode;
extern int motorInUse;
extern int pressSnsrInUse;
extern int VentilatorOperationON;
extern int OkButton;
extern int SnoozeButton;
extern int Motor1_HomePos_Hit_F;
extern int Motor2_HomePos_Hit_F;
extern int CVmode;
extern int ErrorNumber;
extern int selfTestProg;
extern int DevModeDetectionInProg;
extern int devMode;

extern int I_E_InpsFactor[5];
extern int I_E_ExpFactor[5]; 


extern char  tempChar[20];

extern float plateauPressure, /* Plateau pressure is the pressure that is applied by the ventilator to the small airways and alveoli.
                                 It is measured at end-inspiration with an inspiratory hold maneuver.*/
       PEEPressure,              // Positive end-expiratory pressure (PEEP)
       reqBPM,       // respiratory frequency
       reqVolume,             // respiratory volume in milliliters
       reqPressure;           // compression for the ambu-bag in Pa
/*********************************END************************************/

inline static void LCD_Display(int WarmUpFlag, int TotalWarmUpTime_msec, int SnoozKeyPressed, struct setpointStatus sp, int DevModeDetectionInProg ,
                               int DevMode, int MotorInUse, int PrSnrInUse,
                               int VenMode, int SysON, int ErrorNum,
                               int PltPrsr, int PEEP,
                               int SelfTestInPrg, int CalibInPrg, int SpdSF, int StpSF)
{
  static unsigned long milli_old = 0;
  lcd.clear();
  //Delay(10);
  lcd.home();
  lcd.noCursor();
  lcd.display();
  if (WarmUpFlag == 1)
  {
    if (WUTimeStart_LCD == 0)
    {
      WUTimeStart_LCD = millis();
      WUTimeStartF_LCD = 1;
    }
    lcd.setCursor(0, 0);
    sprintf(tempChar, "*OpenVentPk Ver %1d.%1d*", CODE_VER_MAJOR, CODE_VER_MINOR);
    lcd.print(tempChar);

    lcd.setCursor(0, 1);
    sprintf(tempChar, "System Will Start In");
    lcd.print(tempChar);

    lcd.setCursor(0, 2);
    sprintf(tempChar, "     %02d seconds     ", (int((TotalWarmUpTime_msec - int(millis() - WUTimeStart_LCD)) * 0.001)));
    lcd.print(tempChar);//int((TotalWarmUpTime_msec - int(millis() - TimeStart_LCD)) * 0.001));

    lcd.setCursor(0, 3);
    if (DevModeDetectionInProg == 1)
      sprintf(tempChar, "Dev Mode Detecting..");
    else
      sprintf(tempChar, "********************");

    lcd.print(tempChar);
  }
  else
  {
    WUTimeStartF_LCD = 0;

    if (DevMode == 1)
    {
      lcd.setCursor(0, 0);
      sprintf(tempChar, "*Dev Mode, Ver:%1d.%1d*", CODE_VER_MAJOR, CODE_VER_MINOR);
      lcd.print(tempChar);

      lcd.setCursor(0, 1);
      if (MotorInUse == STEPPER_IN_USE)
        sprintf(tempChar, "Motor Selctd - Stepr");
      else
        sprintf(tempChar, "Motor Selctd - DC");

      lcd.print(tempChar);

      lcd.setCursor(0, 2);
      if (PrSnrInUse == MPXV7002DP_IN_USE)
        sprintf(tempChar, "Prsr Snsr - MPXV7002");
      else if (PrSnrInUse == MPX2010D_IN_USE)
        sprintf(tempChar, "Prsr Snsr - MPX2010");
      else if (PrSnrInUse == MPX10DP_IN_USE)
        sprintf(tempChar, "Prsr Snsr - MPX10DP");
      else if (PrSnrInUse == MS4525_IN_USE)
        sprintf(tempChar, "Prsr Snsr - MS4525D");
      else
        sprintf(tempChar, "Prsr Snsr - Spare");

      lcd.print(tempChar);

      lcd.setCursor(0, 3);
      sprintf(tempChar, "Restart for Nrml Ops");
      lcd.print(tempChar);
    }
    else
    {
      if ((SelfTestInPrg == ST_IN_PROG) || (CalibInPrg == ST_IN_PROG))
      {
        lcd.setCursor(0, 0);
        if (SelfTestInPrg == ST_IN_PROG)
          sprintf(tempChar, "Self Test  : In Prog");
        else if (SelfTestInPrg == ST_COMPLETE)
          sprintf(tempChar, "Self Test  : Done");
        else
          sprintf(tempChar, "Self Test  : Nt Done");

        lcd.print(tempChar);

        lcd.setCursor(0, 1);
        if (CalibInPrg == ST_IN_PROG)
          sprintf(tempChar, "Calib    : In Prog");
        else if (CalibInPrg == ST_COMPLETE)
          sprintf(tempChar, "Calib  : Done");
        else
          sprintf(tempChar, "Calib  : xxxx");

        lcd.print(tempChar);

        lcd.setCursor(0, 2);
        sprintf(tempChar, "SpdSF :%02d, StpSF :%02d", SpdSF, StpSF);
        lcd.print(tempChar);

        lcd.setCursor(0, 3);
        switch (ErrorNum)
        {
        case HOMING_NOT_DONE_ERROR:
          sprintf(tempChar2, "   HOMING NOT DONE  ");
          break;
        case START_SWT_ERROR:
          sprintf(tempChar2, " VENTILATION ACTIVE ");
          break;
        case FLOW_SENSOR_DISCONNECTED:
          sprintf(tempChar2, " Flow Sensor Error  ");
        break;
        default:
        sprintf(tempChar, "Wait 4 ST/Calib Done");
          break;
        }


        lcd.print(tempChar);
      }
      else
      {
        if (spStatusAllowChange == 0)
        {
          if (SnoozKeyPressed == 1)
          {
            SnoozKeyPressedCtr++;
            if (SnoozKeyPressedCtr >= 6)
            {
              SnoozKeyPressedCtr = 6;
              spStatusAllowChange = 1;
            }
          }
        }
        else
        {
          SnoozKeyPressedCtr = 0;
        }

        if (spStatusAllowChange == 1)
        {
          // Display Paramter Change Page
          lcd.setCursor(0, 0);
          sprintf(tempChar, "Parm| I:E,TV,BPM,Psr");
          lcd.print(tempChar);

          lcd.setCursor(0, 1);
          sprintf(tempChar, "Unts|  --,mL,cnt,cmH");
          lcd.print(tempChar);

          lcd.setCursor(0, 2);
          sprintf(tempChar, "New|%1d:%1d, %03d, %02d, %02d", I_E_InpsFactor[sp.newI_E_Section - 1], I_E_ExpFactor[sp.newI_E_Section - 1], sp.newTV, sp.newBPM, (int)(sp.newOP * Pa2cmH2O));
          lcd.print(tempChar);

          lcd.setCursor(0, 3);
          sprintf(tempChar, "Svd|%1d:%1d, %03d, %02d, %02d", I_E_InpsFactor[sp.curI_E_Section - 1], I_E_ExpFactor[sp.curI_E_Section - 1], sp.curTV, sp.curBPM, (int)(sp.curOP * Pa2cmH2O));
          lcd.print(tempChar);

        }
        else
        {
          if ((millis() - milli_old) > 5000)
          {
            milli_old = millis();
            if (DisplayPageNum == 2)
            {
              DisplayPageNum = 3;
            }
            else if (DisplayPageNum == 3)
            {
              DisplayPageNum = 2;
            }
            else if (DisplayPageNum == 4)
            {
              DisplayPageNum = 2;
            }
          }

          if (SnoozKeyPressed == 1)
          {
            DisplayPageNum = 4;
            milli_old = millis();
          }

          switch (DisplayPageNum)
          {
            case 2:
              lcd.setCursor(0, 0);
              sprintf(tempChar, "** Set Parameters **");
              lcd.print(tempChar);

              lcd.setCursor(0, 1);
              if (VenMode == VOL_CONT_MODE)
                sprintf(tempChar, "TV: %03dml,PrL: %02dcmH", sp.curTV, (int)(sp.curOP * Pa2cmH2O));
              else
                sprintf(tempChar, "Mode - Spare");

              lcd.print(tempChar);

              lcd.setCursor(0, 2);
              sprintf(tempChar, "I:E - %1d:%1d, BPM - %2d", I_E_InpsFactor[sp.curI_E_Section - 1], I_E_ExpFactor[sp.curI_E_Section - 1], sp.curBPM);
              lcd.print(tempChar);

              lcd.setCursor(0, 3);
              if (SysON == 1)
                sprintf(tempChar1, "ON ");
              else
                sprintf(tempChar1, "OFF");

              if (ErrorNum > 0)
                sprintf(tempChar2, "Error # %1d", ErrorNum);
              else
                sprintf(tempChar2, "All Sensr OK !");

              sprintf(tempChar, "%s,%s", tempChar1, tempChar2);
              lcd.print(tempChar);

              break;
            case 3:
              lcd.setCursor(0, 0);
              sprintf(tempChar, "** Curr Parameter **");
              lcd.print(tempChar);

              lcd.setCursor(0, 1);
              sprintf(tempChar, "Press - %02d cmH2O    ", (int)(p_sensor.pressure_gauge_CM));
              lcd.print(tempChar);

              lcd.setCursor(0, 2);
              sprintf(tempChar, "Plt-%02d | PEEP-%02d cmH", (int)(PltPrsr), (int)(PEEP));
              lcd.print(tempChar);

              lcd.setCursor(0, 3);
              if (SysON == 1)
                sprintf(tempChar1, "ON");
              else
                sprintf(tempChar1, "OFF");

              if (ErrorNum > 0)
                sprintf(tempChar2, "Error # %1d", ErrorNum);
              else
                sprintf(tempChar2, "All Sensr OK !");

              sprintf(tempChar, "%s,%s", tempChar1, tempChar2);
              lcd.print(tempChar);

              break;
            case 4:
              lcd.setCursor(0, 0);
              sprintf(tempChar, "** Alarm(s) Types **");
              lcd.print(tempChar);

              lcd.setCursor(0, 1);
              sprintf(tempChar, "1:Pwr 2:PltPrs 3:TV");
              lcd.print(tempChar);

              lcd.setCursor(0, 2);
              sprintf(tempChar, "4:PEEP 5:AirwayPress");
              lcd.print(tempChar);

              lcd.setCursor(0, 3);
              sprintf(tempChar, "6:HomPos 7:StartSwON ");
              lcd.print(tempChar);

              break;
          }//switch
        }//else of spStatusAllowChange==1
      }//else of if ((SelfTestInPrg > 0) || (CalibInPrg > 0) )
    }//else of if (DevMode == 1)
  }//else of if (WarmUpFlag == 1)
}//function end

/*
   Used to wait until the debonce effect of button is being stable
*/
inline static int8_t debounce(int buttonPin, unsigned long &lastDebounceTime, int &lastButtonState, int buttonState)
{
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  return buttonState;
}
/*
  This is the main User Interface Control Functions.
     Read Button values.
     Read Keypad.
     Update LCD. etc.
     Check user input values to be within bounds.
     Output User Input Values Req Pressure, Volume and BPM
*/
void userInterface(void)
{
  static unsigned long lastDebounceTimeOkBtn = 0;
  static unsigned long lastDebounceTimeSnzBtn = 0;
  static int lastButtonStateOkBtn = LOW;
  static int lastButtonStateSnzBtn = LOW;

  float tempFloat;

#ifdef Keypad_4x3
  PressedKey = keypad.getKey();
#endif

  // Following assignment is modified because currently EEROM read write isnt enables so user input wont be effective otherwise
  reqExpirationRatioIndex = spStatus.curI_E_Section;
  reqBPM = spStatus.curBPM;
  reqVolume = spStatus.curTV;
  reqPressure = spStatus.curOP;


  OkButton = debounce(pin_Button_OK, lastDebounceTimeOkBtn, lastButtonStateOkBtn, OkButton);
  SnoozeButton = debounce(pin_Button_SNZ, lastDebounceTimeSnzBtn, lastButtonStateSnzBtn, SnoozeButton);

#ifdef Beeper
  if (SnoozeButton == 1)
  {
    alarm.action = SNOOZE_ALARM;
    ErrorNumber = 0;
  }
#endif

  if (spStatusAllowChange == 1) // This flag is allowd only when SnoozeBttn is pressed for 3 sec and Setting page is being displayed in LCD_Display
  {

    tempFloat = map(analogRead(pin_Knob_1), minPot, maxPot, 0, 10);
    if (tempFloat >= 0.0 && tempFloat < 2.0)
      spStatus.newI_E_Section = 1;
    else if (tempFloat >= 2.0 && tempFloat < 4.0)
      spStatus.newI_E_Section = 2;
    else if (tempFloat >= 4.0 && tempFloat < 6.0)
      spStatus.newI_E_Section = 3;
    else if (tempFloat >= 6.0 && tempFloat < 8.0)
      spStatus.newI_E_Section = 4;
    else
      spStatus.newI_E_Section = 5;

    spStatus.newI_E_Section = constrain(spStatus.newI_E_Section, 2, 4); //1:1, 1:2, 1:3

    spStatus.newBPM = map(analogRead(pin_Knob_2), minPot, maxPot, minBPM, maxBPM);
    spStatus.newTV = map(analogRead(pin_Knob_3), minPot, maxPot, minVolume, maxVolume);
    spStatus.newOP = map(analogRead(pin_Knob_4), minPot, maxPot, minPressure, maxPressure);


    if (OkButton == 1)
    {
      spStatus.curI_E_Section = spStatus.newI_E_Section;
      spStatus.curBPM = spStatus.newBPM;
      spStatus.curTV = spStatus.newTV;
      spStatus.curOP = spStatus.newOP;

      spStatusAllowChange = 0;
    }
  }

  if (digitalRead(pin_Switch_START) == HIGH)
    activateVentilatorOperation = 1;
  else
    activateVentilatorOperation = 0;

  //  if (digitalRead(pin_Switch_MODE == HIGH))     CVmode = PRESS_CONT_MODE; else activateVentilatorOperation = VOL_CONT_MODE;

  LCDDisplayCtr++;
  if (LCDDisplayCtr >= 5) // At 500msec
  {
    LCDDisplayCtr = 0;
    // LCD_Display(WarmUpFlag, WARM_UP_TIME, SnoozeButton, spStatus, DevModeDetectionInProg ,
    //             devMode, motorInUse, pressSnsrInUse,
    //             CVmode, VentilatorOperationON /* This is SysON (Breath In-Out Loop Running) */,
    //             ErrorNumber /* This is ErrorNum (0 means no error)*/,
    //             plateauPressure * Pa2cmH2O /* This is measured Plateau Pressure*/,
    //             PEEPressure * Pa2cmH2O /* This is measured PEEP*/,
    //             selfTestProg /* This is SelfTestInProgl*/,
    //             ST_COMPLETE /* This is CalibInProg*/,
    //             0 /* This is SpeedSF*/,
    //             0 /* This is StepSD*/);
#ifdef DEBUG
//    Serial.print("Home Pos Hit Position: "); Serial.println(homePosHitMotorPos);
//    Serial.print("OP2 Hit Position: "); Serial.println(OP2HitMotorPos);
#endif 
}
}