// SolderingStation2 
//
// ATmega328-controlled Soldering Station for Hakko T12 / JBC C210 / JBC C245 Tips
// Hybrid PID & Multi-Stage Bang-Bang Control Edition with Buck Converter Relay & Power Limit
//
// 2019 - 2026 by Stefan Wagner modified by Receh DIY
// License: http://creativecommons.org/licenses/by-sa/3.0/
//


// Libraries
#include "U8glib.h"
#include "EEPROM.h"
#include "avr/sleep.h"

// Firmware version
#define VERSION       "v3.1"

// Type of MOSFET
#define N_MOSFET

// Type of OLED Controller
#define SH1106

// Pins
#define SENSOR_PIN    A0
#define BUZZER_PIN     5
#define BUTTON_PIN     6
#define ROTARY_1_PIN   7
#define ROTARY_2_PIN   8
#define CONTROL_PIN    9
#define SWITCH_PIN    10
#define BUCK_12V_PIN  11
#define CH1_PIN        1
#define CH2_PIN        3
#define CH3_PIN        4

// Mode Cartridge Enum
#define MODE_T12      0
#define MODE_C210     1
#define MODE_C245     2

// Control Mode Enum
#define CTRL_PID      0
#define CTRL_DIRECT   1

// Temperature control defaults
#define TEMP_MIN      150
#define TEMP_MAX      450
#define TEMP_DEFAULT  320
#define TEMP_BOOST     50
#define TEMP_STEP       5

// ADC SETTINGS T12
#define T12_ADC200    216
#define T12_ADC280    308
#define T12_ADC360    390

// ADC SETTINGS C210
#define C210_ADC200   81
#define C210_ADC280   118
#define C210_ADC360   156

// ADC SETTINGS C245
#define C245_ADC200   217
#define C245_ADC280   304
#define C245_ADC360   391

#define TEMPCHP       30
#define TIPMAX        8
#define TIPNAMELENGTH 12

// Default timer values (0 = disabled)
#define TIME2OFF       5
#define TIMEOFBOOST   40

// Control values
#define TIME2SETTLE   1100
#define SMOOTHIE      0.35
#define BEEP_ENABLE   true
#define BODYFLIP      false
#define ECREVERSE     false
#define DENOISE_SAMPLES 16


// Tip detection thresholds
#define TIP_ERROR_TEMP    480.0
#define TIP_ADC_MAX       1020
#define TIP_ADC_MIN       2

// EEPROM identifier
#define EEPROM_IDENT  0xE798

// MOSFET control (Active-LOW: 0 = ON, 255 = OFF)
#define HEATER_ON     0
#define HEATER_OFF    255
#define HEATER_PWM    ((uint8_t)Output)

// User settings (stored in EEPROM)
uint16_t  DefaultTemp = TEMP_DEFAULT;
uint8_t   BoostTemp   = TEMP_BOOST;
uint8_t   time2off    = TIME2OFF;
uint8_t   timeOfBoost = TIMEOFBOOST;
bool      beepEnable  = BEEP_ENABLE;
bool      BodyFlip    = BODYFLIP;
bool      ECReverse   = ECREVERSE;
uint8_t   CMode       = MODE_T12;
uint8_t   ControlMode = CTRL_PID;
uint8_t   PowerLimit  = 100;
uint16_t  ChannelTemp[3] = {300, 340, 380};

// Calibration data
uint16_t  CalTemp[TIPMAX][4] = {
  {T12_ADC200,  T12_ADC280,  T12_ADC360,  TEMPCHP},
  {C210_ADC200, C210_ADC280, C210_ADC360, TEMPCHP},
  {C245_ADC200, C245_ADC280, C245_ADC360, TEMPCHP}
};
char      TipName[TIPMAX][TIPNAMELENGTH] = {"T12", "C210", "C245"};
uint8_t   CurrentTip   = 0;
uint8_t   NumberOfTips = 3;

// Menu items (ALL CAPS)
const char *SetupItems[]       = { "SETUP MENU", "C-MODE", "POWER LIMIT", "CTRL MODE", "TIP SETTINGS", "TEMP SETTINGS",
                                   "TIMER SETTINGS", "BUZZER", "SCREEN FLIP", 
                                   "ENC REVERSE", "RESET EEPROM", "DEBUG", "RETURN" };
const char *CtrlModeItems[]    = { "CTRL MODE", "PID", "DIRECT" };
const char *TipItems[]         = { "TIP:", "CHANGE TIP", "CALIBRATE TIP", "RETURN" };
const char *TempItems[]        = { "TEMP SETTINGS", "DEFAULT TEMP", "BOOST TEMP", "RETURN" };
const char *TimerItems[]       = { "TIMER SETTINGS", "AUTO OFF DELAY", "BOOST TIMER", "RETURN" };
const char *StoreItems[]       = { "STORE SETTINGS ?", "NO", "YES" };
const char *SureItems[]        = { "ARE YOU SURE ?", "NO", "YES" };
const char *BuzzerItems[]      = { "BUZZER", "DISABLE", "ENABLE" };
const char *FlipItems[]        = { "SCREEN FLIP", "DISABLE", "ENABLE" };
const char *ECReverseItems[]   = { "ENC REVERSE", "DISABLE", "ENABLE" };
const char *CModeItems[]       = { "C-MODE", "T12", "C210", "C245" };
const char *PowerLimitItems[]  = { "POWER LIMIT", "%" };
const char *DefaultTempItems[] = { "DEFAULT TEMP", "\xB0""C" };
const char *BoostTempItems[]   = { "BOOST TEMP", "\xB0""C" };
const char *SleepTimerItems[]  = { "AUTO OFF DELAY", "SECONDS" };
const char *BoostTimerItems[]  = { "BOOST TIMER", "SECONDS" };

// Setup menu enum
enum SetupMenuItem {
  SETUP_CMODE = 0,     
  SETUP_POWERLIMIT,
  SETUP_CTRLMODE,        
  SETUP_TIP,           
  SETUP_TEMP,         
  SETUP_TIMER,       
  SETUP_BUZZER,       
  SETUP_FLIP,         
  SETUP_ECREVERSE,    
  SETUP_RESET,         
  SETUP_DEBUG,         
  SETUP_RETURN
};

// Interrupt variables
volatile uint8_t  a0, b0, c0, d0;
volatile int      count, countMin, countMax, countStep;
volatile bool     handleMoved;
volatile bool     rotaryChanged = false;

// Temperature control & PID Variables
uint16_t  SetTemp, ShowTemp, gap;
double    Setpoint, RawTemp, CurrentTemp, ChipTemp;
uint8_t   Output = HEATER_OFF;
bool      tempLocked = false;

// PID Controller Variables
double    integralSum = 0.0;
double    lastTemp = 0.0;
uint32_t  lastPidTime = 0;

uint16_t  Vcc;

// State variables
bool      inOffMode   = false;
bool      inBoostMode = false;
bool      inCalibMode = false;
bool      isWorky     = true;
bool      beepIfWorky = true;
bool      TipIsPresent= true;

// Timing
uint32_t  sleepmillis;
uint32_t  boostmillis;
uint32_t  buttonmillis;
uint32_t  setTempShowMillis = 0;
uint8_t   goneMinutes;
uint8_t   goneSeconds;

// Channel button state machine
struct ChannelButtonState {
  uint32_t pressStart;
  bool     isPressed;
  bool     longPressHandled;
  bool     shortPressHandled;
};
ChannelButtonState chBtn[3] = {{0,false,false,false},{0,false,false,false},{0,false,false,false}};
const uint8_t channelPins[3] = {CH1_PIN, CH2_PIN, CH3_PIN};
const uint32_t LONG_PRESS_MS = 1500;

// OLED object
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);

// Forward declarations
void syncTipWithMode();
void resetEEPROMData();
void getEEPROM();
void updateEEPROM();
void updateChannelEEPROM(uint8_t channelIndex);
void SetFlip();
void MainScreen();
void SetupScreen();
void TipScreen();
void TempScreen();
void TimerScreen();
void ChangeTipScreen();
void CalibrationScreen();
void DebugScreen();
void CHANNELCheck();
void ROTARYCheck();
void SLEEPCheck();
void SENSORCheck();
void Thermostat();
void calculateTemp();
void beep();
void longBeep(uint16_t durationMs);
void setRotary(int rmin, int rmax, int rstep, int rvalue);
int getRotary();
uint16_t denoiseAnalog(byte port);
double getChipTemp();
uint16_t getVCC();
uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected);
uint16_t InputScreen(const char *Items[]);

void syncTipWithMode() {
  const char *targetName;
  if (CMode == MODE_T12)       targetName = "T12";
  else if (CMode == MODE_C210) targetName = "C210";
  else                         targetName = "C245";

  for (uint8_t i = 0; i < NumberOfTips; i++) {
    if (strcmp(TipName[i], targetName) == 0) {
      CurrentTip = i;
      break;
    }
  }
}

void resetEEPROMData() {
  DefaultTemp  = TEMP_DEFAULT;
  BoostTemp    = TEMP_BOOST;
  time2off     = TIME2OFF;
  timeOfBoost  = TIMEOFBOOST;
  beepEnable   = BEEP_ENABLE;
  BodyFlip     = BODYFLIP;
  ECReverse    = ECREVERSE;
  CMode        = MODE_T12;
  ControlMode  = CTRL_PID;
  PowerLimit   = 100;
  CurrentTip   = 0;
  NumberOfTips = 3;

  ChannelTemp[0] = 300;
  ChannelTemp[1] = 340;
  ChannelTemp[2] = 380;

  memset(TipName, 0, sizeof(TipName));
  strcpy(TipName[0], "T12");
  CalTemp[0][0] = T12_ADC200;
  CalTemp[0][1] = T12_ADC280;
  CalTemp[0][2] = T12_ADC360;
  CalTemp[0][3] = TEMPCHP;

  strcpy(TipName[1], "C210");
  CalTemp[1][0] = C210_ADC200;
  CalTemp[1][1] = C210_ADC280;
  CalTemp[1][2] = C210_ADC360;
  CalTemp[1][3] = TEMPCHP;

  strcpy(TipName[2], "C245");
  CalTemp[2][0] = C245_ADC200;
  CalTemp[2][1] = C245_ADC280;
  CalTemp[2][2] = C245_ADC360;
  CalTemp[2][3] = TEMPCHP;

  digitalWrite(BUCK_12V_PIN, LOW);

  EEPROM.update(0, EEPROM_IDENT >> 8);
  EEPROM.update(1, EEPROM_IDENT & 0xFF);
  updateEEPROM();
}

void setup() { 
  pinMode(SENSOR_PIN,   INPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(CONTROL_PIN,  OUTPUT);
  pinMode(ROTARY_1_PIN, INPUT_PULLUP);
  pinMode(ROTARY_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN,   INPUT_PULLUP);
  pinMode(SWITCH_PIN,   INPUT_PULLUP);
  pinMode(BUCK_12V_PIN, OUTPUT);
  pinMode(CH1_PIN,      INPUT_PULLUP);
  pinMode(CH2_PIN,      INPUT_PULLUP);
  pinMode(CH3_PIN,      INPUT_PULLUP);
  
  analogWrite(CONTROL_PIN, HEATER_OFF); 
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BUCK_12V_PIN, LOW);

  getEEPROM();

  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);  
  ADCSRA |= bit (ADIE);                                
  interrupts ();                                      

  PCMSK0 = bit (PCINT0);                
  PCICR  = bit (PCIE0);                 
  PCIFR  = bit (PCIF0);                 

  u8g.setColorIndex(1);

  SetFlip();
  Vcc = getVCC();

  for (uint8_t i = 0; i < 10; i++) {
    denoiseAnalog(SENSOR_PIN);
    delay(2);
  }

  SetTemp  = DefaultTemp;
  RawTemp  = denoiseAnalog(SENSOR_PIN); 
  ChipTemp = getChipTemp();
  calculateTemp();

  Setpoint = DefaultTemp;
  ShowTemp = (uint16_t)CurrentTemp;
  lastTemp = CurrentTemp;
  tempLocked = false;

  analogWrite(CONTROL_PIN, HEATER_OFF);

  a0 = PINB & 1; b0 = PIND>>7 & 1;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
  
  sleepmillis = millis();

  d0 = digitalRead(SWITCH_PIN);
  if (d0 == LOW) {
    inOffMode = true;
    Setpoint  = 0;
    Output    = HEATER_OFF;
    analogWrite(CONTROL_PIN, HEATER_OFF);
  }

  beep(); beep();
  setTempShowMillis = millis() - 2000;
  MainScreen();
}

void loop() {
  CHANNELCheck();     
  ROTARYCheck();      
  SLEEPCheck();       
  SENSORCheck();      
  Thermostat();       
  MainScreen();       
}

// Non-blocking channel button check
void CHANNELCheck() {
  uint32_t now = millis();
  
  for (uint8_t i = 0; i < 3; i++) {
    bool pressed = (digitalRead(channelPins[i]) == LOW);
    
    if (pressed && !chBtn[i].isPressed) {
      chBtn[i].isPressed = true;
      chBtn[i].pressStart = now;
      chBtn[i].longPressHandled = false;
      chBtn[i].shortPressHandled = false;
      beep();
    }
    else if (pressed && chBtn[i].isPressed) {
      if (!chBtn[i].longPressHandled && (now - chBtn[i].pressStart) >= LONG_PRESS_MS) {
        ChannelTemp[i] = SetTemp;
        updateChannelEEPROM(i);
        beep(); 
        chBtn[i].longPressHandled = true;
      }
    }
    else if (!pressed && chBtn[i].isPressed) {
      chBtn[i].isPressed = false;
      
      if (!chBtn[i].longPressHandled && !chBtn[i].shortPressHandled) {
        SetTemp = ChannelTemp[i];
        setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
        beepIfWorky = true;
        chBtn[i].shortPressHandled = true;
        tempLocked = false;
        setTempShowMillis = millis();
      }
    }
  }
}

void ROTARYCheck() {
  if (rotaryChanged || getRotary() != SetTemp) {
    SetTemp = getRotary();
    rotaryChanged = false;
    tempLocked = false;
    setTempShowMillis = millis();
  }
  
  uint8_t c = digitalRead(BUTTON_PIN);
  if ( !c && c0 ) {
    beep();
    buttonmillis = millis();
    while( (!digitalRead(BUTTON_PIN)) && ((millis() - buttonmillis) < 500) );
    if ((millis() - buttonmillis) >= 500) SetupScreen();
    else {
      inBoostMode = !inBoostMode;
      tempLocked = false;
      if (inBoostMode) boostmillis = millis();
    }
  }
  c0 = c;

  if (inBoostMode && timeOfBoost) {
    goneSeconds = (millis() - boostmillis) / 1000;
    if (goneSeconds >= timeOfBoost) {
      inBoostMode = false;              
      beep();                           
      beepIfWorky = true;
      tempLocked = false;
    }
  }
}

void SLEEPCheck() {
  if (handleMoved) {                    
    if (inOffMode) {                    
      beep();                           
      beepIfWorky = true;               
    }
    handleMoved = false;                
    inOffMode   = false;                
    sleepmillis = millis();             
  }

  if (d0 == LOW) {
    goneSeconds = (millis() - sleepmillis) / 1000; 
    
    if ( (!inOffMode) && (time2off > 0) && (goneSeconds >= time2off) ) {
      inOffMode = true; 
      longBeep(300);
    }
  } else {
    sleepmillis = millis(); 
    inOffMode = false;
  }
}

void SENSORCheck() {
  analogWrite(CONTROL_PIN, HEATER_OFF);
  delayMicroseconds(TIME2SETTLE);
  
  uint16_t rawPinAdc = analogRead(SENSOR_PIN);

  double temp = denoiseAnalog(SENSOR_PIN);
  uint8_t d = digitalRead(SWITCH_PIN);
  
  if (d == HIGH) { 
    handleMoved = true; 
  }
  d0 = d;
  
  //RawTemp += (temp - RawTemp) * SMOOTHIE;
  double currentSmoothie = (CMode == MODE_C245) ? SMOOTHIE : 0.15;
  RawTemp += (temp - RawTemp) * currentSmoothie;     
  calculateTemp();                          

  // Display Lock Hysteresis
  if (!tempLocked) {
    if (fabs(CurrentTemp - Setpoint) <= 3.0) {
      tempLocked = true;
      ShowTemp = (uint16_t)Setpoint;
    } else {
      ShowTemp = (uint16_t)CurrentTemp;
    }
  } else {
    if ((Setpoint - CurrentTemp) > 10.0 || (CurrentTemp - Setpoint) > 5.0) {
      tempLocked = false;
      ShowTemp = (uint16_t)CurrentTemp;
    } else {
      ShowTemp = (uint16_t)Setpoint;
    }
  }

  uint16_t adcThreshold500 = (CMode == MODE_C210) ? 350 : 650;

  bool instantFault = (rawPinAdc >= 950) || (rawPinAdc > adcThreshold500);

  static uint32_t faultTimer = 0;
  static bool isTimingFault = false;
  bool tipError = false;

  if (instantFault) {
    if (!isTimingFault) {
      isTimingFault = true;
      faultTimer = millis();
    } else {
      if (millis() - faultTimer >= 3000) {
        tipError = true;
      }
    }
  } else {
    isTimingFault = false;
    tipError = false;
  }

  if (tipError) {
    if (TipIsPresent) {
      TipIsPresent = false;
      isWorky     = false;
      beepIfWorky = false;
      analogWrite(CONTROL_PIN, HEATER_OFF);
    }
  }
  else if (!TipIsPresent) {
    if (rawPinAdc >= 5 && rawPinAdc <750) {
      TipIsPresent = true;
      isWorky      = false;
      beepIfWorky  = false;
      beep();

      RawTemp = denoiseAnalog(SENSOR_PIN);
      delay(50);
      RawTemp = denoiseAnalog(SENSOR_PIN);
      calculateTemp();

      CMode = MenuScreen(CModeItems, sizeof(CModeItems), CMode);

      if (CMode == MODE_C210) {
        digitalWrite(BUCK_12V_PIN, HIGH);
      } else {
        digitalWrite(BUCK_12V_PIN, LOW);
      }

      syncTipWithMode();
      updateEEPROM();

      handleMoved = true;
      c0 = LOW;
      RawTemp = denoiseAnalog(SENSOR_PIN);
      calculateTemp();
      setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
      tempLocked = false;
    }
  }

  if (TipIsPresent) {
    gap = (uint16_t)fabs((double)SetTemp - CurrentTemp);
    if (gap < 8) {
      if (!isWorky && beepIfWorky) beep();
      isWorky = true;
      beepIfWorky = false;
    }
    else isWorky = false;
  }
  else {
    isWorky     = false;
    beepIfWorky = false;
  }
}

// Linear & Anti-Stuck Temperature Calculation
void calculateTemp() {
  uint16_t adc200 = CalTemp[CurrentTip][0];
  uint16_t adc280 = CalTemp[CurrentTip][1];
  uint16_t adc360 = CalTemp[CurrentTip][2];

  if (adc200 < 50) {
    if (CMode == MODE_T12)       adc200 = T12_ADC200;
    else if (CMode == MODE_C210) adc200 = C210_ADC200;
    else                         adc200 = C245_ADC200;
  }
  if (adc280 <= adc200) adc280 = adc200 + 90;
  if (adc360 <= adc280) adc360 = adc280 + 80;

  if (RawTemp <= 0) {
    CurrentTemp = 21.0;
  } else if (RawTemp < adc200) {
    CurrentTemp = 21.0 + ((double)RawTemp * (200.0 - 21.0) / (double)adc200);
  } else if (RawTemp < adc280) {
    CurrentTemp = 200.0 + ((double)(RawTemp - adc200) * (280.0 - 200.0) / (double)(adc280 - adc200));
  } else if (RawTemp < adc360) {
    CurrentTemp = 280.0 + ((double)(RawTemp - adc280) * (360.0 - 280.0) / (double)(adc360 - adc280));
  } else {
    uint16_t adcStep = adc360 - adc280;
    CurrentTemp = 360.0 + ((double)(RawTemp - adc360) * 80.0 / (double)adcStep);
  }

  if (CurrentTemp < 20.0) CurrentTemp = 20.0;
}


void Thermostat() {
  if      (inOffMode)   Setpoint = 0;
  else if (inBoostMode) Setpoint = SetTemp + BoostTemp;
  else                  Setpoint = SetTemp; 

  if (inOffMode || Setpoint == 0) {
    Output = HEATER_OFF;
    integralSum = 0;
    analogWrite(CONTROL_PIN, HEATER_OFF);
    return;
  }

  uint8_t maxPwmDuty = (uint16_t)(PowerLimit * 255) / 100;
  double error = Setpoint - CurrentTemp;

  // MODE DIRECT (MULTI-STAGE BANG-BANG v2.9)
  if (ControlMode == CTRL_DIRECT) {
    if (CurrentTemp >= Setpoint) {
      Output = HEATER_OFF;
    } 
    else {
      if (CMode == MODE_C210) {
        if (CurrentTemp >= (Setpoint - 20.0)) {
          uint8_t nearPwmDuty = ((uint16_t)maxPwmDuty * 11) / 100;
          Output = 255 - nearPwmDuty;
        } 
        else {
          Output = 255 - maxPwmDuty;
        }
      } 
      else {
        if (CurrentTemp >= (Setpoint - 10.0)) {
          uint8_t nearPwmDuty = (maxPwmDuty / 2);
          Output = 255 - nearPwmDuty;
        } 
        else {
          Output = 255 - maxPwmDuty;
        }
      }
    }
    analogWrite(CONTROL_PIN, HEATER_PWM);
    return;
  }

  // MODE HYBRID PID (TUNING PER TIP)
  double curKp;
  double curKi;
  double curKd;
  double initIntegral;
  double maxIntegral;
  double pidWindow; 

  if (CMode == MODE_C245) {
    curKp        = 9.5; 
    curKi        = 0.45;
    curKd        = 0.38;
    initIntegral = 15.0;   
    maxIntegral  = 140.0; 
    pidWindow    = 45.0;
  } 
  else if (CMode == MODE_C210) {
    curKp        = 4.2;
    curKi        = 0.25;
    curKd        = 0.08;
    initIntegral = 5.0;
    maxIntegral  = 80.0;
    pidWindow    = 15.0;
  } 
  else {
    curKp        = 9.5; 
    curKi        = 0.58;
    curKd        = 0.38;
    initIntegral = 15.0;   
    maxIntegral  = 140.0; 
    pidWindow    = 30.0;
  }

  if (error > pidWindow) {
    Output = 255 - maxPwmDuty;
    integralSum = initIntegral;
  } 
  else if (error < -1.5) {
    Output = HEATER_OFF;
    integralSum = 0.0; 
  } 
  else {
    uint32_t now = millis();
    double dt = (now - lastPidTime) / 1000.0;
    if (dt <= 0 || dt > 1.0) dt = 0.1;

    if (Setpoint > 400.0) {
      curKp *= 0.7;
      curKd *= 0.5;
    }

    double pTerm = curKp * error;

    integralSum += (error * dt);
    integralSum = constrain(integralSum, -30.0, maxIntegral);
    double iTerm = curKi * integralSum;

    double dTerm = -curKd * ((CurrentTemp - lastTemp) / dt);

    double pidVal = pTerm + iTerm + dTerm;
    uint8_t pwmDuty = (uint8_t)constrain(pidVal, 0.0, (double)maxPwmDuty);

    Output = 255 - pwmDuty;

    lastTemp = CurrentTemp;
    lastPidTime = now;
  }

  analogWrite(CONTROL_PIN, HEATER_PWM);
}

void beep(){
  if (beepEnable) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(50);
      digitalWrite(BUZZER_PIN, LOW);
  }
}

void longBeep(uint16_t durationMs) {
  if (beepEnable) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(durationMs);
      digitalWrite(BUZZER_PIN, LOW);
  }
}

// Absolute direct count -- no bit-shift division
void setRotary(int rmin, int rmax, int rstep, int rvalue) {
  countMin  = rmin;
  countMax  = rmax;
  countStep = ECREVERSE ? -rstep : rstep;
  count     = rvalue;  
  rotaryChanged = false;
}

int getRotary() {
  return count;
}

void getEEPROM() {
  uint16_t identifier = (EEPROM.read(0) << 8) | EEPROM.read(1);
  if (identifier == EEPROM_IDENT) {
    DefaultTemp = (EEPROM.read(2) << 8) | EEPROM.read(3);
    BoostTemp   =  EEPROM.read(4);
    time2off    =  EEPROM.read(5);
    if (time2off > 15) time2off = TIME2OFF;
    timeOfBoost =  EEPROM.read(6);
    beepEnable  =  EEPROM.read(7);
    BodyFlip    =  EEPROM.read(8);
    ECReverse   =  EEPROM.read(9);
    CurrentTip  =  EEPROM.read(10);
    NumberOfTips = EEPROM.read(11);
    CMode       =  EEPROM.read(12);
    if (CMode > 2) CMode = MODE_T12;
    PowerLimit  =  EEPROM.read(13);
    if (PowerLimit < 30 || PowerLimit > 100) PowerLimit = 100;

    if (CMode == MODE_C210) {
      digitalWrite(BUCK_12V_PIN, HIGH);
    } else {
      digitalWrite(BUCK_12V_PIN, LOW);
    }

    ChannelTemp[0] = (EEPROM.read(14) << 8) | EEPROM.read(15);
    ChannelTemp[1] = (EEPROM.read(16) << 8) | EEPROM.read(17);
    ChannelTemp[2] = (EEPROM.read(18) << 8) | EEPROM.read(19);

    if (ChannelTemp[0] == 0xFFFF || ChannelTemp[0] < TEMP_MIN) ChannelTemp[0] = 300;
    if (ChannelTemp[1] == 0xFFFF || ChannelTemp[1] < TEMP_MIN) ChannelTemp[1] = 340;
    if (ChannelTemp[2] == 0xFFFF || ChannelTemp[2] < TEMP_MIN) ChannelTemp[2] = 380;

    ControlMode = EEPROM.read(20);
    if (ControlMode > 1) ControlMode = CTRL_PID;

    memset(TipName, 0, sizeof(TipName));
    uint8_t i, j;
    uint16_t counter = 21;
    for (i = 0; i < NumberOfTips; i++) {
      for (j = 0; j < TIPNAMELENGTH; j++) {
        TipName[i][j] = EEPROM.read(counter++);
      }
      TipName[i][TIPNAMELENGTH - 1] = '\0';
      for (j = 0; j < 4; j++) {
        CalTemp[i][j]  = EEPROM.read(counter++) << 8;
        CalTemp[i][j] |= EEPROM.read(counter++);
      }
    }

    syncTipWithMode();
  }
  else {
    resetEEPROMData();
  }
}

void updateEEPROM() {
  EEPROM.update( 2, DefaultTemp >> 8);
  EEPROM.update( 3, DefaultTemp & 0xFF);
  EEPROM.update( 4, BoostTemp);
  EEPROM.update( 5, time2off);
  EEPROM.update( 6, timeOfBoost);
  EEPROM.update( 7, beepEnable);
  EEPROM.update( 8, BodyFlip);
  EEPROM.update( 9, ECReverse);
  EEPROM.update(10, CurrentTip);
  EEPROM.update(11, NumberOfTips);
  EEPROM.update(12, CMode);
  EEPROM.update(13, PowerLimit);

  EEPROM.update(14, ChannelTemp[0] >> 8); EEPROM.update(15, ChannelTemp[0] & 0xFF);
  EEPROM.update(16, ChannelTemp[1] >> 8); EEPROM.update(17, ChannelTemp[1] & 0xFF);
  EEPROM.update(18, ChannelTemp[2] >> 8); EEPROM.update(19, ChannelTemp[2] & 0xFF);

  EEPROM.update(20, ControlMode);

  uint8_t i, j;
  uint16_t counter = 21;
  for (i = 0; i < NumberOfTips; i++) {
    for (j = 0; j < TIPNAMELENGTH; j++) EEPROM.update(counter++, TipName[i][j]);
    for (j = 0; j < 4; j++) {
      EEPROM.update(counter++, CalTemp[i][j] >> 8);
      EEPROM.update(counter++, CalTemp[i][j] & 0xFF);
    }
  }
}

void updateChannelEEPROM(uint8_t channelIndex) {
  uint16_t addr = 14 + (channelIndex * 2);
  EEPROM.update(addr,     ChannelTemp[channelIndex] >> 8);
  EEPROM.update(addr + 1, ChannelTemp[channelIndex] & 0xFF);
}

void SetFlip() {
  if (BodyFlip) u8g.setRot180();
  else          u8g.undoRotation();
}

// MODERN CLEAN UI
void MainScreen() {
  u8g.firstPage();
  do {
    bool isSettingTemp = (millis() - setTempShowMillis < 1500);

    u8g.setFont(u8g_font_6x12);
    u8g.setFontPosTop();

    const char *statusMsg;
    if (isSettingTemp)                        statusMsg = "SET !";
    else if (!TipIsPresent || ShowTemp > 500) statusMsg = "ERROR!";
    else if (inOffMode)                       statusMsg = "OFF";
    else if (inBoostMode)                     statusMsg = "BOOST!";
    else if (isWorky)                         statusMsg = "READY!";
    else if (Output < 255)                    statusMsg = "HEAT";
    else                                      statusMsg = "HOLD";

    uint8_t strWidth = u8g.getStrPixelWidth(statusMsg);
    uint8_t statusX = 86 + ((41 - strWidth) / 2);
    u8g.setPrintPos(statusX, 0);
    u8g.print(statusMsg);

    u8g.drawFrame(86, 13, 41, 11);

    if (!inOffMode && TipIsPresent && Output < 255) {
      uint8_t currentDuty = 255 - Output;
      uint8_t fillWidth = ((uint16_t)currentDuty * 37) / 255;
      if (fillWidth > 0) {
        u8g.drawBox(88, 15, fillWidth, 7);
      }
    }

    const char *modes[] = {"T12", "C210", "C245"};
    const uint8_t modesVal[] = {MODE_T12, MODE_C210, MODE_C245};
    const uint8_t textOffsetX[] = {97, 94, 94}; 

    for (uint8_t i = 0; i < 3; i++) {
      uint8_t yPos = 26 + (i * 13);
      bool isCurMode = (CMode == modesVal[i]);

      if (isCurMode) {
        u8g.drawRBox(86, yPos, 41, 11, 2);
        u8g.setColorIndex(0);
      } else {
        u8g.drawRFrame(86, yPos, 41, 11, 2);
        u8g.setColorIndex(1);
      }

      u8g.setPrintPos(textOffsetX[i], yPos);
      u8g.print(modes[i]);
      u8g.setColorIndex(1);
    }

    u8g.setFont(u8g_font_fub35n);
    u8g.setFontPosTop();
    u8g.setPrintPos(0, 0);

    if (isSettingTemp) {
      u8g.print(SetTemp);
    } else if (!TipIsPresent || ShowTemp > 500) {
      u8g.print(F("000"));
    } else if (inOffMode) {
      u8g.print(F("---"));
    } else {
      u8g.print(ShowTemp);
    }

    u8g.setFont(u8g_font_6x12);
    u8g.setFontPosBaseline();
    
    const uint8_t chX[3] = {1, 28, 55};
    char chLabel[4] = "CH1";

    for (uint8_t i = 0; i < 3; i++) {
      bool isSelected = (SetTemp == ChannelTemp[i]);
      chLabel[2] = '1' + i;

      if (isSelected) {
        u8g.drawRBox(chX[i], 48, 25, 15, 2);
        u8g.setColorIndex(0);
      } else {
        u8g.drawRFrame(chX[i], 48, 25, 15, 2);
        u8g.setColorIndex(1);
      }

      uint8_t textW = u8g.getStrPixelWidth(chLabel);
      uint8_t textX = chX[i] + ((25 - textW) / 2);

      u8g.drawStr(textX, 59, chLabel);
      u8g.setColorIndex(1);
    }

  } while(u8g.nextPage());
}

// MENU & SUB-MENU SCREENS (ALL CAPS & u8g_font_7x14)
void SetupScreen() {
  analogWrite(CONTROL_PIN, HEATER_OFF);     
  beep();
  uint16_t SaveSetTemp = SetTemp;
  uint8_t selection = 0;
  bool repeat = true;
  
  while (repeat) {
    selection = MenuScreen(SetupItems, sizeof(SetupItems), selection);
    switch (selection) {
      case SETUP_CTRLMODE:
        ControlMode = MenuScreen(CtrlModeItems, sizeof(CtrlModeItems), ControlMode);
        break;
      case SETUP_CMODE:
        CMode = MenuScreen(CModeItems, sizeof(CModeItems), CMode); 
        if (CMode == MODE_C210) {
          digitalWrite(BUCK_12V_PIN, HIGH);
        } else {
          digitalWrite(BUCK_12V_PIN, LOW);
        }
        syncTipWithMode();
        break;
      case SETUP_POWERLIMIT:
        setRotary(30, 100, 5, PowerLimit);
        PowerLimit = InputScreen(PowerLimitItems);
        break;
      case SETUP_TIP:
        TipScreen(); repeat = false; 
        break;
      case SETUP_TEMP:
        TempScreen(); 
        break;
      case SETUP_TIMER:
        TimerScreen(); 
        break;
      case SETUP_BUZZER:
        beepEnable = MenuScreen(BuzzerItems, sizeof(BuzzerItems), beepEnable); 
        break;
      case SETUP_FLIP:
        BodyFlip = MenuScreen(FlipItems, sizeof(FlipItems), BodyFlip); 
        SetFlip(); 
        break;
      case SETUP_ECREVERSE:
        ECReverse = MenuScreen(ECReverseItems, sizeof(ECReverseItems), ECReverse); 
        break;
      case SETUP_RESET:
        if (MenuScreen(SureItems, sizeof(SureItems), 0)) {
          resetEEPROMData();
          repeat = false;
        }
        break;
      case SETUP_DEBUG:
        DebugScreen(); 
        break;
      case SETUP_RETURN:
      default:
        repeat = false; 
        break;
    }
  }  
  updateEEPROM();
  handleMoved = true;
  SetTemp = SaveSetTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
  tempLocked = false;
}

void TipScreen() {
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) {
    selection = MenuScreen(TipItems, sizeof(TipItems), selection);
    switch (selection) {
      case 0:  ChangeTipScreen();   break;
      case 1:  CalibrationScreen(); break;
      default: repeat = false;      break;
    }
  }
}

void TempScreen() {
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) {
    selection = MenuScreen(TempItems, sizeof(TempItems), selection);
    switch (selection) {
      case 0:  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
                DefaultTemp = InputScreen(DefaultTempItems); break;
      case 1:  setRotary(10, 100, TEMP_STEP, BoostTemp);
                BoostTemp = InputScreen(BoostTempItems); break;
      default: repeat = false; break;
    }
  }
}

void TimerScreen() {
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) {
    selection = MenuScreen(TimerItems, sizeof(TimerItems), selection);
    switch (selection) {
      case 0:  setRotary(0, 15, 1, time2off); 
                time2off = InputScreen(SleepTimerItems); break;
      case 1:  setRotary(0, 180, 10, timeOfBoost);
                timeOfBoost = InputScreen(BoostTimerItems); break;
      default: repeat = false; break;
    }
  }
}

uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected) {
  bool isTipScreen = (strcmp(Items[0], "TIP:") == 0);
  uint8_t lastselected = selected;
  int8_t  arrow = 0;
  if (selected) arrow = 1;
  numberOfItems >>= 1;
  setRotary(0, (numberOfItems - 2) * 2, 1, selected * 2);
  bool    lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    selected = getRotary() / 2;
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x14);
        u8g.setFontPosTop();
        u8g.drawStr( 0, 0,  Items[0]);
        if (isTipScreen) u8g.drawStr( 42, 0,  TipName[CurrentTip]);
        u8g.drawStr( 0, 16 * (arrow + 1), ">");
        for (uint8_t i=0; i<3; i++) {
          uint8_t drawnumber = selected + i + 1 - arrow;
          if (drawnumber < numberOfItems)
            u8g.drawStr( 14, 16 * (i + 1), Items[selected + i + 1 - arrow]);
        }
      } while(u8g.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {delay(10); lastbutton = false;}
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return selected;
}

uint16_t InputScreen(const char *Items[]) {
  uint16_t  value;
  bool      lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    value = getRotary();
    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x14);
        u8g.setFontPosTop();
        u8g.drawStr( 0, 0,  Items[0]);
        u8g.setPrintPos(0, 32); u8g.print(">"); u8g.setPrintPos(14, 32);        
        if (value == 0)  u8g.print(F("DEACTIVATED"));
        else            {u8g.print(value);u8g.print(" ");u8g.print(Items[1]);}
      } while(u8g.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {delay(10); lastbutton = false;}
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return value;
}

void DebugScreen() {
  bool lastbutton = (!digitalRead(BUTTON_PIN));
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);

  do {
    SLEEPCheck();
    CHANNELCheck();
    SetTemp = getRotary();

    SENSORCheck();
    Thermostat();

    uint8_t pwrPercent = (uint16_t)((255 - Output) * 100) / 255;

    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x14);
        u8g.setFontPosTop();
        
        u8g.setPrintPos(0, 0);  u8g.print(F("ADC: ")); u8g.print(uint16_t(RawTemp));
        u8g.setPrintPos(0, 16); u8g.print(F("ACT: ")); u8g.print((uint16_t)CurrentTemp); u8g.print(F("\xB0""C"));
        u8g.setPrintPos(0, 32); u8g.print(F("SET: ")); 
        if (inOffMode) u8g.print(F("OFF"));
        else          { u8g.print(Setpoint, 0); u8g.print(F("\xB0""C")); }
        
        u8g.setPrintPos(0, 48); u8g.print(F("PWR: ")); u8g.print(pwrPercent); u8g.print(F("%"));
      } while(u8g.nextPage());

    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10); 
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
}

void ChangeTipScreen() {
  uint8_t selected = CurrentTip;
  uint8_t lastselected = selected;
  int8_t  arrow = 0;
  if (selected) arrow = 1;
  setRotary(0, NumberOfTips - 1, 1, selected);
  bool    lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x14);
        u8g.setFontPosTop();
        u8g.drawStr( 0, 0,  F("SELECT TIP"));
        u8g.drawStr( 0, 16 * (arrow + 1), ">");
        for (uint8_t i=0; i<3; i++) {
          uint8_t drawnumber = selected + i - arrow;
          if (drawnumber < NumberOfTips)
            u8g.drawStr( 14, 16 * (i + 1), TipName[selected + i - arrow]);
        }
      } while(u8g.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {delay(10); lastbutton = false;}
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  CurrentTip = selected;
}

void CalibrationScreen() {
  uint16_t CalTempNew[4]; 
  const uint16_t targetTemps[3] = {200, 280, 360};
  
  for (uint8_t CalStep = 0; CalStep < 3; CalStep++) {
    SetTemp = targetTemps[CalStep];
    setRotary(100, 500, 1, targetTemps[CalStep]); 
    beepIfWorky = true;
    bool lastbutton = (!digitalRead(BUTTON_PIN));
    bool readyToAdjust = false; 

    do {
      SENSORCheck();      
      Thermostat();       
      
      if (isWorky) {
        readyToAdjust = true;
      }

      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x14);
        u8g.setFontPosTop();
        u8g.drawStr(0, 0, F("CALIBRATION"));
        u8g.setPrintPos(0, 16); 
        u8g.print(F("STEP: ")); u8g.print(CalStep + 1); u8g.print(F(" (")); u8g.print(targetTemps[CalStep]); u8g.print(F("C)"));
        
        if (readyToAdjust) {
          u8g.setPrintPos(0, 32); u8g.print(F("MEASURED: ")); u8g.print(getRotary());
          u8g.setPrintPos(0, 48); u8g.print(F("PRESS TO SAVE"));
        } else {
          u8g.setPrintPos(0, 32); u8g.print(F("ADC: ")); u8g.print(uint16_t(RawTemp));
          u8g.setPrintPos(0, 48); u8g.print(F("HEATING UP..."));
        }
      } while(u8g.nextPage());

      if (lastbutton && digitalRead(BUTTON_PIN)) {
        delay(10); 
        lastbutton = false;
      }

    } while (digitalRead(BUTTON_PIN) || lastbutton);

    uint16_t measuredTemp = getRotary();
    if (measuredTemp == 0) measuredTemp = targetTemps[CalStep];
    CalTempNew[CalStep] = (uint16_t)(((uint32_t)RawTemp * (uint32_t)targetTemps[CalStep]) / (uint32_t)measuredTemp);
    
    beep(); delay(100);
  }

  analogWrite(CONTROL_PIN, HEATER_OFF);       
  delayMicroseconds(TIME2SETTLE);             
  CalTempNew[3] = getChipTemp();              
  
  if ((CalTempNew[0] + 10 < CalTempNew[1]) && (CalTempNew[1] + 10 < CalTempNew[2])) {
    if (MenuScreen(StoreItems, sizeof(StoreItems), 0)) {
      for (uint8_t i = 0; i < 4; i++) CalTemp[CurrentTip][i] = CalTempNew[i];
      updateEEPROM();
    }
  }
}

uint16_t denoiseAnalog (byte port) {
  uint16_t result = 0;
  ADCSRA |= bit (ADEN) | bit (ADIF);    
  if (port >= A0) port -= A0;           
  ADMUX = (0x0F & port) | bit(REFS0);   
  set_sleep_mode (SLEEP_MODE_ADC);      
  
  for (uint8_t i = 0; i < DENOISE_SAMPLES; i++) {        
    sleep_mode();                       
    while (bitRead(ADCSRA, ADSC));      
    result += ADC;                      
  }
  bitClear (ADCSRA, ADEN);              
  return (result / DENOISE_SAMPLES);   
}

double getChipTemp() {
  uint16_t result = 0;
  ADCSRA |= bit (ADEN) | bit (ADIF);    
  ADMUX = bit (REFS1) | bit (REFS0) | bit (MUX3); 
  delay(20);                            
  set_sleep_mode (SLEEP_MODE_ADC);      
  for (uint8_t i=0; i<32; i++) {        
    sleep_mode();                       
    while (bitRead(ADCSRA, ADSC));      
    result += ADC;                      
  }
  bitClear (ADCSRA, ADEN);              
  result >>= 2;                         
  return ((result - 2594) / 9.76);      
}

uint16_t getVCC() {
  uint16_t result = 0;
  ADCSRA |= bit (ADEN) | bit (ADIF);    
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  delay(1);                             
  set_sleep_mode (SLEEP_MODE_ADC);      
  for (uint8_t i=0; i<16; i++) {        
    sleep_mode();                       
    while (bitRead(ADCSRA, ADSC));      
    result += ADC;                      
  }
  bitClear (ADCSRA, ADEN);              
  result >>= 4;                         
  return (1125300L / result);           
}

EMPTY_INTERRUPT (ADC_vect);             

ISR (PCINT0_vect) {
  uint8_t a = PINB & 1;        // Pin 8 (PB0)
  uint8_t b = (PIND >> 7) & 1; // Pin 7 (PD7)

  if (a != a0) {
    a0 = a;
    if (a == LOW) {
      if (b != a) {
        count = constrain(count - countStep, countMin, countMax);
      } else {
        count = constrain(count + countStep, countMin, countMax);
      }
      rotaryChanged = true;
    }
  }
}
