#include <IRremote.h>
#include <IRremoteInt.h>
#include <WProgram.h>
#include <Wire.h>
#include <NewSoftSerial.h>
#include <OneWire.h>
#include <FormatDouble.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>

/**
* Copyright 2011, Roger Reed
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 
* 02110-1301, USA.
*/

// whether to print debug messages to Serial
const boolean SERIAL_DEBUG = false;

//////////////////////////////
// Control Constants
//////////////////////////////

// light constants
const int DAY1_ON_HOUR = 8,
  DAY1_ON_MINUTE = 30,
  DAY1_OFF_HOUR = 20,
  DAY1_OFF_MINUTE = 30,
  DAY2_ON_HOUR = 9,
  DAY2_ON_MINUTE = 30,
  DAY2_OFF_HOUR = 21,
  DAY2_OFF_MINUTE = 0,
  SUMP_ON_HOUR = 20, 
  SUMP_ON_MINUTE = 0,
  SUMP_OFF_HOUR = 8,
  SUMP_OFF_MINUTE = 30;

// feed constants
const int FEED_MORNING_HOUR = 9,
  FEED_MORNING_MINUTE = 15,
  FEED_MID_DAY_HOUR = 14,
  FEED_MID_DAY_MINUTE = 30,
  FEED_EVENING_HOUR = 20,
  FEED_EVENING_MINUTE = 0;

// temperature constants
const float CHILLER_ON_TEMP_F = 80.5f,  // temp chiller is turned on
  CHILLER_OFF_TEMP_F = 80.0f,  // temp chiller is turned off
  HEATER_ON_TEMP_F = 77.0f,  // temp heater is turned on
  HEATER_OFF_TEMP_F = 77.5f;  // temp heater is turned off

// skimmer constants
const int SKIMMER_ON_HOUR = 8,
  SKIMMER_ON_MINUTE = 0,
  SKIMMER_OFF_HOUR = 22,
  SKIMMER_OFF_MINUTE = 0;
  
// wave constants
const int WAVE_ON_TIME_SEC = 14,
  BOTH_WAVE_ON_TIME_SEC = 1,
  FEED_WAVE_MODE_TIME_SEC = 90;

// RO air pump (stir) constants
const int RO_AIR_PUMP_INTERVAL_MINUTES = 60,
  RO_AIR_PUMP_ON_SECONDS= 45;
//////////////////////////////
// End Control Constants
//////////////////////////////

// feeder stepper constants
const int FEEDER_STEPPER_MODE = 8,
  FEEDER_STEPPER_STEPS = 1600,
  FEEDER_STEPPER_STEP_DELAY_MS = 5000;          

// constants converted to hour/minute fraction for easier comparison
const float DAY1_ON_HOUR_MINUTE = hourMinuteToHour(DAY1_ON_HOUR, DAY1_ON_MINUTE),
  DAY1_OFF_HOUR_MINUTE = hourMinuteToHour(DAY1_OFF_HOUR, DAY1_OFF_MINUTE),
  DAY2_ON_HOUR_MINUTE = hourMinuteToHour(DAY2_ON_HOUR, DAY2_ON_MINUTE),
  DAY2_OFF_HOUR_MINUTE = hourMinuteToHour(DAY2_OFF_HOUR, DAY2_OFF_MINUTE),
  SUMP_ON_HOUR_MINUTE = hourMinuteToHour(SUMP_ON_HOUR, SUMP_ON_MINUTE),
  SUMP_OFF_HOUR_MINUTE = hourMinuteToHour(SUMP_OFF_HOUR, SUMP_OFF_MINUTE),
  SKIMMER_ON_HOUR_MINUTE = hourMinuteToHour(SKIMMER_ON_HOUR, SKIMMER_ON_MINUTE),
  SKIMMER_OFF_HOUR_MINUTE = hourMinuteToHour(SKIMMER_OFF_HOUR, SKIMMER_OFF_MINUTE);

// pin definitions
const byte SUMP_TEMP_PIN = 22,
  OUTSIDE_TEMP_PIN = 23,
  HOOD_TEMP_PIN = 24,
  OVERFLOW_TEMP_PIN = 25,
  INSIDE_TEMP_PIN = 26,

  RO_FLOAT1_PIN = 30, 
  RO_FLOAT2_PIN = 29,
  RO_FLOAT3_PIN = 28,
  SUMP_TOP_OFF_FLOAT_PIN = 32,
  SUMP_EMPTY_FLOAT_PIN = 33,
  SKIM_FLOAT1_PIN = 35, 
  SKIM_FLOAT2_PIN = 34,

  TOP_OFF_RELAY_PIN = 40,
  WAVE1_RELAY_PIN = 45,
  WAVE2_RELAY_PIN = 42,
  HEATER_RELAY_PIN = 43, 
  CHILLER_RELAY_PIN = 44,
  
  DAY1_LIGHT_RELAY_PIN = 48,
  DAY2_LIGHT_RELAY_PIN = 49,
  SUMP_LIGHT_RELAY_PIN = 50,
  SKIMMER_RELAY_PIN = 51,
  RO_AIR_PUMP_RELAY_PIN = 52,
  
  FEEDER_STEPPER_DIR_PIN = 36,
  FEEDER_STEPPER_STEP_PIN = 37,
  FEEDER_STEPPER_MS1_PIN = 46,
  FEEDER_STEPPER_MS2_PIN = 38,
  FEEDER_STEPPER_SLEEP_PIN = 47,
  
  PH_PIN = 15,

  LCD1_RX_PIN = 15,  // rxPin is immaterial; assigned to unused pin
  LCD1_TX_PIN = 14,

  LCD2_RX_PIN = 17,  // rxPin is immaterial; assigned to unused pin
  LCD2_TX_PIN = 16,
  
  IR_RECV_PIN = 39;

// temperature threshold constants  
const float MIN_REASONABLE_H20_TEMP_F = 50.0f,  // low H20 temp we should never see; if we do most likely a temp probe issue
  MAX_REASONABLE_H20_TEMP_F = 120.0f, // high H20 temp we should never see
  INIT_TEMP = -1000.0f;

// temperature history 
const int TEMP_MOVING_ARRAY_SIZE = 10;

// Infrared constants
const int
  SONY_IR_0 = 0x910,
  SONY_IR_1 = 0x10,
  SONY_IR_2 = 0x810,
  SONY_IR_3 = 0x410,
  SONY_IR_4 = 0xC10,
  SONY_IR_5 = 0x210,
  SONY_IR_6 = 0xA10,
  SONY_IR_7 = 0x610,
  SONY_IR_8 = 0xE10,
  SONY_IR_9 = 0x110;
    
// temperature history index
int overflowTempMovingIndex = 0,
  insideTempMovingIndex = 0,
  outsideTempMovingIndex = 0;

// temperature variables
float outsideTempSmoothed = INIT_TEMP, 
  hoodTemp = INIT_TEMP, 
  sumpTemp = INIT_TEMP,
  overflowTempSmoothed = INIT_TEMP, 
  insideTemp = INIT_TEMP, 
  overflowTempMovingArray[TEMP_MOVING_ARRAY_SIZE], 
  outsideTempMovingArray[TEMP_MOVING_ARRAY_SIZE];

// OneWire library temperature objects
OneWire overflowTempOneWire(OVERFLOW_TEMP_PIN); 
OneWire insideTempOneWire(INSIDE_TEMP_PIN); 
OneWire outsideTempOneWire(OUTSIDE_TEMP_PIN); 
OneWire hoodTempOneWire(HOOD_TEMP_PIN); 
OneWire sumpTempOneWire(SUMP_TEMP_PIN); 

// Infrared variables
IRrecv irRecv(IR_RECV_PIN);
decode_results irResults;

// temperature sensor error flags
boolean overflowTempError = false,
  sumpTempError = false,
  outsideTempError = false,
  insideTempError = false,
  hoodTempError = false;

// custom LCD character definitions
const char DEGREE_LCD_CHAR_DEFINE[] = "?D01E121E0000000000",
  DEGREE_LCD_CHAR[] = "?0",
  RISING_LCD_CHAR_DEFINE[] = "?D1040E150404040400",
  RISING_LCD_CHAR[] = "?1",
  FALLING_LCD_CHAR_DEFINE[] = "?D20004040404150E04",
  FALLING_LCD_CHAR[] = "?2",
  SUN_LCD_CHAR_DEFINE[] = "?D304150E0E0E150400",
  SUN_LCD_CHAR[] = "?3",
  DARK_LCD_CHAR_DEFINE[] = "?D41F1F1F1F1F1F1F1F",
  DARK_LCD_CHAR[] = "?4",
  HAPPY_FISH1_LCD_CHAR_DEFINE[] = "?D50103171F1F170301",
  HAPPY_FISH1_LCD_CHAR[] = "?5",
  HAPPY_FISH2_LCD_CHAR_DEFINE[] = "?D6181C12131F1E1C18",
  HAPPY_FISH2_LCD_CHAR[] = "?6",  
  WAVE_LCD_CHAR_DEFINE[] = "?D702070D1C1C1E1F1F",
  WAVE_LCD_CHAR[] = "?7";        
  
const char * NO_CHANGE_LCD_CHAR = "-";

// ph constants
const int PH_MOVING_ARRAY_SIZE = 30;

// ph variables
float phSmoothed = 0.0f;
int phAnalog = 0, phMovingIndex = 0;
float phMovingArray[PH_MOVING_ARRAY_SIZE];

// float variables
int roFloat1 = LOW, 
roFloat2 = LOW, 
roFloat3 = LOW, 
skimFloat1 = HIGH, 
skimFloat2 = HIGH, 
sumpTopOffFloat = LOW,
sumpEmptyFloat = LOW;

// LCD serial communication objects
NewSoftSerial lcd1Serial = NewSoftSerial(LCD1_RX_PIN, LCD1_TX_PIN);
NewSoftSerial lcd2Serial = NewSoftSerial(LCD2_RX_PIN, LCD2_TX_PIN);

// LCD variables
boolean currentTimeColonBlinkOn = false;
long lastCurrentTimeColonBlink = now();
boolean errorMessageBlinkOn = false;

// wave constants
const int WAVE_DIRECTION_1 = 1,
  WAVE_DIRECTION_2 = 2;
  
// wave variables
long waveOnTimeSec = 0,
  feedWaveModeTimeSec = 0,
  previousWaveDirection = WAVE_DIRECTION_1; 
boolean 
feedWaveMode = false;


// RO air pump variables
long roAirPumpOnTimeSec = 0;

// time variables
long lastLoopPreLcdUpdateSec = now();

// feed variables
boolean feeding = false;
time_t lastFedTime;

// moving averages constants
const int MOVING_SAMPLE_INTERVAL_SEC = 3;

// moving averages variables
int movingSampleSec = 0; 
boolean movingSample = true; // whether to take a data sample on this loop

/**
 * Arduino setup method
 */
void setup()  
{                
  if(SERIAL_DEBUG){
    Serial.begin(57600); // initialize hardware serial port    
  }
  
  Wire.begin();
  irRecv.enableIRIn();
 
  setSyncProvider(RTC.get);

  setupAlarms();
  setupOutputPins();  
  initRelays();
  updatePh();
  
  setupLcd(&lcd1Serial, LCD1_TX_PIN);
  setupLcd(&lcd2Serial, LCD2_TX_PIN);
 
  delay(1000);  
}

/**
 * Arduino loop method
 */
void loop()                     
{        
  updateTemps();
  updateFloats();
  updatePh();
  
  syncRelays();
  
  Alarm.delay(0);
  lastLoopPreLcdUpdateSec = now();

  handleInfrared();
  
  if(SERIAL_DEBUG){
    serialDebug();
  }
  
  refreshLcd1(&lcd1Serial);
  refreshLcd2(&lcd2Serial);  
  
  manageMovingSample();
}

/**
 *
 */
void manageMovingSample(){
  Serial.print("movingSample: ");
  Serial.println(movingSample ? "true" : " false"); 
  if(movingSample){
    movingSample = false;
    movingSampleSec = 0;
  }else{
    movingSampleSec += now() - lastLoopPreLcdUpdateSec;
  
    if(movingSampleSec >= MOVING_SAMPLE_INTERVAL_SEC){
      movingSample = true;
    }
  }
}

/**
 * Handles any action that need to be taken from Infrared
 */
void handleInfrared() {
  if (!irRecv.decode(&irResults)) {
    irRecv.resume();
    return;
  }
  
 // if(SERIAL_DEBUG){
 //    Serial.println(irResults->value, HEX);
 // }
  
  switch(irResults.value){
    case SONY_IR_0:
    feed();
    break;
    case SONY_IR_1:
    if(digitalRead(DAY1_LIGHT_RELAY_PIN) == LOW){
          day1LightOn();  
    }else{
          day1LightOff();  
    }
    break;
    case SONY_IR_2:
    if(digitalRead(DAY2_LIGHT_RELAY_PIN) == LOW){
          day2LightOn();  
    }else{
          day2LightOff();  
    }
    break;
    case SONY_IR_3:
    if(digitalRead(SUMP_LIGHT_RELAY_PIN) == LOW){
          sumpLightOn();  
    }else{
          sumpLightOff();  
    }
    break;
  }
  
  irRecv.resume();
}

/**
 * Refreshes LCD 1 
 */
void refreshLcd1(NewSoftSerial * lcdSerial)
{  
  char tempBuffer[7];

  // line 1
  lcdSerial->print("?y0?x00");  
  // h20 temp
  lcdSerial->print("H2O ");    
  fmtDouble(overflowTempSmoothed, 2, tempBuffer, 7);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
  lcdSerial->print(getDeltaChar(overflowTempMovingIndex, overflowTempMovingArray, TEMP_MOVING_ARRAY_SIZE));  
  lcdSerial->print("  ");
  
    // heating/cooling/neutral (fish temp)
  if(digitalRead(HEATER_RELAY_PIN) == HIGH){
    lcdSerial->print("Heating"); 
  }else if(digitalRead(CHILLER_RELAY_PIN) == HIGH){
    lcdSerial->print("Cooling"); 
  }else{
    lcdSerial->print(HAPPY_FISH1_LCD_CHAR);
    lcdSerial->print(HAPPY_FISH2_LCD_CHAR);
    lcdSerial->print(" Temp"); 
  }
  

  // line 2
  // outside temp
  lcdSerial->print("Air ");
  fmtDouble(outsideTempSmoothed, 2, tempBuffer, 7);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
  lcdSerial->print(getDeltaChar(outsideTempMovingIndex, outsideTempMovingArray, TEMP_MOVING_ARRAY_SIZE));  
  lcdSerial->print(" ");
   
  // waves
  if(digitalRead(WAVE1_RELAY_PIN) == HIGH && digitalRead(WAVE2_RELAY_PIN) == HIGH){
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
  }else if(digitalRead(WAVE1_RELAY_PIN) == HIGH){
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print("____");
  }else if(digitalRead(WAVE2_RELAY_PIN) == HIGH){
    lcdSerial->print("____");
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
    lcdSerial->print(WAVE_LCD_CHAR);
  }else{
    lcdSerial->print("________");
  }
  
  // line 3
  lcdSerial->print("?y2?x00");  
  
  // sun set/rise time 1
  lcdSerial->print(SUN_LCD_CHAR);
  if(digitalRead(DAY2_LIGHT_RELAY_PIN) == HIGH){
    lcdSerial->print(FALLING_LCD_CHAR);  
    lcdSerial->print(" ");
    lcdSerial->print(DAY1_OFF_HOUR - 12);
    lcdSerial->print(":");
    if(DAY1_OFF_MINUTE < 10){
      lcdSerial->print("0");
    }
    lcdSerial->print(DAY1_OFF_MINUTE);
    lcdSerial->print("pm");          
     if(DAY1_OFF_HOUR - 12 < 10){
      lcdSerial->print(" ");
    }
  }else{
    lcdSerial->print(RISING_LCD_CHAR);
    lcdSerial->print(" ");  
    lcdSerial->print(DAY1_ON_HOUR);
    lcdSerial->print(":");
    if(DAY1_ON_MINUTE < 10){
      lcdSerial->print("0");
    }  
    lcdSerial->print(DAY1_ON_MINUTE);
    lcdSerial->print("am");
    if(DAY1_ON_HOUR < 10){
      lcdSerial->print(" ");
    }    
  }
  
  // ph
  char phBuffer[5];
  fmtDouble(phSmoothed, 2, phBuffer, 6);  
  //lcdSerial->print("          ");  // not showing anything when ph probe not hooked up
  lcdSerial->print("  pH ");
  lcdSerial->print(phBuffer);
  lcdSerial->print(getDeltaChar(phMovingIndex, phMovingArray, PH_MOVING_ARRAY_SIZE));  
  
  // line 4
  lcdSerial->print("?y3?x00");   
  
    // sun set/rise time 2
  lcdSerial->print(SUN_LCD_CHAR);
  lcdSerial->print(SUN_LCD_CHAR);
  if(digitalRead(DAY2_LIGHT_RELAY_PIN) == HIGH){
    lcdSerial->print(FALLING_LCD_CHAR);  
    lcdSerial->print(" ");
    lcdSerial->print(DAY2_OFF_HOUR - 12);
    lcdSerial->print(":");
    if(DAY2_OFF_MINUTE < 10){
      lcdSerial->print("0");
    } 
    lcdSerial->print(DAY2_OFF_MINUTE);
    lcdSerial->print("pm");          
    if(DAY2_OFF_HOUR - 12 < 10){
      lcdSerial->print(" ");
    }
  }else{
    lcdSerial->print(RISING_LCD_CHAR);
    lcdSerial->print(" ");  
    lcdSerial->print(DAY2_ON_HOUR);
    lcdSerial->print(":");
    if(DAY2_ON_MINUTE < 10){
      lcdSerial->print("0");
    }  
    lcdSerial->print(DAY2_ON_MINUTE);
    lcdSerial->print("am"); 
    if(DAY2_ON_HOUR < 10){
      lcdSerial->print(" ");
    }    
  }
 
  lcdSerial->print("  ");
    
  // current time
  int currentHour = hourFormat12(); 
  int currentMinute = minute();  
  if(currentHour < 10){
    lcdSerial->print(" ");
  }  
  lcdSerial->print(currentHour);
  if(now() - lastCurrentTimeColonBlink >= 1){
     currentTimeColonBlinkOn = !currentTimeColonBlinkOn;
     lastCurrentTimeColonBlink = now();
  }
  if(currentTimeColonBlinkOn){
    lcdSerial->print(":");
  }else{
    lcdSerial->print(" ");
  }
  if(currentMinute < 10){
    lcdSerial->print("0");
  }  
  lcdSerial->print(currentMinute);
  if(isAM()){
    lcdSerial->print("am");
  }else{
    lcdSerial->print("pm");
  }  

  
  
  
}

/**
 * Refreshes LCD 2
 */
void refreshLcd2(NewSoftSerial * lcdSerial)
{
  // line 1
  lcdSerial->print("?y0?x00");  
  
  // RO column heading
  lcdSerial->print("RO ");
  
  // lights
  lcdSerial->print("Day ");
  if(digitalRead(DAY1_LIGHT_RELAY_PIN) == HIGH){
    lcdSerial->print(SUN_LCD_CHAR);
    lcdSerial->print(SUN_LCD_CHAR);
  }else{
    lcdSerial->print("--");
  }
  if(digitalRead(DAY2_LIGHT_RELAY_PIN) == HIGH){
    lcdSerial->print(SUN_LCD_CHAR);
    lcdSerial->print(SUN_LCD_CHAR);
  }else{
    lcdSerial->print("--");
  }
  
  lcdSerial->print(" Sump ");
  if(digitalRead(SUMP_LIGHT_RELAY_PIN) == HIGH){
    lcdSerial->print(SUN_LCD_CHAR);
    lcdSerial->print(SUN_LCD_CHAR);
  }else{
    lcdSerial->print("--");
  }  
  
  // line 2    
  lcdSerial->print("?y1?x00");  

  // RO float 3
  if(roFloat3 == LOW){
    lcdSerial->print(DARK_LCD_CHAR);
    lcdSerial->print(DARK_LCD_CHAR);
  }else{
    lcdSerial->print("  ");
  }  

  // RO air pump
  lcdSerial->print(" Stir ");    
  if(digitalRead(RO_AIR_PUMP_RELAY_PIN) == HIGH){
    lcdSerial->print("On "); 
  }else{
    lcdSerial->print("Off"); 
  }

  // skimmer
  lcdSerial->print(" Skim ");    
  if(digitalRead(SKIMMER_RELAY_PIN) == HIGH){
    lcdSerial->print("On "); 
  }else{
    lcdSerial->print("Off"); 
  }

  // line 3
  lcdSerial->print("?y2?x00");  
  
  // RO float 2
  if(roFloat2 == LOW){
    lcdSerial->print(DARK_LCD_CHAR);
    lcdSerial->print(DARK_LCD_CHAR);
  }else{
    lcdSerial->print("  ");
  }

  // error message or happy fish
  // ADD CHANGE MESSAGE HERE
  lcdSerial->print(" ");  
  char * errorMessage = 0;
  if(overflowTempError){
    errorMessage = "OVER PROBE ERR";
  }else if(sumpTempError){
    errorMessage = "SUMP PROBE ERR ";
  }else if(insideTempError){
    errorMessage = "IN PROBE ERR   ";
  }else if(outsideTempError){
    errorMessage = "OUT PROBE ERR  ";
  }else if(hoodTempError){
    errorMessage = "HOOD PROBE ERR ";
  }else if(digitalRead(SKIM_FLOAT1_PIN) == LOW){
    errorMessage = "EMPTY SKIMMER  ";    
  }
  
  if(errorMessage != 0){
    if(errorMessageBlinkOn){
       lcdSerial->print(errorMessage);
    }else{
      lcdSerial->print("               ");
    }
    errorMessageBlinkOn = !errorMessageBlinkOn;
  }else{
    // happy fish    
    lcdSerial->print(HAPPY_FISH1_LCD_CHAR);
    lcdSerial->print(HAPPY_FISH2_LCD_CHAR);
    lcdSerial->print(" "); 
    lcdSerial->print(HAPPY_FISH1_LCD_CHAR);
    lcdSerial->print(HAPPY_FISH2_LCD_CHAR);
    lcdSerial->print(" "); 
    lcdSerial->print(HAPPY_FISH1_LCD_CHAR);
    lcdSerial->print(HAPPY_FISH2_LCD_CHAR);
    lcdSerial->print(" "); 
    lcdSerial->print(HAPPY_FISH1_LCD_CHAR);
    lcdSerial->print(HAPPY_FISH2_LCD_CHAR);
    lcdSerial->print(" "); 
    lcdSerial->print(HAPPY_FISH1_LCD_CHAR);
    lcdSerial->print(HAPPY_FISH2_LCD_CHAR);
    lcdSerial->print("  "); 
  }
  
  // line 4  
  lcdSerial->print("?y3?x00");  
  
  // RO float 1
  if(roFloat1 == LOW){
    lcdSerial->print(DARK_LCD_CHAR);
    lcdSerial->print(DARK_LCD_CHAR);
  }else{
    lcdSerial->print("  ");
  }    
  lcdSerial->print(" ");  
  
  // activity status or last fed
  if(feeding){
    lcdSerial->print("Feeding          "); 
  }else if(digitalRead(TOP_OFF_RELAY_PIN) == HIGH){
    lcdSerial->print("Topping Off      "); 
  }else if(lastFedTime != 0){
    lcdSerial->print("Last Fed ");    
    int lastFedHour = hour(lastFedTime); 
    int lastFedMinute = minute(lastFedTime); 
    if(lastFedHour <= 12){
      lcdSerial->print(lastFedHour);
     }else{
       lcdSerial->print(lastFedHour-12);
     }
     lcdSerial->print(":");
     if(lastFedMinute < 10){
       lcdSerial->print("0");
     }  
     lcdSerial->print(lastFedMinute);
     if(lastFedHour < 12){
       lcdSerial->print("am");
     }else{
       lcdSerial->print("pm");
     }  
     if(lastFedHour < 10){
       lcdSerial->print(" ");
     }  
     lcdSerial->print(" ");
  }else{
      lcdSerial->print("                 ");
  }
  //   Emptying   Empty   <º))))><
}

/**
 * Sync relays with state of controller
 */
void syncRelays(){
  // top off
  if(sumpTopOffFloat == HIGH){
    digitalWrite(TOP_OFF_RELAY_PIN, HIGH);
  }else{
    digitalWrite(TOP_OFF_RELAY_PIN, LOW);
  }
  
  // heater/chiller
  if(!overflowTempError){   
    if(overflowTempSmoothed <= HEATER_ON_TEMP_F){
      digitalWrite(HEATER_RELAY_PIN, HIGH);
    }   

    if(overflowTempSmoothed >= HEATER_OFF_TEMP_F){  
      digitalWrite(HEATER_RELAY_PIN, LOW);    
    }
    
    if(overflowTempSmoothed >= CHILLER_ON_TEMP_F){
      digitalWrite(CHILLER_RELAY_PIN, HIGH);
    }
    
    if(overflowTempSmoothed <= CHILLER_OFF_TEMP_F){
      digitalWrite(CHILLER_RELAY_PIN, LOW);    
    }    
  }else{
    // turn heater/chiller off if probe error
    digitalWrite(HEATER_RELAY_PIN, LOW);    
    digitalWrite(CHILLER_RELAY_PIN, LOW);    
  }
  
  // waves
  if(feedWaveMode){ // feed mode
    feedWaveModeTimeSec += now() - lastLoopPreLcdUpdateSec;
    if(feedWaveModeTimeSec >= FEED_WAVE_MODE_TIME_SEC){
      feedWaveModeTimeSec = 0;
      feedWaveMode = false;
      waveDirection1();
    }
  }else{ // regular mode
    if(digitalRead(WAVE1_RELAY_PIN) == HIGH || digitalRead(WAVE2_RELAY_PIN) == HIGH){
      waveOnTimeSec += now() - lastLoopPreLcdUpdateSec;
    }
      
    if(digitalRead(WAVE1_RELAY_PIN) == LOW && digitalRead(WAVE2_RELAY_PIN) == LOW){
      waveDirection1();
    }else if(digitalRead(WAVE1_RELAY_PIN) == HIGH && digitalRead(WAVE2_RELAY_PIN) == HIGH && waveOnTimeSec >= BOTH_WAVE_ON_TIME_SEC){
      if(previousWaveDirection == WAVE_DIRECTION_1){
        waveDirection2();
      }else{
        waveDirection1();
      }
    }else if((digitalRead(WAVE1_RELAY_PIN) == HIGH || digitalRead(WAVE2_RELAY_PIN) == HIGH) && waveOnTimeSec >= WAVE_ON_TIME_SEC){
      waveDirectionBoth();
    }
  }
  
  // RO air pump
  if(digitalRead(RO_AIR_PUMP_RELAY_PIN) == HIGH){
    roAirPumpOnTimeSec += now() - lastLoopPreLcdUpdateSec;
    
    if(roAirPumpOnTimeSec > RO_AIR_PUMP_ON_SECONDS){
      digitalWrite(RO_AIR_PUMP_RELAY_PIN, LOW); 
      roAirPumpOnTimeSec = 0;
    }
  } 
  
}

void updatePh(){  
  if(movingSample){
    phAnalog = analogRead(PH_PIN);
      
    sumpTempError = !updateTemp(&sumpTempOneWire, &sumpTemp, sumpTempError);
  
    float phNow = 7.0 - (2.5 - phAnalog / 204.6) / (0.257179 + 0.000941468 * convertFahrenheitToCelius(sumpTemp));  // using sump temp since ph probe is in sump
  
    phMovingArray[phMovingIndex] = phNow;
    phMovingIndex = ++phMovingIndex % PH_MOVING_ARRAY_SIZE;
    phSmoothed = calculateAverage(phMovingArray, PH_MOVING_ARRAY_SIZE);
  }
}

/**
 * Calculates the average of values in the array, discounting any values <= 0.
 */
float calculateAverage(float fa[], int arrayLength){
  int count = 0;
  float sum = 0.0f;
  for(int i = 0; i < arrayLength; i++){
    if(fa[i] > 0){
      sum += fa[i];
      ++count;
    }
  }
  
 if(count <= 0){
   return 0.0f; 
 }else{
   return sum / (float)count;
 }
}
  
/**
 * Updates all temps
 */
void updateTemps(){
  if(movingSample){
    float overflowTempNow;
    overflowTempError = !updateTemp(&overflowTempOneWire, &overflowTempNow, overflowTempError);  
    if(overflowTempError || overflowTempNow <= MIN_REASONABLE_H20_TEMP_F || overflowTempNow >= MAX_REASONABLE_H20_TEMP_F){
      overflowTempError = true;
    }
    
    if(!overflowTempError){
      overflowTempMovingArray[overflowTempMovingIndex] = overflowTempNow;      
      overflowTempSmoothed = calculateAverage(overflowTempMovingArray, TEMP_MOVING_ARRAY_SIZE);
      overflowTempMovingIndex = ++overflowTempMovingIndex % TEMP_MOVING_ARRAY_SIZE;
    }
    
    float outsideTempNow;
    outsideTempError = !updateTemp(&outsideTempOneWire, &outsideTempNow, outsideTempError);
     
    if(!outsideTempError){
      outsideTempMovingArray[outsideTempMovingIndex] = outsideTempNow;
      outsideTempSmoothed = calculateAverage(outsideTempMovingArray, TEMP_MOVING_ARRAY_SIZE);
      outsideTempMovingIndex = ++outsideTempMovingIndex % TEMP_MOVING_ARRAY_SIZE;
    }
  }
  
  sumpTempError = !updateTemp(&sumpTempOneWire, &sumpTemp, sumpTempError);
    
  if(sumpTempError || sumpTemp <= MIN_REASONABLE_H20_TEMP_F || sumpTemp >= MAX_REASONABLE_H20_TEMP_F){
    sumpTempError = true;
  }
   
  hoodTempError = !updateTemp(&hoodTempOneWire, &hoodTemp, hoodTempError);       
  insideTempError = !updateTemp(&insideTempOneWire, &insideTemp, insideTempError);       
}

/** 
 * Updates all floats
 */
void updateFloats(){
  roFloat1 = digitalRead(RO_FLOAT1_PIN);
  roFloat2 = digitalRead(RO_FLOAT2_PIN);
  roFloat3 = digitalRead(RO_FLOAT3_PIN);
  skimFloat1 = digitalRead(SKIM_FLOAT1_PIN);
  skimFloat2 = digitalRead(SKIM_FLOAT2_PIN);
  sumpTopOffFloat = digitalRead(SUMP_TOP_OFF_FLOAT_PIN);
  sumpEmptyFloat = digitalRead(SUMP_EMPTY_FLOAT_PIN);
}

/**
 * Updates a single temp
 */
boolean updateTemp(OneWire * oneWireTherm, float * fltTemp, boolean previousError){
  byte thermAddr[8], data[12];
  oneWireTherm->reset_search();   
  oneWireTherm->search(thermAddr); 

  if(OneWire::crc8(thermAddr,7)!=thermAddr[7]) { // checksum invalid
    return false;
  }
 
  if(!oneWireTherm->reset()){
    return false; 
  }
  
  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0x44, 1); // start conversation w/ parasite power
  
  if(previousError){
    delay(1000);
  }
  
  if(!oneWireTherm->reset()){
    return false; 
  }
  
  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0xBE); // read scratchpad  
  for(int i=0;i<9;i++) { // need 9 bytes
    data[i] = oneWireTherm->read();
  }

  *fltTemp = getTemperature(data[0], data[1]);

  return true;
}

/** 
 * Sets up LCD
 */
void setupLcd(NewSoftSerial * lcdSerial, int pin){
  pinMode(pin, OUTPUT);
  lcdSerial->begin(9600); 

  lcdSerial->print("?G420");
  delay(100);	           // pause to allow LCD EEPROM to program

  lcdSerial->print("?Bff");  // set backlight to 40 hex
  delay(100);                // pause to allow LCD EEPROM to program

  lcdSerial->print(DEGREE_LCD_CHAR_DEFINE);  // define special characters
  delay(100); 
    
  lcdSerial->print(RISING_LCD_CHAR_DEFINE);
  delay(100); 
    
  lcdSerial->print(FALLING_LCD_CHAR_DEFINE);
  delay(100); 
  
  lcdSerial->print(SUN_LCD_CHAR_DEFINE);
  delay(100); 
  
  lcdSerial->print(DARK_LCD_CHAR_DEFINE);
  delay(100); 
  
  lcdSerial->print(HAPPY_FISH1_LCD_CHAR_DEFINE);
  delay(100); 
  
  lcdSerial->print(HAPPY_FISH2_LCD_CHAR_DEFINE);
  delay(100); 
  
  lcdSerial->print(WAVE_LCD_CHAR_DEFINE);
  delay(100); 
  
  lcdSerial->print("?c0");  // turn cursor off
  delay(200);  

  lcdSerial->print("?f");   // clear the LCD
  lcdSerial->print("?f");
  delay(1000);  
}

/**
 * Debug method to show temps on passed LCD serial 
 */
void refreshLcdTemps(NewSoftSerial * lcdSerial){
  char tempBuffer[6];

  lcdSerial->print("?y0?x00");  
  lcdSerial->print("H2O ");    
  fmtDouble(overflowTempSmoothed, 1, tempBuffer, 6);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
  lcdSerial->print(" In ");
  fmtDouble(insideTemp, 1, tempBuffer, 6);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
  lcdSerial->print("?y1?x00");  
  lcdSerial->print("Out ");
  fmtDouble(outsideTempSmoothed, 1, tempBuffer, 6);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
  lcdSerial->print(" Sump ");
  fmtDouble(sumpTemp, 1, tempBuffer, 6);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
  lcdSerial->print("?y2?x00");  
  lcdSerial->print("Hood ");
  fmtDouble(hoodTemp, 1, tempBuffer, 6);  
  lcdSerial->print(tempBuffer);
  lcdSerial->print(DEGREE_LCD_CHAR);
}

/**
 * Debug method to show floats on passed LCD serial
 */ 
void refreshLcdFloats(NewSoftSerial * lcdSerial){
  lcdSerial->print("?y0?x00");  
  lcdSerial->print("RO 1=");    
  lcdSerial->print(roFloat1);
  lcdSerial->print(" RO 2=");    
  lcdSerial->print(roFloat2);
  lcdSerial->print(" RO 3=");    
  lcdSerial->print(roFloat3);
  lcdSerial->print("?y1?x00");  
  lcdSerial->print("Skim 1=");    
  lcdSerial->print(skimFloat1);
  lcdSerial->print(" Skim 2=");    
  lcdSerial->print(skimFloat2);
  lcdSerial->print("?y2?x00");  
  lcdSerial->print("Sump TO=");    
  lcdSerial->print(sumpTopOffFloat);
  lcdSerial->print(" Sump Emp=");    
  lcdSerial->print(sumpEmptyFloat);
}

/**
 * Converts celsius to fahrenheit
 */ 
float convertCeliusToFahrenheit(float c) {
  return((c*1.8)+32); 
}

/**
 * Converts fahrenheit to celsius
 */ 
float convertFahrenheitToCelius(float f) {
  return((f-32)*0.555555556); 
}

/**
 * Gets temp from bytes for OneWire
 */
float getTemperature(int lowByte, int highByte) {
  int intPostDecimal, boolSign, intHexTempReading, intTempReadingBeforeSplit, preDecimal, i;
  float fltPostDecimal, fltTemp;
  intHexTempReading = (highByte << 8) + lowByte;
  boolSign = intHexTempReading & 0x8000;
  if(boolSign) {
    intHexTempReading = (intHexTempReading ^ 0xffff) + 1;
  }
  intTempReadingBeforeSplit = 6.25 * intHexTempReading; // multiply by (100 * precision) = (100 * 0.0625) = 6.25 = 12-bit precision
  preDecimal = intTempReadingBeforeSplit / 100;
  intPostDecimal = intTempReadingBeforeSplit % 100;
  fltPostDecimal = intPostDecimal;
  if(intPostDecimal<10) {
    fltTemp = preDecimal+(fltPostDecimal/1000);
  }
  else {
    fltTemp = preDecimal+(fltPostDecimal/100);
  }  
  if(boolSign) { 
    fltTemp = -fltTemp; 
  }
  return convertCeliusToFahrenheit(fltTemp);
}

/**
 * Writes status info to Serial for debugging
 */
void serialDebug(){
    Serial.print("overflowTempSmoothed=");
    Serial.print(overflowTempSmoothed);
    Serial.print("; insideTemp=");
    Serial.print(insideTemp);
    Serial.print("; outsideTempSmoothed=");
    Serial.print(outsideTempSmoothed);
    Serial.print("; hoodTemp=");
    Serial.print(hoodTemp);
    Serial.print("; sumpTemp=");
    Serial.print(sumpTemp);
    Serial.print("; roFloat1=");
    Serial.print(roFloat1);
    Serial.print("; roFloat2=");
    Serial.print(roFloat2);
    Serial.print("; roFloat3=");
    Serial.print(roFloat3);
    Serial.print("; skimFloat1=");
    Serial.print(skimFloat1);
    Serial.print("; skimFloat2=");
    Serial.print(skimFloat2);
    Serial.print("; sumpTopOffFloat=");
    Serial.print(sumpTopOffFloat);
    Serial.print("; sumpEmptyFloat=");
    Serial.print(sumpEmptyFloat);
    Serial.print("; phSmoothed: ");
    Serial.print(phSmoothed);
    Serial.print("; waveOnTimeSec: ");
    Serial.println(waveOnTimeSec);
}

/**
 * Checks moving array buffer to return a delta char 
 */
const char * getDeltaChar(int movingIndex, float movingArray[], int arrayLength){
  float lastValue = movingArray[movingIndex];
  float value = 0.0f;
  float deltaSum = 0.0f;
  for(int i = 0; i < arrayLength; i++){
    value =  movingArray[(movingIndex + i) % arrayLength];
    if(lastValue > 0 && value > 0){
      deltaSum += (value - lastValue);
    }
    
    lastValue = value;
  }
    
  if(deltaSum == 0){
    return NO_CHANGE_LCD_CHAR;
  }else if(deltaSum > 0){
    return RISING_LCD_CHAR;
  }else{
    return FALLING_LCD_CHAR;
  }
}

/**
 * Setup pins for output
 */
void setupOutputPins(){
  // relays
  pinMode(TOP_OFF_RELAY_PIN, OUTPUT);
  pinMode(WAVE1_RELAY_PIN, OUTPUT);
  pinMode(WAVE2_RELAY_PIN, OUTPUT);
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  pinMode(CHILLER_RELAY_PIN, OUTPUT);
  pinMode(DAY1_LIGHT_RELAY_PIN, OUTPUT);
  pinMode(DAY2_LIGHT_RELAY_PIN, OUTPUT);
  pinMode(SUMP_LIGHT_RELAY_PIN, OUTPUT);
  pinMode(RO_AIR_PUMP_RELAY_PIN, OUTPUT);
  pinMode(SKIMMER_RELAY_PIN, OUTPUT);
  
  // feeder stepper
  pinMode(FEEDER_STEPPER_DIR_PIN, OUTPUT);   
  pinMode(FEEDER_STEPPER_STEP_PIN, OUTPUT);   
  pinMode(FEEDER_STEPPER_MS1_PIN, OUTPUT);   
  pinMode(FEEDER_STEPPER_MS2_PIN, OUTPUT);   
  pinMode(FEEDER_STEPPER_SLEEP_PIN, OUTPUT);   
}

float hourMinuteToHour(int hour, int minute){
  return hour + minute/60.0f;
}

void initRelays(){
  float currentHourMinute = hour() + minute()/60.0f;
  
  // day lights
  if(currentHourMinute >= DAY1_ON_HOUR_MINUTE && currentHourMinute < DAY1_OFF_HOUR_MINUTE){
    day1LightOn();
  }else{
    day1LightOff();
  }
  
  if(currentHourMinute >= DAY2_ON_HOUR_MINUTE && currentHourMinute < DAY2_OFF_HOUR_MINUTE){
    day2LightOn();  
  }else{
    day2LightOff();
  }
  
  // skimmer
  if( currentHourMinute >= SKIMMER_ON_HOUR_MINUTE && currentHourMinute < SKIMMER_OFF_HOUR_MINUTE){
    skimmerOn();
  }else{
    skimmerOff();
  } 
  
  // sump
  if( currentHourMinute >= SUMP_ON_HOUR_MINUTE || currentHourMinute < SUMP_OFF_HOUR_MINUTE){
    sumpLightOn();
  }else{
    sumpLightOff();
  } 
}

void day1LightOn(){
  digitalWrite(DAY1_LIGHT_RELAY_PIN, HIGH);
}

void day1LightOff(){
  digitalWrite(DAY1_LIGHT_RELAY_PIN, LOW);
}

void day2LightOn(){
  digitalWrite(DAY2_LIGHT_RELAY_PIN, HIGH);
}

void day2LightOff(){
  digitalWrite(DAY2_LIGHT_RELAY_PIN, LOW);
}

void roAirPumpOff(){
  digitalWrite(RO_AIR_PUMP_RELAY_PIN, LOW);
}

void roAirPumpOn(){
  digitalWrite(RO_AIR_PUMP_RELAY_PIN, HIGH);
}

void wavesTemporaryOff(){
  digitalWrite(WAVE1_RELAY_PIN, LOW); 
  digitalWrite(WAVE2_RELAY_PIN, LOW); 
}

void wavesOff(){
  digitalWrite(WAVE1_RELAY_PIN, LOW); 
  digitalWrite(WAVE2_RELAY_PIN, LOW); 
}

void wavesOn(){
  digitalWrite(WAVE1_RELAY_PIN, HIGH); 
  digitalWrite(WAVE2_RELAY_PIN, LOW); 
}

void sumpLightOn(){
  digitalWrite(SUMP_LIGHT_RELAY_PIN, HIGH);
}

void sumpLightOff(){
  digitalWrite(SUMP_LIGHT_RELAY_PIN, LOW);
}

void skimmerOn(){
  digitalWrite(SKIMMER_RELAY_PIN, HIGH);   
}

void skimmerOff(){
  digitalWrite(SKIMMER_RELAY_PIN, LOW);   
}

void setPreviousWaveDirection(){
  if(digitalRead(WAVE1_RELAY_PIN) == HIGH && digitalRead(WAVE2_RELAY_PIN) == LOW){
    previousWaveDirection = WAVE_DIRECTION_1;
  }else if(digitalRead(WAVE1_RELAY_PIN) == LOW && digitalRead(WAVE2_RELAY_PIN) == HIGH){
    previousWaveDirection = WAVE_DIRECTION_2;
  }
}

void waveDirection1(){
  setPreviousWaveDirection();
  digitalWrite(WAVE1_RELAY_PIN, HIGH); 
  digitalWrite(WAVE2_RELAY_PIN, LOW); 
  waveOnTimeSec = 0;
}

void waveDirection2(){
  setPreviousWaveDirection();
  digitalWrite(WAVE1_RELAY_PIN, LOW); 
  digitalWrite(WAVE2_RELAY_PIN, HIGH); 
  waveOnTimeSec = 0;
}

void waveDirectionBoth(){
  setPreviousWaveDirection();
  digitalWrite(WAVE1_RELAY_PIN, HIGH); 
  digitalWrite(WAVE2_RELAY_PIN, HIGH); 
  waveOnTimeSec = 0;
}

void setupAlarms(){  
  Alarm.alarmRepeat(DAY1_ON_HOUR, DAY1_ON_MINUTE, 0, day1LightOn); 
  Alarm.alarmRepeat(DAY1_OFF_HOUR, DAY1_OFF_MINUTE, 0, day1LightOff); 
  Alarm.alarmRepeat(DAY2_ON_HOUR, DAY2_ON_MINUTE, 0, day2LightOn); 
  Alarm.alarmRepeat(DAY2_OFF_HOUR, DAY2_OFF_MINUTE, 0, day2LightOff);
  Alarm.alarmRepeat(SUMP_ON_HOUR, SUMP_ON_MINUTE, 0, sumpLightOn); 
  Alarm.alarmRepeat(SUMP_OFF_HOUR, SUMP_OFF_MINUTE, 0, sumpLightOff);
  Alarm.alarmRepeat(SKIMMER_ON_HOUR, SKIMMER_ON_MINUTE, 0, skimmerOn);
  Alarm.alarmRepeat(SKIMMER_OFF_HOUR, SKIMMER_OFF_MINUTE, 0, skimmerOff);
  Alarm.alarmRepeat(FEED_MORNING_HOUR, FEED_MORNING_MINUTE, 0, feed);
  Alarm.alarmRepeat(FEED_MID_DAY_HOUR, FEED_MID_DAY_MINUTE, 0, feed);
  Alarm.alarmRepeat(FEED_EVENING_HOUR, FEED_EVENING_MINUTE, 0, feed);
  Alarm.timerRepeat(RO_AIR_PUMP_INTERVAL_MINUTES * 60, roAirPumpOn);
}

void feed(){
  feeding = true;
  feedWaveMode = true;
  wavesTemporaryOff();
  refreshLcd1(&lcd1Serial);
  refreshLcd2(&lcd2Serial);
  
  digitalWrite(FEEDER_STEPPER_DIR_PIN, LOW);  
  digitalWrite(FEEDER_STEPPER_MS1_PIN, getStepperMs1Mode(FEEDER_STEPPER_MODE));
  digitalWrite(FEEDER_STEPPER_MS2_PIN, getStepperMs2Mode(FEEDER_STEPPER_MODE));
  digitalWrite(FEEDER_STEPPER_SLEEP_PIN, HIGH);
  
  int i = 0;                              
  while(i < FEEDER_STEPPER_STEPS){
      digitalWrite(FEEDER_STEPPER_STEP_PIN, LOW);             
      digitalWrite(FEEDER_STEPPER_STEP_PIN, HIGH);            
      delayMicroseconds(FEEDER_STEPPER_STEP_DELAY_MS);
                                                                                            
      i++;                      
    }           

   digitalWrite(FEEDER_STEPPER_SLEEP_PIN, LOW);        
   
   feeding = false;
   
   lastFedTime = now();
}

int getStepperMs1Mode(int stepperMode){          
  switch(stepperMode){                                                                 
  case 1:
    return 0;
  case 2:
    return 1;
  case 4:
    return  0;
  case 8:
    return 1;
  }
}

int getStepperMs2Mode(int stepperMode){          
  switch(stepperMode){                                                              
  case 1:
    return 0;
  case 2:
    return 0;    
  case 4:
    return 1;    
  case 8:
    return 1;    
  }  
}

