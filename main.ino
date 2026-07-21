/*
 * Arduino Coop Controller – CC2540 / HM-10 BLE Edition
 * ──────────────────────────────────────────────────────
 * Hardware (Arduino Mega):
 *   HM-10 TX  → Mega RX1 (pin 19)
 *   HM-10 RX  → Mega TX1 (pin 18)  [3.3V level — pakai voltage divider]
 *   HM-10 VCC → 3.3V
 *   HM-10 GND → GND
 *
 * BLE Command Format:
 *   ?water=1|0        ?light=1|0|toggle   ?feed
 *   ?door=open|close  ?fan=1|0|toggle     ?status
 *   ?bleinfo          ?digitalread=pin    ?analogread=pin
 *   ?digitalwrite=pin;v  ?pinmode=pin;mode
 */

// ─── Libraries ───────────────────────────────────────────────────────────────
#include <Wire.h>
#include <OneWire.h>
#include <TimeLord.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>
#include "HM_10_BLE.h"

// ─── Forward Declarations ────────────────────────────────────────────────────
// Scheduling
void calculateAlarmTimes();
void scheduleDailyAlarms();
void scheduleTodayAlarms();

// Init
void initState();
void setupOutputPins();

// BLE
void readBLESerial();
void executeBLECommand();
void sendBLEStatus();
void sendBLE(String data);

// Handlers
void handleFloat();
void handleControls();
void handleFan();
void updateAirTemp();
void handleDoor();
void handleDurations();
void serialDebugState();

// Feed
void feed();
void feedOn();
void feedOff();
boolean isFeedOn();
boolean isFeedButton();

// Water
void water();
void waterOn();
void waterOff();
boolean isWaterOn();
boolean isWaterButton();

// Light
void toggleLight();
void lightOn();
void lightOff();
boolean isLightOn();
boolean isLightToggleButton();

// Fan
void toggleFan();
void fanOn();
void fanOff();
boolean isFanOn();

// Door
boolean isDoorOpenSwitch();
boolean isDoorCloseSwitch();
boolean isDoorClosing();
boolean isDoorOpening();
boolean isDoorInMiddle();
boolean isDoorCloseStop();
boolean isDoorOpenStop();
void enableDoorClosing();
void enableDoorOpening();
void disableDoor();

// Utility
boolean updateTemp(OneWire* oneWireTherm, float* fltTemp, boolean previousError);
float convertCeliusToFahrenheit(float c);
float convertFahrenheitToCelius(float f);
float hourMinuteToHour(int hr, int min);
float getTemperature(int lowByte, int highByte);

// ─── Debug ───────────────────────────────────────────────────────────────────
const boolean SERIAL_DEBUG = false;

// ─── Pin Definitions ─────────────────────────────────────────────────────────
const byte LIGHT_TOGGLE_BUTTON_PIN  = 2,
           WATER_BUTTON_PIN         = 3,
           DOOR_RELAY_CLOSE_PIN     = 4,
           DOOR_RELAY_OPEN_PIN      = 5,
           AIR_TEMP_SENSOR_PIN      = 6,
           // Pin 7 bebas (dulu IR_RECV_PIN) — BLE pakai Serial1 hw pin 18/19
           LIGHT_RELAY_PIN          = 8,
           FEEDER_RELAY_PIN         = 9,
           FAN_RELAY_PIN            = 10,
           WATER_SOLENOID_RELAY_PIN = 11;

const byte DOOR_OPEN_SWITCH_PIN   = A0,
           DOOR_CLOSE_SWITCH_PIN  = A1,
           FEED_BUTTON_PIN        = A2,
           WATER_FLOAT_SWITCH_PIN = A3,
           DOOR_CLOSE_STOP_PIN    = A6,
           DOOR_OPEN_STOP_PIN     = A7;

// ─── Delay Constants ─────────────────────────────────────────────────────────
const int POST_BLE_HANDLE_DELAY_MS    = 300,
          POST_BUTTON_HANDLE_DELAY_MS = 750;

// ─── Duration Constants ───────────────────────────────────────────────────────
const int FEED_DURATION_SEC  = 20,
          WATER_DURATION_SEC = 20;

// ─── Time Constants ───────────────────────────────────────────────────────────
const int DOOR_SHUT_AFTER_SUNSET_MIN       = 30,
          LIGHT_PER_DAY_HRS               = 16,
          WATER_AFTER_SUNRISE_MIN         = 15,
          FEED_MORNING_AFTER_SUNRISE_MIN  = 30,
          FEED_AFTERNOON_BEFORE_SUNSET_HR =  4,
          FEED_EVENING_BEFORE_SUNSET_HR   =  1,
          FEED_MID_DAY_HR                 = 12,
          FEED_MID_DAY_MIN                =  0,
          LIGHT_ON_BEFORE_SUNSET_HRS      =  2;

// ─── Temperature Constants ────────────────────────────────────────────────────
const float MIN_REASONABLE_AIR_TEMP_F = 0.0f,
            MAX_REASONABLE_AIR_TEMP_F = 150.0f,
            INIT_TEMP                 = -1000.0f,
            FAN_ON_TEMP_F             = 77.0f,
            FAN_OFF_TEMP_F            = 70.0f;

// ─── Location Constants ───────────────────────────────────────────────────────
const float LATITUDE  =  33.0369867f,
            LONGITUDE = -117.2919818f;
const int   TIMEZONE  = -8;

// ─── BLE Object ──────────────────────────────────────────────────────────────
HM_10_BLE ble(Serial1);
bool bleConnected = false;

// ─── Temperature ─────────────────────────────────────────────────────────────
OneWire airTempOneWire(AIR_TEMP_SENSOR_PIN);
boolean airTempError = false;
float   airTemp      = INIT_TEMP;

// ─── Time / Schedule ─────────────────────────────────────────────────────────
TimeLord coopTimeLord;
long feedOnTimeSec  = 0,
     waterOnTimeSec = 0,
     lastLoopSec    = 0;

byte openDoorHr,   openDoorMin,   closeDoorHr,  closeDoorMin,
     lightOnHr,    lightOnMin,    lightOffHr,   lightOffMin,
     waterHr,      waterMin,
     feedMorningHr,   feedMorningMin,
     feedAfternoonHr, feedAfternoonMin,
     feedEveningHr,   feedEveningMin;

// ─── Control State ────────────────────────────────────────────────────────────
boolean manualWater = false;

// ─── BLE Parser State ─────────────────────────────────────────────────────────
String bleReceive = "";
String bleCmd = "", bleStr1 = "", bleStr2 = "";


// ═════════════════════════════════════════════════════════════════════════════
// SCHEDULING & TIME
// ═════════════════════════════════════════════════════════════════════════════
void calculateAlarmTimes() {
  int nowDay = day(), nowMonth = month(), nowYear = year();

  byte timeLordSunRise[] = {0, 0, 0, (byte)nowDay, (byte)nowMonth, (byte)(nowYear % 100)};
  byte timeLordSunSet[]  = {0, 0, 0, (byte)nowDay, (byte)nowMonth, (byte)(nowYear % 100)};

  coopTimeLord.SunRise(timeLordSunRise);
  coopTimeLord.SunSet(timeLordSunSet);

  if (SERIAL_DEBUG) {
    Serial.print(F("sunrise: ")); Serial.print(timeLordSunRise[2]); Serial.print(':'); Serial.println(timeLordSunRise[1]);
    Serial.print(F("sunset:  ")); Serial.print(timeLordSunSet[2]);  Serial.print(':'); Serial.println(timeLordSunSet[1]);
  }

  openDoorHr  = timeLordSunRise[2];
  openDoorMin = timeLordSunRise[1];
  closeDoorHr = timeLordSunSet[2];
  closeDoorMin= timeLordSunSet[1];

  if (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN >= 60) {
    closeDoorHr += (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN) / 60;
    closeDoorMin = (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN) % 60;
  } else {
    closeDoorMin += DOOR_SHUT_AFTER_SUNSET_MIN;
  }

  float naturalDaylightHr  = 12.0f - (timeLordSunRise[2] + timeLordSunRise[1] / 60.0f);
        naturalDaylightHr += (timeLordSunSet[2] + timeLordSunSet[1] / 60.0f) - 12.0f;

  lightOnHr  = timeLordSunSet[2];
  lightOnMin = timeLordSunSet[1];
  lightOffHr = timeLordSunSet[2];
  lightOffMin= timeLordSunSet[1];

  float lightNeededHrs  = LIGHT_PER_DAY_HRS - naturalDaylightHr;
  int   lightNeededMins = (int)(lightNeededHrs * 60);

  lightOnHr -= LIGHT_ON_BEFORE_SUNSET_HRS;

  if (lightOffMin + lightNeededMins >= 60) {
    lightOffHr += (lightOffMin + lightNeededMins) / 60;
    lightOffMin = (lightOffMin + lightNeededMins) % 60;
  } else {
    lightOffMin += lightNeededMins;
  }

  waterHr  = timeLordSunRise[2];
  waterMin = timeLordSunRise[1];
  feedMorningHr  = timeLordSunRise[2];
  feedMorningMin = timeLordSunRise[1];
  feedAfternoonHr  = timeLordSunSet[2];
  feedAfternoonMin = timeLordSunSet[1];
  feedEveningHr  = timeLordSunSet[2];
  feedEveningMin = timeLordSunSet[1];

  if (waterMin + WATER_AFTER_SUNRISE_MIN >= 60) {
    waterHr += (waterMin + WATER_AFTER_SUNRISE_MIN) / 60;
    waterMin = (waterMin + WATER_AFTER_SUNRISE_MIN) % 60;
  } else {
    waterMin += WATER_AFTER_SUNRISE_MIN;
  }

  if (feedMorningMin + FEED_MORNING_AFTER_SUNRISE_MIN >= 60) {
    feedMorningHr += (feedMorningMin + FEED_MORNING_AFTER_SUNRISE_MIN) / 60;
    feedMorningMin = (feedMorningMin + FEED_MORNING_AFTER_SUNRISE_MIN) % 60;
  } else {
    feedMorningMin += FEED_MORNING_AFTER_SUNRISE_MIN;
  }

  feedAfternoonHr -= FEED_AFTERNOON_BEFORE_SUNSET_HR;
  feedEveningHr   -= FEED_EVENING_BEFORE_SUNSET_HR;
}

void scheduleDailyAlarms() {
  Alarm.alarmRepeat(1, 0, 0, scheduleTodayAlarms);
  Alarm.alarmRepeat(FEED_MID_DAY_HR, FEED_MID_DAY_MIN, 0, feed);
}

void scheduleTodayAlarms() {
  calculateAlarmTimes();
  float nowHr = hourMinuteToHour(hour(), minute());

  if (nowHr < hourMinuteToHour(openDoorHr,      openDoorMin))      Alarm.alarmOnce(openDoorHr,      openDoorMin,      0, enableDoorOpening);
  if (nowHr < hourMinuteToHour(closeDoorHr,      closeDoorMin))     Alarm.alarmOnce(closeDoorHr,     closeDoorMin,     0, enableDoorClosing);
  if (nowHr < hourMinuteToHour(lightOnHr,        lightOnMin))       Alarm.alarmOnce(lightOnHr,       lightOnMin,       0, lightOn);
  if (nowHr < hourMinuteToHour(lightOffHr,       lightOffMin))      Alarm.alarmOnce(lightOffHr,      lightOffMin,      0, lightOff);
  if (nowHr < hourMinuteToHour(waterHr,          waterMin))         Alarm.alarmOnce(waterHr,         waterMin,         0, water);
  if (nowHr < hourMinuteToHour(feedMorningHr,    feedMorningMin))   Alarm.alarmOnce(feedMorningHr,   feedMorningMin,   0, feed);
  if (nowHr < hourMinuteToHour(feedAfternoonHr,  feedAfternoonMin)) Alarm.alarmOnce(feedAfternoonHr, feedAfternoonMin, 0, feed);
  if (nowHr < hourMinuteToHour(feedEveningHr,    feedEveningMin))   Alarm.alarmOnce(feedEveningHr,   feedEveningMin,   0, feed);
}


// ═════════════════════════════════════════════════════════════════════════════
// INIT
// ═════════════════════════════════════════════════════════════════════════════
void setupOutputPins() {
  pinMode(DOOR_RELAY_OPEN_PIN,      OUTPUT);
  pinMode(DOOR_RELAY_CLOSE_PIN,     OUTPUT);
  pinMode(FEEDER_RELAY_PIN,         OUTPUT);
  pinMode(LIGHT_RELAY_PIN,          OUTPUT);
  pinMode(FAN_RELAY_PIN,            OUTPUT);
  pinMode(WATER_SOLENOID_RELAY_PIN, OUTPUT);
}

void initState() {
  fanOff(); feedOff(); waterOff();
  calculateAlarmTimes();

  float nowHr = hourMinuteToHour(hour(), minute());

  if (nowHr > hourMinuteToHour(lightOnHr, lightOnMin) && nowHr < hourMinuteToHour(lightOffHr, lightOffMin)) {
    if (SERIAL_DEBUG) Serial.println(F("light init on"));
    lightOn();
  } else {
    if (SERIAL_DEBUG) Serial.println(F("light init off"));
    lightOff();
  }

  if (nowHr > hourMinuteToHour(openDoorHr, openDoorMin) && nowHr < hourMinuteToHour(closeDoorHr, closeDoorMin)) {
    if (SERIAL_DEBUG) Serial.println(F("door init open"));
    enableDoorOpening();
  } else {
    if (SERIAL_DEBUG) Serial.println(F("door init close"));
    enableDoorClosing();
  }
}


// ═════════════════════════════════════════════════════════════════════════════
// BLE – READ & DISPATCH
// ═════════════════════════════════════════════════════════════════════════════
void readBLESerial() {
  bleReceive = ""; bleCmd = ""; bleStr1 = ""; bleStr2 = "";

  unsigned long t = millis();
  while (millis() - t < 20) {
    if (!Serial1.available()) { delay(1); continue; }
    char c = (char)Serial1.read();
    bleReceive += c;

    if (bleReceive.startsWith("OK+CONN")) {
      bleConnected = true;
      if (SERIAL_DEBUG) Serial.println(F("[BLE] Connected"));
      sendBLEStatus();
      bleReceive = "";
      return;
    }
    if (bleReceive.startsWith("OK+LOST")) {
      bleConnected = false;
      if (SERIAL_DEBUG) Serial.println(F("[BLE] Disconnected"));
      bleReceive = "";
      return;
    }
  }

  if (SERIAL_DEBUG && bleReceive.length() > 0) {
    Serial.print(F("[BLE RX] ")); Serial.println(bleReceive);
  }

  if (bleReceive.indexOf('?') != 0) return;

  int eqIdx = bleReceive.indexOf('=');
  int scIdx = bleReceive.indexOf(';');

  if (eqIdx > 0) {
    bleCmd = bleReceive.substring(1, eqIdx);
    if (scIdx > eqIdx) {
      bleStr1 = bleReceive.substring(eqIdx + 1, scIdx);
      bleStr2 = bleReceive.substring(scIdx + 1);
      bleStr2.trim();
    } else {
      bleStr1 = bleReceive.substring(eqIdx + 1);
      bleStr1.trim();
    }
  } else {
    bleCmd = bleReceive.substring(1);
    bleCmd.trim();
  }

  executeBLECommand();
}

void executeBLECommand() {
  if (SERIAL_DEBUG) {
    Serial.print(F("BLE cmd=")); Serial.print(bleCmd);
    Serial.print(F(" s1="));    Serial.println(bleStr1);
  }

  if (bleCmd == "water") {
    if (bleStr1 == "0") { manualWater = false; waterOff(); sendBLE("OK+water=off"); }
    else                { manualWater = true;  water();    sendBLE("OK+water=on");  }
    delay(POST_BLE_HANDLE_DELAY_MS);

  } else if (bleCmd == "light") {
    if      (bleStr1 == "toggle") { toggleLight(); sendBLE(isLightOn() ? "OK+light=on" : "OK+light=off"); }
    else if (bleStr1 == "1")      { lightOn();     sendBLE("OK+light=on"); }
    else                          { lightOff();    sendBLE("OK+light=off"); }
    delay(POST_BLE_HANDLE_DELAY_MS);

  } else if (bleCmd == "feed") {
    feed();
    sendBLE("OK+feed=on");
    delay(POST_BLE_HANDLE_DELAY_MS);

  } else if (bleCmd == "door") {
    if      (bleStr1 == "open")  { enableDoorOpening(); sendBLE("OK+door=opening"); }
    else if (bleStr1 == "close") { enableDoorClosing(); sendBLE("OK+door=closing"); }
    else                         { sendBLE("ERR+door:use open|close"); }
    delay(POST_BLE_HANDLE_DELAY_MS);

  } else if (bleCmd == "fan") {
    if      (bleStr1 == "toggle") { toggleFan(); sendBLE(isFanOn() ? "OK+fan=on" : "OK+fan=off"); }
    else if (bleStr1 == "1")      { fanOn();     sendBLE("OK+fan=on"); }
    else                          { fanOff();    sendBLE("OK+fan=off"); }
    delay(POST_BLE_HANDLE_DELAY_MS);

  } else if (bleCmd == "status") {
    sendBLEStatus();

  } else if (bleCmd == "bleinfo") {
    char addr[20] = "?", name[16] = "?";
    uint8_t batt = 0;
    int8_t  rssi = 0;
    ble.queryAddr(addr, sizeof(addr));
    ble.queryName(name, sizeof(name));
    ble.queryBatt(batt);
    ble.queryRSSI(rssi);
    String info = "OK+bleinfo:addr=";
    info += addr; info += ";name="; info += name;
    info += ";batt="; info += batt; info += ";rssi="; info += rssi;
    sendBLE(info);

  } else if (bleCmd == "pinmode") {
    pinMode(bleStr1.toInt(), bleStr2.toInt());
    sendBLE("OK+pinmode");

  } else if (bleCmd == "digitalwrite") {
    pinMode(bleStr1.toInt(), OUTPUT);
    digitalWrite(bleStr1.toInt(), bleStr2.toInt());
    sendBLE("OK+digitalwrite");

  } else if (bleCmd == "digitalread") {
    sendBLE("OK+dread=" + String(digitalRead(bleStr1.toInt())));

  } else if (bleCmd == "analogread") {
    sendBLE("OK+aread=" + String(analogRead(bleStr1.toInt())));

  } else {
    sendBLE("ERR+unknown:" + bleCmd);
  }
}

void sendBLEStatus() {
  String s = "OK+status:{";
  s += "\"light\":";      s += isLightOn()       ? "1" : "0";
  s += ",\"fan\":";       s += isFanOn()          ? "1" : "0";
  s += ",\"feed\":";      s += isFeedOn()         ? "1" : "0";
  s += ",\"water\":";     s += isWaterOn()        ? "1" : "0";
  s += ",\"doorOpen\":";  s += isDoorOpening()    ? "1" : "0";
  s += ",\"doorClose\":"; s += isDoorClosing()    ? "1" : "0";
  s += ",\"doorAtOpen\":";  s += isDoorOpenStop()  ? "1" : "0";
  s += ",\"doorAtClose\":"; s += isDoorCloseStop() ? "1" : "0";
  if (!airTempError) { s += ",\"tempF\":"; s += airTemp; }
  else               { s += ",\"tempF\":null"; }
  s += ",\"time\":\"";
  s += hour(); s += ":";
  if (minute() < 10) s += "0";
  s += minute();
  s += "\",\"ble\":1}";
  sendBLE(s);
}

void sendBLE(String data) {
  Serial1.println(data);
  if (SERIAL_DEBUG) { Serial.print(F("[BLE TX] ")); Serial.println(data); }
}


// ═════════════════════════════════════════════════════════════════════════════
// HANDLER FUNCTIONS
// ═════════════════════════════════════════════════════════════════════════════
void handleFloat() {
  if (digitalRead(WATER_FLOAT_SWITCH_PIN)) { waterOn(); }
  else if (!manualWater)                   { waterOff(); }
}

void handleControls() {
  if (isLightToggleButton()) {
    if (SERIAL_DEBUG) Serial.println(F("toggle light btn"));
    toggleLight();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
  if (isFeedButton()) {
    if (SERIAL_DEBUG) Serial.println(F("feed btn"));
    feed();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
  if (isWaterButton()) {
    if (SERIAL_DEBUG) Serial.println(F("water btn"));
    manualWater = true;
    water();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
  if (isDoorOpenSwitch()) {
    if (SERIAL_DEBUG) Serial.println(F("door open sw"));
    enableDoorOpening();
  }
  if (isDoorCloseSwitch()) {
    if (SERIAL_DEBUG) Serial.println(F("door close sw"));
    enableDoorClosing();
  }
}

void handleFan() {
  if (!airTempError) {
    if (airTemp >= FAN_ON_TEMP_F)  fanOn();
    if (airTemp <= FAN_OFF_TEMP_F) fanOff();
  }
}

void updateAirTemp() {
  airTempError = !updateTemp(&airTempOneWire, &airTemp, airTempError);
  if (airTempError || airTemp <= MIN_REASONABLE_AIR_TEMP_F || airTemp >= MAX_REASONABLE_AIR_TEMP_F) {
    airTempError = true;
  }
}

void handleDoor() {
  if (isDoorClosing() && isDoorCloseStop()) disableDoor();
  if (isDoorOpening() && isDoorOpenStop())  disableDoor();
}

void handleDurations() {
  if (isFeedOn()) {
    feedOnTimeSec += now() - lastLoopSec;
    if (SERIAL_DEBUG) { Serial.print(F("feeding (")); Serial.print(feedOnTimeSec); Serial.println(F(" sec)")); }
    if (feedOnTimeSec >= FEED_DURATION_SEC) feedOff();
  }
  if (isWaterOn()) {
    waterOnTimeSec += now() - lastLoopSec;
    if (SERIAL_DEBUG) { Serial.print(F("watering (")); Serial.print(waterOnTimeSec); Serial.println(F(" sec)")); }
    if (waterOnTimeSec >= WATER_DURATION_SEC) { waterOff(); manualWater = false; }
  }
}

void serialDebugState() {
  Serial.print(F("airTemp: "));
  if (!airTempError) Serial.println(airTemp); else Serial.println(F("ERROR"));
  Serial.print(F("time: "));
  Serial.print(year());  Serial.print('/');
  Serial.print(month()); Serial.print('/');
  Serial.print(day());   Serial.print(' ');
  Serial.print(hour());  Serial.print(':');
  Serial.print(minute());Serial.print(':');
  Serial.println(second());
}


// ═════════════════════════════════════════════════════════════════════════════
// ACTUATOR METHODS
// ═════════════════════════════════════════════════════════════════════════════

// ── Feed ─────────────────────────────────────────────────────────────────────
void feed()    { feedOnTimeSec = 0; feedOn(); }
void feedOn()  { digitalWrite(FEEDER_RELAY_PIN, LOW); }
void feedOff() { digitalWrite(FEEDER_RELAY_PIN, HIGH); }
boolean isFeedOn()     { return digitalRead(FEEDER_RELAY_PIN) == LOW; }
boolean isFeedButton() { return digitalRead(FEED_BUTTON_PIN)  == HIGH; }

// ── Water ────────────────────────────────────────────────────────────────────
void water()    { waterOnTimeSec = 0; waterOn(); }
void waterOn()  { digitalWrite(WATER_SOLENOID_RELAY_PIN, HIGH); }
void waterOff() { digitalWrite(WATER_SOLENOID_RELAY_PIN, LOW); }
boolean isWaterOn()     { return digitalRead(WATER_SOLENOID_RELAY_PIN) == HIGH; }
boolean isWaterButton() { return digitalRead(WATER_BUTTON_PIN) == HIGH; }

// ── Light ────────────────────────────────────────────────────────────────────
void toggleLight() { isLightOn() ? lightOff() : lightOn(); }
void lightOn()     { digitalWrite(LIGHT_RELAY_PIN, LOW); }
void lightOff()    { digitalWrite(LIGHT_RELAY_PIN, HIGH); }
boolean isLightOn()           { return digitalRead(LIGHT_RELAY_PIN)         == LOW; }
boolean isLightToggleButton() { return digitalRead(LIGHT_TOGGLE_BUTTON_PIN) == HIGH; }

// ── Fan ──────────────────────────────────────────────────────────────────────
void toggleFan() { isFanOn() ? fanOff() : fanOn(); }
void fanOn()     { digitalWrite(FAN_RELAY_PIN, LOW); }
void fanOff()    { digitalWrite(FAN_RELAY_PIN, HIGH); }
boolean isFanOn() { return digitalRead(FAN_RELAY_PIN) == LOW; }

// ── Door ─────────────────────────────────────────────────────────────────────
boolean isDoorOpenSwitch()  { return digitalRead(DOOR_OPEN_SWITCH_PIN); }
boolean isDoorCloseSwitch() { return digitalRead(DOOR_CLOSE_SWITCH_PIN); }
boolean isDoorClosing()     { return digitalRead(DOOR_RELAY_CLOSE_PIN) == HIGH; }
boolean isDoorOpening()     { return digitalRead(DOOR_RELAY_OPEN_PIN)  == HIGH; }
boolean isDoorInMiddle()    { return !isDoorOpenStop() && !isDoorCloseStop(); }
boolean isDoorCloseStop()   { return analogRead(DOOR_CLOSE_STOP_PIN) > 500; }
boolean isDoorOpenStop()    { return analogRead(DOOR_OPEN_STOP_PIN)  > 500; }

void enableDoorClosing() {
  if (!isDoorCloseStop()) {
    digitalWrite(DOOR_RELAY_OPEN_PIN,  LOW);
    digitalWrite(DOOR_RELAY_CLOSE_PIN, HIGH);
  }
}
void enableDoorOpening() {
  if (!isDoorOpenStop()) {
    digitalWrite(DOOR_RELAY_CLOSE_PIN, LOW);
    digitalWrite(DOOR_RELAY_OPEN_PIN,  HIGH);
  }
}
void disableDoor() {
  digitalWrite(DOOR_RELAY_OPEN_PIN,  LOW);
  digitalWrite(DOOR_RELAY_CLOSE_PIN, LOW);
}


// ═════════════════════════════════════════════════════════════════════════════
// UTILITY & CONVERSION
// ═════════════════════════════════════════════════════════════════════════════
boolean updateTemp(OneWire* oneWireTherm, float* fltTemp, boolean previousError) {
  byte thermAddr[8], data[12];
  oneWireTherm->reset_search();
  oneWireTherm->search(thermAddr);
  if (OneWire::crc8(thermAddr, 7) != thermAddr[7]) return false;
  if (!oneWireTherm->reset()) return false;
  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0x44, 1);
  if (previousError) delay(1000);
  if (!oneWireTherm->reset()) return false;
  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0xBE);
  for (int i = 0; i < 9; i++) data[i] = oneWireTherm->read();
  *fltTemp = getTemperature(data[0], data[1]);
  return true;
}

float convertCeliusToFahrenheit(float c) { return (c * 1.8f) + 32.0f; }
float convertFahrenheitToCelius(float f)  { return (f - 32.0f) * 0.555555556f; }
float hourMinuteToHour(int hr, int min)   { return hr + min / 60.0f; }

float getTemperature(int lowByte, int highByte) {
  int intHexTempReading = (highByte << 8) + lowByte;
  int boolSign = intHexTempReading & 0x8000;
  if (boolSign) intHexTempReading = (intHexTempReading ^ 0xffff) + 1;
  int intTempReadingBeforeSplit = (int)(6.25f * intHexTempReading);
  int preDecimal     = intTempReadingBeforeSplit / 100;
  int intPostDecimal = intTempReadingBeforeSplit % 100;
  float fltTemp = preDecimal + (intPostDecimal < 10 ? intPostDecimal / 1000.0f : intPostDecimal / 100.0f);
  if (boolSign) fltTemp = -fltTemp;
  return convertCeliusToFahrenheit(fltTemp);
}


// ═════════════════════════════════════════════════════════════════════════════
// SETUP  ←  paling bawah (ArduinoDroid / strict C++ forward-decl compliance)
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  if (SERIAL_DEBUG) {
    Serial.begin(57600);
    Serial.println(F("=== Coop BLE (CC2540) ==="));
  }

  Wire.begin();

  ble.begin("COOP_BLE", "123456", '!');
  if (SERIAL_DEBUG) Serial.println(F("BLE init..."));
  delay(1500);

  setSyncProvider(RTC.get);
  coopTimeLord.TimeZone(TIMEZONE * 60);
  coopTimeLord.Position(LATITUDE, LONGITUDE);

  lastLoopSec = now();

  setupOutputPins();
  initState();
  scheduleDailyAlarms();
  scheduleTodayAlarms();

  if (SERIAL_DEBUG) Serial.println(F("Setup done."));
}


// ═════════════════════════════════════════════════════════════════════════════
// LOOP  ←  paling bawah
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  if (SERIAL_DEBUG) serialDebugState();

  ble.messageHandler();

  if (Serial1.available()) readBLESerial();

  handleDoor();
  updateAirTemp();
  handleFan();
  handleDurations();
  handleControls();
  handleFloat();

  Alarm.delay(0);
  lastLoopSec = now();
}
