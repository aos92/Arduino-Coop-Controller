// The MIT License (MIT)
// Copyright (c) 2017 Simon Kirchner (cyborg.simon@gmail.com)
// Updated 2026 - CC2540 / HM-10 BLE integration for Arduino Mega (HardwareSerial)

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

#ifndef _HM_10_BLE_H_
#define _HM_10_BLE_H_

#include "Arduino.h"

// ─── Debug & Baudrate ────────────────────────────────────────────────────────
#define HM_10_BLE_DEBUG    false
#define HM_10_BLE_BAUDRATE 9600

// ─── AT Command timeout (ms) ─────────────────────────────────────────────────
#define HM_10_AT_TIMEOUT   500

// ─── Default UUID & Characteristic (CC2540 defaults) ─────────────────────────
#define HM_10_DEFAULT_UUID "FFE0"
#define HM_10_DEFAULT_CHAR "FFE1"

// ─── Role ─────────────────────────────────────────────────────────────────────
#define HM_10_ROLE_PERIPHERAL 0
#define HM_10_ROLE_CENTRAL    1

// ─── Bond / Auth Type ────────────────────────────────────────────────────────
// TYPE0 = No PIN, TYPE1 = Bond no PIN, TYPE2 = Bond with PIN, TYPE3 = Bond+PIN
#define HM_10_TYPE_NOPIN   0
#define HM_10_TYPE_BOND    1
#define HM_10_TYPE_BONDPIN 2
#define HM_10_TYPE_AUTH    3

// ─── Work Mode ───────────────────────────────────────────────────────────────
#define HM_10_MODE_TRANSMISSION 0
#define HM_10_MODE_PIO_COLLECT  1
#define HM_10_MODE_REMOTE_CTRL  2

// ─── TX Power ────────────────────────────────────────────────────────────────
#define HM_10_POWE_NEG23DBM 0
#define HM_10_POWE_NEG6DBM  1
#define HM_10_POWE_0DBM     2
#define HM_10_POWE_6DBM     3

// ─── Baud rates ──────────────────────────────────────────────────────────────
// 0=9600, 1=19200, 2=38400, 3=57600, 4=115200, 5=4800, 6=2400, 7=1200, 8=230400

class HM_10_BLE {
public:
  // Constructor: pass a HardwareSerial port (Serial1/Serial2/Serial3 on Mega)
  HM_10_BLE(HardwareSerial& serial);

  // ── Init ──────────────────────────────────────────────────────────────────
  // Simple init with message delimiter
  void begin(char delimiter);

  // Full init: name, 6-digit PIN, delimiter
  void begin(const char* name, const char* pass, char delimiter);

  // ── AT Commands (CC2540 full set) ─────────────────────────────────────────
  void atCommand();                              // AT  (test)
  void atCommand(const char* cmd);              // AT+CMD
  void atCommand(const char* cmd, const char* param); // AT+CMDparam

  // Convenience wrappers
  bool  queryVersion(char* buf, uint8_t len);   // AT+VERR?
  bool  queryAddr(char* buf, uint8_t len);      // AT+ADDR?
  bool  queryBatt(uint8_t& pct);               // AT+BATT?
  bool  queryRSSI(int8_t& rssi);               // AT+RSSI?
  bool  queryName(char* buf, uint8_t len);      // AT+NAME?
  bool  setName(const char* name);             // AT+NAMExxxx
  bool  setPass(const char* pass);             // AT+PINxxxxxx (PIN command)
  bool  setBaud(uint8_t baud);                 // AT+BAUDn
  bool  setRole(uint8_t role);                 // AT+ROLEn
  bool  setMode(uint8_t mode);                 // AT+MODEn
  bool  setPower(uint8_t pwr);                 // AT+POWEn
  bool  setType(uint8_t type);                 // AT+TYPEn
  bool  setUUID(const char* uuid);             // AT+UUIDxxxx
  bool  setChar(const char* chr);              // AT+CHARxxxx
  bool  setNotify(bool on);                    // AT+NOTIn
  bool  setImme(uint8_t mode);                 // AT+IMMEn  0=work now,1=wait
  bool  clearLastAddr();                       // AT+CLEAR
  bool  connectLast();                         // AT+CONNL
  bool  connectAddr(const char* addr);         // AT+CONxxxxxxxx
  bool  sleep();                               // AT+SLEEP
  bool  start();                               // AT+START
  bool  reset();                               // AT+RESET
  bool  renew();                               // AT+RENEW (factory reset)

  // ── Loop Handlers ─────────────────────────────────────────────────────────
  // Call one of these in loop():
  bool atHandler();       // handles queued AT commands
  bool messageHandler();  // handles incoming BLE data messages

  // ── Send ──────────────────────────────────────────────────────────────────
  void writeMessage(const char* msg);          // send + delimiter
  void writeRaw(const char* data);             // send raw bytes

  // ── Override to process incoming messages ────────────────────────────────
  virtual void processMessage(char* msg);

private:
  HardwareSerial& _serial;

  // AT queue (simple ring buffer, no external lib dependency)
  static const uint8_t AT_QUEUE_SIZE = 16;
  static const uint8_t AT_CMD_MAXLEN = 48;
  char  _atQueue[AT_QUEUE_SIZE][AT_CMD_MAXLEN];
  uint8_t _atHead = 0;
  uint8_t _atTail = 0;

  bool   _waitForATCommand = false;
  String _atAnswer;
  unsigned long _atSentAt = 0;

  bool   _waitForMessage = false;
  char   _messageDelimiter = '!';
  String _message;

  void   _queueATCommand(const char* cmd);
  bool   _atQueueEmpty();
  void   _handleATAnswer();
  void   _handleMessage();

  // Raw query: send cmd, wait up to HM_10_AT_TIMEOUT ms, copy response
  bool   _sendQuery(const char* cmd, char* buf, uint8_t len);
  bool   _sendSet(const char* cmd);
};

#endif /* _HM_10_BLE_H_ */
