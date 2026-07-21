// The MIT License (MIT)
// Copyright (c) 2017 Simon Kirchner (cyborg.simon@gmail.com)
// Updated 2026 - CC2540 / HM-10 BLE integration for Arduino Mega (HardwareSerial)

#include "HM_10_BLE.h"

// ─── Constructor ─────────────────────────────────────────────────────────────
HM_10_BLE::HM_10_BLE(HardwareSerial& serial) : _serial(serial) {
}

// ─── begin (simple) ──────────────────────────────────────────────────────────
void HM_10_BLE::begin(char delimiter) {
  begin("HM10_BLE", "123456", delimiter);
}

// ─── begin (full) ────────────────────────────────────────────────────────────
void HM_10_BLE::begin(const char* name, const char* pass, char delimiter) {
  _serial.begin(HM_10_BLE_BAUDRATE);

#if HM_10_BLE_DEBUG
  Serial.begin(9600);
  Serial.println(F("[HM10] init CC2540"));
#endif

  _messageDelimiter = delimiter;

  delay(200); // let module settle after power-on

  // Configure CC2540 defaults
  atCommand("IMME", "0");    // work immediately
  atCommand("ROLE", "0");    // peripheral (server)
  atCommand("MODE", "0");    // transparent transmission
  atCommand("NOTI", "1");    // notify connect/disconnect
  atCommand("TYPE", "0");    // no PIN required (change to 3 if you need PIN auth)
  atCommand("NAME", name);
  // Note: CC2540 uses AT+PIN for set but AT+PASS? for query
  char pinCmd[14];
  snprintf(pinCmd, sizeof(pinCmd), "AT+PIN%s", pass);
  _queueATCommand(pinCmd);
  atCommand("RESET");        // apply settings
}

// ─── AT queue helpers ────────────────────────────────────────────────────────
void HM_10_BLE::_queueATCommand(const char* cmd) {
  uint8_t next = (_atTail + 1) % AT_QUEUE_SIZE;
  if (next == _atHead) {
#if HM_10_BLE_DEBUG
    Serial.println(F("[HM10] AT queue full!"));
#endif
    return;
  }
  strncpy(_atQueue[_atTail], cmd, AT_CMD_MAXLEN - 1);
  _atQueue[_atTail][AT_CMD_MAXLEN - 1] = '\0';
  _atTail = next;

#if HM_10_BLE_DEBUG
  Serial.print(F("[HM10] queue: "));
  Serial.println(cmd);
#endif
}

bool HM_10_BLE::_atQueueEmpty() {
  return _atHead == _atTail;
}

// ─── atCommand overloads ─────────────────────────────────────────────────────
void HM_10_BLE::atCommand() {
  _queueATCommand("AT");
}

void HM_10_BLE::atCommand(const char* cmd) {
  char buf[AT_CMD_MAXLEN];
  snprintf(buf, sizeof(buf), "AT+%s", cmd);
  _queueATCommand(buf);
}

void HM_10_BLE::atCommand(const char* cmd, const char* param) {
  char buf[AT_CMD_MAXLEN];
  snprintf(buf, sizeof(buf), "AT+%s%s", cmd, param);
  _queueATCommand(buf);
}

// ─── Internal raw query (blocking, used by convenience wrappers) ──────────────
bool HM_10_BLE::_sendQuery(const char* cmd, char* buf, uint8_t len) {
  // flush
  while (_serial.available()) _serial.read();

  _serial.print(cmd);

  unsigned long t = millis();
  uint8_t idx = 0;
  while (millis() - t < HM_10_AT_TIMEOUT) {
    if (_serial.available()) {
      char c = (char)_serial.read();
      if (idx < len - 1) buf[idx++] = c;
    }
  }
  buf[idx] = '\0';
  return idx > 0;
}

bool HM_10_BLE::_sendSet(const char* cmd) {
  char tmp[AT_CMD_MAXLEN];
  _sendQuery(cmd, tmp, sizeof(tmp));
  return (strstr(tmp, "OK") != nullptr);
}

// ─── Convenience wrappers ─────────────────────────────────────────────────────

bool HM_10_BLE::queryVersion(char* buf, uint8_t len) {
  return _sendQuery("AT+VERR?", buf, len);
}

bool HM_10_BLE::queryAddr(char* buf, uint8_t len) {
  char tmp[32];
  if (!_sendQuery("AT+ADDR?", tmp, sizeof(tmp))) return false;
  // response: OK+ADDR:XXXXXXXXXXXX
  const char* p = strstr(tmp, "ADDR:");
  if (!p) return false;
  strncpy(buf, p + 5, len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool HM_10_BLE::queryBatt(uint8_t& pct) {
  char tmp[24];
  if (!_sendQuery("AT+BATT?", tmp, sizeof(tmp))) return false;
  // response: OK+BATT:XXX
  const char* p = strstr(tmp, "BATT:");
  if (!p) return false;
  pct = (uint8_t)atoi(p + 5);
  return true;
}

bool HM_10_BLE::queryRSSI(int8_t& rssi) {
  char tmp[24];
  if (!_sendQuery("AT+RSSI?", tmp, sizeof(tmp))) return false;
  const char* p = strstr(tmp, "RSSI:");
  if (!p) return false;
  rssi = (int8_t)atoi(p + 5);
  return true;
}

bool HM_10_BLE::queryName(char* buf, uint8_t len) {
  char tmp[32];
  if (!_sendQuery("AT+NAME?", tmp, sizeof(tmp))) return false;
  // response: OK+NAME:xxxxxxxx
  const char* p = strstr(tmp, "NAME:");
  if (!p) return false;
  strncpy(buf, p + 5, len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool HM_10_BLE::setName(const char* name) {
  char cmd[AT_CMD_MAXLEN];
  snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);
  return _sendSet(cmd);
}

bool HM_10_BLE::setPass(const char* pass) {
  // CC2540 uses AT+PIN to SET, AT+PASS? to QUERY
  char cmd[AT_CMD_MAXLEN];
  snprintf(cmd, sizeof(cmd), "AT+PIN%s", pass);
  return _sendSet(cmd);
}

bool HM_10_BLE::setBaud(uint8_t baud) {
  char b[2] = { (char)('0' + baud), '\0' };
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+BAUD%s", b);
  return _sendSet(cmd);
}

bool HM_10_BLE::setRole(uint8_t role) {
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+ROLE%u", role);
  return _sendSet(cmd);
}

bool HM_10_BLE::setMode(uint8_t mode) {
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+MODE%u", mode);
  return _sendSet(cmd);
}

bool HM_10_BLE::setPower(uint8_t pwr) {
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+POWE%u", pwr);
  return _sendSet(cmd);
}

bool HM_10_BLE::setType(uint8_t type) {
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+TYPE%u", type);
  return _sendSet(cmd);
}

bool HM_10_BLE::setUUID(const char* uuid) {
  char cmd[AT_CMD_MAXLEN];
  snprintf(cmd, sizeof(cmd), "AT+UUID%s", uuid);
  return _sendSet(cmd);
}

bool HM_10_BLE::setChar(const char* chr) {
  char cmd[AT_CMD_MAXLEN];
  snprintf(cmd, sizeof(cmd), "AT+CHAR%s", chr);
  return _sendSet(cmd);
}

bool HM_10_BLE::setNotify(bool on) {
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+NOTI%u", on ? 1 : 0);
  return _sendSet(cmd);
}

bool HM_10_BLE::setImme(uint8_t mode) {
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "AT+IMME%u", mode);
  return _sendSet(cmd);
}

bool HM_10_BLE::clearLastAddr() { return _sendSet("AT+CLEAR"); }

bool HM_10_BLE::connectLast() {
  char tmp[32];
  _sendQuery("AT+CONNL", tmp, sizeof(tmp));
  return (strstr(tmp, "OK+CONNL") != nullptr || strstr(tmp, "OK+CONN") != nullptr);
}

bool HM_10_BLE::connectAddr(const char* addr) {
  char cmd[AT_CMD_MAXLEN];
  snprintf(cmd, sizeof(cmd), "AT+CON%s", addr);
  char tmp[32];
  _sendQuery(cmd, tmp, sizeof(tmp));
  return (strstr(tmp, "OK+CONNA") != nullptr);
}

bool HM_10_BLE::sleep()  { return _sendSet("AT+SLEEP"); }
bool HM_10_BLE::start()  { return _sendSet("AT+START"); }
bool HM_10_BLE::reset()  { return _sendSet("AT+RESET"); }
bool HM_10_BLE::renew()  { return _sendSet("AT+RENEW"); }

// ─── atHandler (non-blocking queue processor) ────────────────────────────────
bool HM_10_BLE::atHandler() {
  if (_atQueueEmpty()) return false;
  if (_waitForMessage && !_waitForATCommand) return false;

  if (!_waitForATCommand) {
    const char* cmd = _atQueue[_atHead];

#if HM_10_BLE_DEBUG
    Serial.print(F("[HM10] send: "));
    Serial.println(cmd);
#endif

    _serial.print(cmd);
    _atSentAt = millis();
    _waitForATCommand = true;
    _atAnswer = "";
    return true;
  }

  // Read available chars
  while (_serial.available()) {
    _atAnswer += (char)_serial.read();
  }

  // Timeout → treat as done
  if (millis() - _atSentAt >= HM_10_AT_TIMEOUT) {
    _handleATAnswer();
  }
  return true;
}

void HM_10_BLE::_handleATAnswer() {
#if HM_10_BLE_DEBUG
  Serial.print(F("[HM10] answer: "));
  Serial.println(_atAnswer);
#endif
  _atAnswer = "";
  _waitForATCommand = false;
  _atHead = (_atHead + 1) % AT_QUEUE_SIZE;
}

// ─── messageHandler (non-blocking incoming data) ─────────────────────────────
bool HM_10_BLE::messageHandler() {
  // Process pending AT commands first
  if (!_atQueueEmpty() && !_waitForMessage) {
    atHandler();
    return false;
  }
  if (!_serial.available()) return true;

  char c = (char)_serial.read();

  // CC2540 sends "OK+CONN" / "OK+LOST" notifications — handle or ignore
  if (!_waitForMessage) {
    // start accumulating only after first meaningful char
    if (c == '\r' || c == '\n') return true;
  }

  if (c == _messageDelimiter) {
    _handleMessage();
    return true;
  }

  _waitForMessage = true;
  _message += c;
  return true;
}

void HM_10_BLE::_handleMessage() {
  char msg[_message.length() + 1];
  _message.toCharArray(msg, _message.length() + 1);
  processMessage(msg);

#if HM_10_BLE_DEBUG
  Serial.print(F("[HM10] msg: "));
  Serial.println(msg);
#endif

  _message = "";
  _waitForMessage = false;
}

// ─── processMessage (override in subclass or main sketch) ────────────────────
void HM_10_BLE::processMessage(char* msg) {
#if HM_10_BLE_DEBUG
  Serial.print(F("[HM10] process: "));
  Serial.println(msg);
#endif
}

// ─── writeMessage ─────────────────────────────────────────────────────────────
void HM_10_BLE::writeMessage(const char* msg) {
  _serial.print(msg);
  _serial.write(_messageDelimiter);
}

// ─── writeRaw ────────────────────────────────────────────────────────────────
void HM_10_BLE::writeRaw(const char* data) {
  _serial.print(data);
}
