// =========================================
// MSP Protocol Library Implementation
// OpenDrones Nav Module
// =========================================

#include "MSP.h"

MSPLib::MSPLib() {}

void MSPLib::begin(HardwareSerial &serial, int rxPin, int txPin, uint32_t baud) {
  _serial = &serial;
  _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
}

// =========================================
// Send MSP packet to FC
// Format: $M< [size] [cmd] [payload...] [checksum]
// =========================================
void MSPLib::sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  if (!_serial) return;

  uint8_t ck = 0;

  _serial->write('$');
  _serial->write('M');
  _serial->write('<');
  _serial->write(len);
  _serial->write(cmd);
  ck ^= len;
  ck ^= cmd;

  for (uint8_t i = 0; i < len; i++) {
    _serial->write(payload[i]);
    ck ^= payload[i];
  }

  _serial->write(ck);
  _serial->flush();
  packetsSent++;
}

void MSPLib::sendRequest(uint8_t cmd) {
  sendCommand(cmd, nullptr, 0);
}

// =========================================
// Read and parse incoming MSP data
// Call this in loop() - returns true when complete packet received
// =========================================
bool MSPLib::readPacket() {
  if (!_serial) return false;

  while (_serial->available()) {
    uint8_t c = _serial->read();

    switch (_state) {
      case WAIT_HEADER1:
        if (c == '$') _state = WAIT_HEADER2;
        break;

      case WAIT_HEADER2:
        if (c == 'M') _state = WAIT_DIR;
        else _state = WAIT_HEADER1;
        break;

      case WAIT_DIR:
        if (c == '>') {  // Response from FC
          _state = WAIT_SIZE;
          _checksum = 0;
        } else {
          _state = WAIT_HEADER1;
        }
        break;

      case WAIT_SIZE:
        _payloadLen = c;
        _checksum = c;
        _state = WAIT_CMD;
        break;

      case WAIT_CMD:
        _cmd = c;
        _checksum ^= c;
        _payloadIdx = 0;
        if (_payloadLen == 0) {
          _state = WAIT_CHECKSUM;
        } else {
          _state = WAIT_PAYLOAD;
        }
        break;

      case WAIT_PAYLOAD:
        _payload[_payloadIdx++] = c;
        _checksum ^= c;
        if (_payloadIdx >= _payloadLen) {
          _state = WAIT_CHECKSUM;
        }
        break;

      case WAIT_CHECKSUM:
        _state = WAIT_HEADER1;
        if (c == _checksum) {
          packetsReceived++;
          _lastResponse = millis();
          _connected = true;
          return true;  // Valid packet!
        } else {
          packetsFailed++;
        }
        break;
    }
  }
  return false;
}

// =========================================
// High-level request functions
// =========================================
void MSPLib::requestAPIVersion() { sendRequest(MSP_API_VERSION); }
void MSPLib::requestStatus()     { sendRequest(MSP_STATUS); }
void MSPLib::requestRC()         { sendRequest(MSP_RC); }
void MSPLib::requestAttitude()   { sendRequest(MSP_ATTITUDE); }
void MSPLib::requestAltitude()   { sendRequest(MSP_ALTITUDE); }

// =========================================
// Send RC channel overrides (MSP_SET_RAW_RC)
// 16 channels, each 1000-2000 (1500 = center)
// =========================================
void MSPLib::setRawRC(const uint16_t channels[RC_CHANNELS]) {
  uint8_t payload[RC_CHANNELS * 2];
  for (int i = 0; i < RC_CHANNELS; i++) {
    payload[i * 2]     = channels[i] & 0xFF;        // Low byte
    payload[i * 2 + 1] = (channels[i] >> 8) & 0xFF; // High byte
  }
  sendCommand(MSP_SET_RAW_RC, payload, RC_CHANNELS * 2);
}

void MSPLib::setRawRC(uint8_t ch, uint16_t value) {
  uint16_t channels[RC_CHANNELS];
  for (int i = 0; i < RC_CHANNELS; i++) {
    channels[i] = RC_CHANNEL_CENTER;
  }
  channels[ch] = value;
  setRawRC(channels);
}

// =========================================
// Parse response data
// =========================================
bool MSPLib::parseAPIVersion(uint8_t &major, uint8_t &minor) {
  if (_cmd != MSP_API_VERSION || _payloadLen < 3) return false;
  // payload[0] = MSP protocol version (always 0)
  major = _payload[1];
  minor = _payload[2];
  return true;
}

bool MSPLib::parseFCVariant(char variant[5]) {
  if (_cmd != MSP_FC_VARIANT || _payloadLen < 4) return false;
  memcpy(variant, _payload, 4);
  variant[4] = '\0';
  return true;
}

bool MSPLib::parseStatus(uint32_t &flags, uint16_t &cycleTime) {
  if (_cmd != MSP_STATUS || _payloadLen < 6) return false;
  cycleTime = _payload[0] | (_payload[1] << 8);
  // I2C errors (2 bytes) at [2,3]
  // sensors (2 bytes) at [4,5]
  flags = _payload[6] | (_payload[7] << 8) | (_payload[8] << 16) | (_payload[9] << 24);
  return true;
}

bool MSPLib::parseRC(uint16_t channels[RC_CHANNELS]) {
  if (_cmd != MSP_RC || _payloadLen < RC_CHANNELS * 2) return false;
  for (int i = 0; i < RC_CHANNELS; i++) {
    channels[i] = _payload[i * 2] | (_payload[i * 2 + 1] << 8);
  }
  return true;
}

bool MSPLib::parseAttitude(int16_t &roll, int16_t &pitch, int16_t &yaw) {
  if (_cmd != MSP_ATTITUDE || _payloadLen < 6) return false;
  roll  = (int16_t)(_payload[0] | (_payload[1] << 8));
  pitch = (int16_t)(_payload[2] | (_payload[3] << 8));
  yaw   = (int16_t)(_payload[4] | (_payload[5] << 8));
  return true;
}

bool MSPLib::parseAltitude(int32_t &alt) {
  if (_cmd != MSP_ALTITUDE || _payloadLen < 6) return false;
  alt = (int32_t)(_payload[0] | (_payload[1] << 8) | (_payload[2] << 16) | (_payload[3] << 24));
  return true;
}

// =========================================
// Connection management
// =========================================
void MSPLib::checkConnection() {
  if (_lastResponse > 0 && millis() - _lastResponse > 3000) {
    _connected = false;
  }
}
