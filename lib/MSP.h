// =========================================
// MSP Protocol Library for Betaflight
// MultiWii Serial Protocol implementation
// For OpenDrones Nav Module
// =========================================
//
// MSP Packet Format:
//   $M< [size] [cmd] [payload...] [checksum]
//   $M> [size] [cmd] [payload...] [checksum]
//
// $M = header, < = to FC, > = from FC
// checksum = XOR of size + cmd + all payload bytes
//
// UART wiring:
//   Nav TX (D9/GPIO8) -> FC RX
//   Nav RX (D8/GPIO7) -> FC TX
//   GND -> GND

#ifndef MSP_LIB_H
#define MSP_LIB_H

#include <Arduino.h>

// MSP Command Codes
#define MSP_API_VERSION         1
#define MSP_FC_VARIANT          2
#define MSP_FC_VERSION          3
#define MSP_BOARD_INFO          4
#define MSP_BUILD_INFO          5

#define MSP_STATUS              101
#define MSP_RAW_GPS             106
#define MSP_ATTITUDE            108
#define MSP_ALTITUDE            109
#define MSP_ANALOG              110
#define MSP_RC                  105
#define MSP_SET_RAW_RC          200
#define MSP_SET_PID             202
#define MSP_SET_RAW_GPS         201

// MSP direction markers
#define MSP_HEADER      '$'
#define MSP_DIR_TO_FC   'M'  // We send this
#define MSP_DIR_FROM_FC '>'  // FC sends this when responding

// RC channel constants
#define RC_CHANNEL_MIN     1000
#define RC_CHANNEL_CENTER  1500
#define RC_CHANNEL_MAX     2000
#define RC_CHANNELS        16

class MSPLib {
public:
  MSPLib();

  // Initialize with Serial reference and pin assignments
  void begin(HardwareSerial &serial, int rxPin, int txPin, uint32_t baud = 115200);

  // Send MSP commands
  void sendCommand(uint8_t cmd, const uint8_t *payload = nullptr, uint8_t len = 0);
  void sendRequest(uint8_t cmd);

  // Read incoming MSP data (call in loop)
  bool readPacket();

  // High-level requests
  void requestAPIVersion();
  void requestStatus();
  void requestRC();
  void requestAttitude();
  void requestAltitude();

  // Send RC channel overrides (16 channels, 1000-2000 each)
  void setRawRC(const uint16_t channels[RC_CHANNELS]);
  // Send RC with specific channel overrides (rest stay at center)
  void setRawRC(uint8_t ch, uint16_t value);
  void setRawRCPassThrough(); // Send current read channels back (passthrough)

  // Parse response data (only valid after readPacket returns true)
  uint8_t getCommand() const { return _cmd; }
  uint8_t getPayloadSize() const { return _payloadLen; }
  const uint8_t* getPayload() const { return _payload; }

  // Convenience: parse specific response types
  bool parseAPIVersion(uint8_t &major, uint8_t &minor);
  bool parseFCVariant(char variant[5]);
  bool parseStatus(uint32_t &flags, uint16_t &cycleTime);
  bool parseRC(uint16_t channels[RC_CHANNELS]);
  bool parseAttitude(int16_t &roll, int16_t &pitch, int16_t &yaw);
  bool parseAltitude(int32_t &alt);

  // Connection state
  bool isConnected() const { return _connected; }
  void checkConnection();

  // Stats
  uint32_t packetsSent = 0;
  uint32_t packetsReceived = 0;
  uint32_t packetsFailed = 0;

private:
  HardwareSerial *_serial = nullptr;
  bool _connected = false;
  unsigned long _lastResponse = 0;

  // Parser state
  enum ParseState {
    WAIT_HEADER1,
    WAIT_HEADER2,
    WAIT_DIR,
    WAIT_SIZE,
    WAIT_CMD,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
  };
  ParseState _state = WAIT_HEADER1;

  uint8_t _cmd = 0;
  uint8_t _payloadLen = 0;
  uint8_t _payload[256];
  uint8_t _payloadIdx = 0;
  uint8_t _checksum = 0;

  uint8_t _calcChecksum(const uint8_t *data, uint8_t len);
};

#endif // MSP_LIB_H
