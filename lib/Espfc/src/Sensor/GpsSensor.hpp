#include "GpsSensor.hpp"
#include "../Model.h"
#include "../SerialManager.h"
#include <Arduino.h>  // millis, Serial

namespace Espfc::Sensor {

GpsSensor::GpsSensor(Model& model) : _model(model) {
  _state = DETECT_BAUD;
  _counter = 0;
  _timeout = 0;
  _currentBaud = 0;
  _targetBaud = 0;
  _port = nullptr;
  _timer.reset();
}

int GpsSensor::begin(Device::SerialDevice* port, int baud) {
  if (!port) return -1;
  _port = port;
  _targetBaud = baud;
  _port->begin(baud);
  Serial.printf("[GPS] NEO-6M init: baud=%d, port=%d\n", baud, _model.config.gps.port);
  setState(DETECT_BAUD);
  return 0;
}

int GpsSensor::update() {
  if (!_port) return -1;

  uint32_t now = millis();
  while (_port->available()) {
    uint8_t c = _port->read();
    _counter++;

    // Route to parsers based on state
    if (_state <= SET_BAUD || _state == RECEIVE) {
      processNmea(c);
    }
    processUbx(c);  // UBX always (for config ACKs)

    handle();  // Advance FSM
  }

  // Timeout handling
  if (_timer.hasElapsed(_timeout)) {
    setState(_timeoutState);
  }

  return 0;
}

void GpsSensor::handle() {
  switch (_state) {
    case DETECT_BAUD:
      if (_nmeaParser.isValid()) {  // Valid NMEA? Baud OK
        _currentBaud = _targetBaud;
        Serial.printf("[GPS] Baud detected: %d\n", _currentBaud);
        setState(SET_BAUD, DISABLE_NMEA);
      } else if (_counter > 500 && _targetBaud == 9600) {
        // No data at 9600? Try 38400
        setBaud(38400);
        _counter = 0;
        _targetBaud = 38400;
      } else if (_counter > 1000) {
        Serial.println("[GPS] No baud detected - fallback NMEA");
        setState(RECEIVE);
      }
      break;

    case SET_BAUD:
      // Baud confirmed, stay at current
      send(Gps::UbxCfgPrt{ .portID = 1, .inProto1 = Gps::PROTO_UBX, .outProto1 = Gps::PROTO_UBX, .flags = 0 }, DISABLE_NMEA);
      break;

    case DISABLE_NMEA:
      // Disable all NMEA msgs
      for (uint8_t i = 0; i < 18; ++i) {  // GGA, RMC, etc.
        send(Gps::UbxCfgMsg{ .msgClass = Gps::NMEA_MSG_CLASS, .msgID = i, .rate = 0 }, DISABLE_NMEA);
      }
      setState(GET_VERSION);
      break;

    case GET_VERSION:
      send(Gps::UbxMonVer{}, ENABLE_UBX, DISABLE_NMEA);  // Poll version, timeout to NMEA disable
      break;

    case ENABLE_UBX:
      // Enable UBX NAV-PVT (pos/alt/vel/fix)
      send(Gps::UbxCfgMsg{ .msgClass = Gps::UBX_MSG_CLASS, .msgID = Gps::UBX_NAV_PVT, .rate = 1 }, ENABLE_NAV5);
      break;

    case ENABLE_NAV5:
      // Enable UBX NAV-SAT (sats count/quality)
      send(Gps::UbxCfgMsg{ .msgClass = Gps::UBX_MSG_CLASS, .msgID = Gps::UBX_NAV_SAT, .rate = 1 }, ENABLE_SBAS);
      break;

    case ENABLE_SBAS:
      // Enable SBAS for better accuracy
      send(Gps::UbxCfgSbas{ .usage = 1, .scanmode1 = 2, .scanmode2 = 2, .maxSBAS = 3 }, SET_RATE);
      break;

    case SET_RATE:
      // Set 5Hz update (200ms meas rate)
      send(Gps::UbxCfgRate{ .measRate = 200, .navRate = 1, .timeRef = 0 }, WAIT);
      break;

    case WAIT:
      if (_ubxParser.isMessageReady()) {
        onMessage();
        if (_state == RECEIVE) {  // Config done
          Serial.println("[GPS] UBX config complete - streaming NAV-PVT/SAT");
        }
      }
      break;

    case RECEIVE:
      // Ongoing parsing - onMessage handles population
      if (_ubxParser.isMessageReady()) {
        onMessage();
        _ubxParser.clear();
      }
      break;

    case ERROR:
      handleError();
      setState(RECEIVE);  // Fallback to receive mode
      break;
  }
}

bool GpsSensor::processUbx(uint8_t c) {
  if (_ubxParser.decode(c)) {
    _ubxMsg = _ubxParser.getMessage();
    return true;
  }
  return false;
}

void GpsSensor::processNmea(uint8_t c) {
  _nmeaParser.decode(c);
  if (_nmeaParser.isMessageReady()) {
    _nmeaMsg = _nmeaParser.getMessage();
    _nmeaParser.clear();

    // Fallback parsing (if UBX fails)
    if (_nmeaMsg.type == Gps::NmeaType::GGA) {
      _model.state.gps.latitude = _nmeaMsg.lat;
      _model.state.gps.longitude = _nmeaMsg.lon;
      _model.state.gps.altitude = _nmeaMsg.alt;
      _model.state.gps.fix = _nmeaMsg.fix;
      _model.state.gps.numSat = _nmeaMsg.sats;
      _model.state.gps.hdop = _nmeaMsg.hdop;
    } else if (_nmeaMsg.type == Gps::NmeaType::RMC) {
      _model.state.gps.speed = _nmeaMsg.speed;
      _model.state.gps.groundCourse = _nmeaMsg.course;
    }
  }
}

void GpsSensor::setBaud(int baud) {
  _port->end();
  delay(100);  // Settle
  _port->begin(baud);
  _currentBaud = baud;
  Serial.printf("[GPS] Switched to baud %d\n", baud);
}

void GpsSensor::onMessage() {
  switch (_ubxMsg.id) {
    case Gps::UBX_NAV_PVT:
      handleNavPvt();
      break;
    case Gps::UBX_NAV_SAT:
      handleNavSat();
      break;
    case Gps::UBX_MON_VER:
      handleVersion();
      break;
    case Gps::UBX_ACK_ACK:  // Config ACK
      Serial.printf("[GPS] ACK for msg 0x%02X\n", _ubxMsg.payload.ack.msg);
      setState(_ackState);  // Advance
      break;
    case Gps::UBX_ACK_NACK:
      Serial.printf("[GPS] NACK for msg 0x%02X - retry\n", _ubxMsg.payload.ack.msg);
      setState(ERROR);
      break;
    default:
      break;
  }
}

template<typename MsgType>
void GpsSensor::send(const MsgType m, State ackState, State timeoutState) {
  Gps::UbxFrame<MsgType> frame{m};
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&frame);
  _port->write(ptr, sizeof(frame));
  Serial.printf("[GPS] Sent UBX %s (len=%d)\n", typeid(MsgType).name(), sizeof(frame));
  setState(WAIT, ackState, timeoutState ? timeoutState : ERROR);
}

void GpsSensor::setState(State state, State ackState, State timeoutState) {
  _state = state;
  _ackState = ackState;
  _timeoutState = timeoutState;
  _timeout = millis() + (_state == DETECT_BAUD ? DETECT_TIMEOUT : TIMEOUT);
  _timer.reset();
  _counter = 0;
  Serial.printf("[GPS] State: %d (timeout=%lu)\n", state, _timeout);
}

void GpsSensor::setState(State state) {
  setState(state, state, ERROR);
}

void GpsSensor::handleError() const {
  Serial.println("[GPS] UBX error - check wiring/module");
}

void GpsSensor::handleNavPvt() const {
  const auto& pvt = _ubxMsg.payload.navPvt;
  _model.state.gps.latitude = pvt.lat / 1e7f;
  _model.state.gps.longitude = pvt.lon / 1e7f;
  _model.state.gps.altitude = pvt.hMSL / 1e3f;
  _model.state.gps.speed = pvt.gSpeed / 1e2f;
  _model.state.gps.groundCourse = pvt.headMot / 1e5f;
  _model.state.gps.fix = ((pvt.flags >> 8) & 0x03);  // gnssFixOk & fixType
  _model.state.gps.hdop = pvt.hDOP / 100;  // Scale to match NMEA

  if (_model.state.gps.fix > 0 && _model.config.gps.setHomeOnce && !_model.state.gps.homeSet) {
    _model.state.gps.homeLat = _model.state.gps.latitude;
    _model.state.gps.homeLon = _model.state.gps.longitude;
    _model.state.gps.homeSet = true;
    Serial.println("[GPS] Home position set!");
  }
}

void GpsSensor::handleNavSat() const {
  const auto& sat = _ubxMsg.payload.navSat;
  uint8_t activeSats = 0;
  for (uint8_t i = 0; i < std::min(8u, Gps::UBX_NAV_SAT_NUMSV); ++i) {  // NEO-6M max ~8
    if (sat.numSV[i] > 0) activeSats += sat.numSV[i];
  }
  _model.state.gps.numSat = activeSats;
}

void GpsSensor::handleVersion() const {
  const auto& ver = _ubxMsg.payload.monVer;
  Serial.printf("[GPS] u-blox ver: %s (HW:%s SW:%s)\n", ver.swVersion, ver.hwVersion, ver.extension);
  checkSupport(ver.extension);  // Check UBX support
}

void GpsSensor::checkSupport(const char* payload) const {
  // Simple check for UBX-PROTOCOL-VERSION=1 (supports NAV-PVT)
  if (strstr(payload, "UBX-PROTOCOL-VERSION=1")) {
    Serial.println("[GPS] UBX supported");
  } else {
    Serial.println("[GPS] UBX not fully supported - fallback NMEA");
    setState(RECEIVE);  // Skip config if old firmware
  }
}

}  // namespace Espfc::Sensor