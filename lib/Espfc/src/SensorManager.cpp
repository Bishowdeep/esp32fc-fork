#include "SensorManager.h"
#include "Sensor/GpsSensor.hpp"  // NEW: GPS include
namespace Espfc {

SensorManager::SensorManager(Model& model): _model(model), _gyro(model), _accel(model), _mag(model), _baro(model), _voltage(model), _fusion(model), _fusionUpdate(false) {}

int SensorManager::begin()
{
  _gyro.begin();
  _accel.begin();
  _mag.begin();
  _baro.begin();
  _voltage.begin();
  _fusion.begin();
  _button.begin(_model.config.pin[PIN_BUTTON]);

  // NEW: GPS init (after baro, low priority serial)
  if (_model.config.gps.enabled) {
    Device::SerialDevice* gpsPort = new Device::SerialDevice("GPS");
    gpsPort->setBaud(_model.config.gps.baud);
    gpsPort->setPins(_model.config.pin[PIN_SERIAL_2_RX], _model.config.pin[PIN_SERIAL_2_TX]);
    int gpsStatus = _gps.begin(gpsPort, _model.config.gps.baud);
    if (gpsStatus == 0) {
      Serial.println("[FC] GPS sensor initialized successfully");
    } else {
      Serial.printf("[FC] GPS init failed: %d (check wiring/baud)\n", gpsStatus);
    }
  }

  return 1;
}

int FAST_CODE_ATTR SensorManager::read()
{
  _gyro.read();

  if(_model.state.loopTimer.syncTo(_model.state.gyro.timer))
  {
    _model.state.appQueue.send(Event(EVENT_GYRO_READ));
  }

  if(_model.state.accel.timer.syncTo(_model.state.gyro.timer))
  {
    _accel.update();
    _model.state.appQueue.send(Event(EVENT_ACCEL_READ));
    _model.state.mode.button = _button.update();
    return 1;
  }

  if(_mag.update()) return 1;

  if(_baro.update()) return 1;

  if(_voltage.update()) return 1;

  return 0;
}

int FAST_CODE_ATTR SensorManager::preLoop()
{
  _gyro.filter();
  if(_model.state.gyro.biasSamples == 0)
  {
    _model.state.gyro.biasSamples = -1;
    _fusion.restoreGain();
  }
  return 1;
}

int SensorManager::postLoop()
{
  _gyro.postLoop();
  return 1;
}

int FAST_CODE_ATTR SensorManager::fusion()
{
  return _fusion.update();
}

// main task
int FAST_CODE_ATTR SensorManager::update()
{
  _gyro.read();
  return preLoop();
}

// sub task
int SensorManager::updateDelayed()
{
  _gyro.postLoop();

  // update at most one sensor besides gyro
  int status = 0;
  if(_model.state.accel.timer.syncTo(_model.state.gyro.timer))
  {
    _accel.update();
    _model.state.mode.button = _button.update();
    status = 1;
  }

  // delay imu update to next cycle
  if(_fusionUpdate)
  {
    _fusionUpdate = false;
    _fusion.update();
  }
  _fusionUpdate = status;

  if(status) return 1;

  if(_mag.update()) return 1;

  if(_baro.update()) return 1;

  if(_voltage.update()) return 0;

  // NEW: GPS update (serial, low freq ~1-5Hz, after voltage)
  if (_model.config.gps.enabled && _gps.update() == 0) {
    // Copy parsed data to model state (for MSP, PID, etc.)
    auto gpsData = _gps.getData();  // Assumes GpsSensor has getData() returning GpsEst
    _model.state.gps.latitude = gpsData.latitude;
    _model.state.gps.longitude = gpsData.longitude;
    _model.state.gps.altitude = gpsData.altitude;
    _model.state.gps.speed = gpsData.speed;
    _model.state.gps.groundCourse = gpsData.course;
    _model.state.gps.fix = gpsData.fix;
    _model.state.gps.numSat = gpsData.sats;
    _model.state.gps.hdop = gpsData.hdop;

    // Optional: Trigger events if fix acquired
    if (_model.state.gps.fix > 0 && _model.state.gps.numSat >= _model.config.gps.minSats) {
      _model.state.appQueue.send(Event(EVENT_GPS_FIX));  // Custom event for modes
    }
    status = 1;  // Mark as updated
  }

  return status ? 1 : 0;
}

}