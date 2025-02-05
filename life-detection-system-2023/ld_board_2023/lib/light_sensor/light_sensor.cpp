#include "light_sensor.h"
#include <mbed.h>

bool lightReqRecieved;

LightSensor::LightSensor(PinName in) : _in(in), _counter(0) {
  _in.rise(callback(this, &LightSensor::_onPulse));
}

float LightSensor::getFrequency(int ms) {
  Timer t;
  _counter = 0;
  t.start();
  ThisThread::sleep_for(ms);
  t.stop();
  int total = _counter;
  // elapsed_time returns std::chrono::microseconds, count returns int
  float secs_elapsed = t.elapsed_time().count() / 1000;
  float freq = total / secs_elapsed;
  return freq;
}

void LightSensor::_onPulse() {
  _counter++;
}

void setLightReqRecieved(int val) {
  lightReqRecieved = val;
}

int getLightReqRecieved() {
  return lightReqRecieved;
}