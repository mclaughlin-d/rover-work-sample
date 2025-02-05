#ifndef light_sensor_h
#define light_sensor_h

#include <mbed.h>

class LightSensor {
 public:
  LightSensor(PinName in);
  float getFrequency(int ms = 3000);

 private:
  InterruptIn _in;
  volatile int _counter;
  void _onPulse();

  void setLightReqRecieved(int val);
  int getLightReqRecieved();
};

#endif