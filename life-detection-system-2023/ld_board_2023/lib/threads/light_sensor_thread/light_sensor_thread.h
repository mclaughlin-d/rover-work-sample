#ifndef light_sensor_thread_h
#define light_sensor_thread_h

#include "MbedQueue.h"
#include "light_sensor.h"
#include "mbed.h"

#define NUM_LIGHT_SENSORS 6

LightSensor light_sensors[NUM_LIGHT_SENSORS] = {
    {D5}, {D4}, {D3}, {D9}, {D14}, {D15},
};

float frequencies[NUM_LIGHT_SENSORS];

unsigned int timestamps[NUM_LIGHT_SENSORS];

rovercan::MbedQueue<int, 3> light_sensor_queue;
// -1 for no request, otherwise position of sensor in array
int current_light_sensor_cmd = -1;

bool cmd_in_bounds(int cmd) {
  return cmd > -1 && cmd < NUM_LIGHT_SENSORS;
}

void light_sensor_thread_main() {
  while (true) {
    if (light_sensor_queue.get(&current_light_sensor_cmd)) {
      if (cmd_in_bounds(current_light_sensor_cmd)) {
        frequencies[current_light_sensor_cmd] =
            light_sensors[current_light_sensor_cmd].getFrequency();
        timestamps[current_light_sensor_cmd] = ((unsigned int)(time(NULL)));
        volatile int a = 0;
      }
    }
  }
}

Thread light_sensor_thread(osPriorityNormal);

#endif
