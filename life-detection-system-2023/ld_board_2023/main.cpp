#include <stdio.h>
#include <time.h>
#include "RoverSerial.h"
#include "USBSerial.h"
#include "actuator_thread.h"
#include "light_sensor.h"
#include "light_sensor_thread.h"
#include "linearActuator.h"
#include "mbed.h"
#include "pump_thread.h"

typedef struct {
  float linActPos;         // float from 0-1 representing goal potentiometer reading
  int pumpDir;             // -1 for backwards, 0 for off, 1 for forwards
  int lightSensorRequest;  // int from 0-4 representing which light sensor we want a reading from, OR -1 to represent no request
  int lightSensorRequestID;  // int from 0-infinity representing the ID of the light sensor request
  float velocity;            // velocity
  bool isPos;                // wether it's in position mode
} ld_command_t;

// typedef struct {
//   int sensorPos;
//   float sensorFreq;
// } light_sensor_reading_t;

bool lightReqRecieved2;
int lastLightReqID;

typedef struct {
  float sensorFreq[NUM_LIGHT_SENSORS];
  unsigned int sensorTime[NUM_LIGHT_SENSORS];
  // light_sensor_reading_t sensorReading; // response to most recent request, or null for none
  float linActPos;  // float from 0-1 representing current pos of linear actuator
  int pumpDir;      // -1 for "pump is moving backwards", 0 for pump not moving, 1 for forwards
  int posn6recieved;
  int lightReqRecieved;
} ld_status_t;

int main() {
  // readCanID();
  DigitalOut led1(LED1);
  led1 = 0;

  UnbufferedRoverSerial<ld_status_t, ld_command_t> laptop(USBTX, USBRX, 57600);
  ld_status_t status;
  // light_sensor_reading_t sensorReading;

  actuator_thread.start(actuator_thread_main);
  pump_thread.start(pump_thread_main);
  light_sensor_thread.start(light_sensor_thread_main);

  lastLightReqID = -1;

  while (1) {
    // light_sensor_queue.put(0);

    if (laptop.readable()) {
      ld_command_t command = laptop.read();
      pump_queue.put(command.pumpDir);

      if (command.lightSensorRequestID != lastLightReqID) {
        // only put light sensor request in the queue if it is a legit command
        if (cmd_in_bounds(command.lightSensorRequest)) {
          light_sensor_queue.put(command.lightSensorRequest);
          lightReqRecieved2 = 1;
        } else {
          lightReqRecieved2 = 0;
        }
      }
      lastLightReqID = command.lightSensorRequestID;

      lin_act_cmd_t lin_command;
      if (command.isPos) {
        lin_command.value = command.linActPos;
        lin_command.isPos = command.isPos;

      } else {
        lin_command.value = command.velocity;
        lin_command.isPos = command.isPos;
      }

      actuator_queue.put(lin_command);
    }

    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
      status.sensorFreq[i] = frequencies[i];
      status.sensorTime[i] = timestamps[i];
    }
    status.linActPos = actuator.position();
    status.pumpDir = pumpDir.read();
    // status = {frequencies, timestamps, actuator.position(), pumpDir.read()};
    status.posn6recieved = actuator.get6Recieved();
    status.lightReqRecieved = lightReqRecieved2;
    laptop.write(status);

    ThisThread::sleep_for(1ms);
  }
  return 1;
}
