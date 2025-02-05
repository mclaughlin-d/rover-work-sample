#ifndef actuator_thread_h
#define actuator_thread_h

#include "MbedQueue.h"
#include "linearActuator.h"
#include "mbed.h"

LinearActuator actuator(D7, D11);

typedef struct {
  float value;
  bool isPos;
} lin_act_cmd_t;

rovercan::MbedQueue<lin_act_cmd_t, 3> actuator_queue;
lin_act_cmd_t current_act_cmd;

//returns true if a is equal to b within the diff
bool compare_float(float a, float b, float diff) {
  return (abs(a - b) < diff);
}

void actuator_thread_main() {
  while (true) {
    if (actuator_queue.get(&current_act_cmd)) {
      //we're in position mode
      if (current_act_cmd.isPos) {
        if (compare_float(current_act_cmd.value, 1.2, 0.01)) {
          actuator.set6Recieved(true);
          actuator.goToPosition(0.95);

          actuator.drive(1);
          ThisThread::sleep_for(1050ms);
          actuator.drive(0);

          // hopefully clears the queue
          while (actuator_queue.get(&current_act_cmd) != false) {
            actuator_queue.get(&current_act_cmd);
          }

        } else if (current_act_cmd.value > 1.0 || current_act_cmd.value < 0.0) {
          actuator.drive(0);
          actuator.set6Recieved(false);

        } else {
          actuator.goToPosition(current_act_cmd.value);
          actuator.set6Recieved(false);
        }
      }

      // //spin until we get data in mailbox
      // if (actuator_queue.get(&current_act_cmd)) {
      //   actuator.goToPosition(current_act_cmd);
      //   //actuator.drive(-1);
      // }

      //otherwise we're in velocity mode
      else {
        actuator.drive(current_act_cmd.value);
      }
    }
  }
}

Thread actuator_thread(osPriorityNormal);

#endif
