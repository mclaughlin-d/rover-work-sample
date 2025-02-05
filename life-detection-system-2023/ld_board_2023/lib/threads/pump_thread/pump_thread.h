#ifndef pump_thread_h
#define pump_thread_h

#include "MbedQueue.h"
#include "mbed.h"

DigitalOut pumpStep(A1);
DigitalOut pumpDir(A3);

rovercan::MbedQueue<int, 3> pump_queue;
// 1 for forward, 0 for off, -1 for backward
int current_pump_cmd;

void pump_thread_main() {
  pumpStep = 0;
  pumpDir = 1;

  while (true) {
    if (pump_queue.get(&current_pump_cmd)) {
      if (current_pump_cmd == 1) {
        pumpDir = 1;
      } else if (current_pump_cmd == -1) {
        pumpDir = 0;
      }
    }

    if (current_pump_cmd != 0) {
      pumpStep = !pumpStep;
    }

    //ThisThread::sleep_for(1ms);
    wait_us(500);
  }
}

Thread pump_thread(osPriorityNormal);

#endif
