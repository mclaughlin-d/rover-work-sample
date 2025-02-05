#include "mbed.h"
#include "MbedQueue.h"

// serial stuff
static BufferedSerial serial_port(PA_0, PA_1);  // on TX and RX pins, with baud rate

// rovercan stuff
rovercan::MbedQueue<bool, 3> temp_queue;

Thread temp_thread(osPriorityNormal);

// make a buffer for the message
uint8_t msg[12];

float c_temp;
float f_temp;

void buf_to_temps()
{
  // TODO abstract? also only use lower two bytes for now, change ot only send two bytes later?
  c_temp = ((msg[4] * pow(2, 8)) + msg[5]) / 100;
  f_temp = ((msg[10] * pow(2, 8)) + msg[11]) / 100;
}

int curr_index;

void cb()
{
  uint8_t req_byte = 0xFF;
  serial_port.write(&req_byte, 1);

  ThisThread::sleep_for(250ms);

  if (12 == serial_port.read(msg, 12) && msg[0] == 0x0F && msg[1] == 0x0F && msg[6] == 0xF0 && msg[7] == 0xF0)
  {
    buf_to_temps();
  }
}

void temp_sensor_main()
{
  serial_port.set_baud(9600);
  serial_port.set_format(8, BufferedSerial::None, 1);

  while (1)
  {
    cb();
  }
}