#include "mbed.h"
#include "PwmOut.h"
#include "RoverSerial.h"
#include "./threads/temp_thread.cpp"

// pins
PwmOut drillSpin(PA_6);
PwmOut stageMove(PA_7);
PwmOut linAcc1(PB_6);
PwmOut linAcc2(PB_9);
PwmOut lidServo(PB_10);

DigitalOut pin(D7);

AnalogIn humiditySensorPin(PC_3);

// constants for humidity sensor calibration TODO find better ones
float air_reading = 1.2;
float water_reading = 0.7;

typedef struct
{
  float drill_spin;
  float stage_move;
  float lin_actuator;
  int pump_dir;
  float lid_servo;

} auger_command_t;

typedef struct
{
  float rh_reading;
  float f_temp;
  float c_temp;
  // TODO add a timestamp for the temp readings maybe?
} auger_status_t;

int floatToPulseWidth(float speed)
{
  return 1500 + 1000 * speed;
}

float pinInputToVoltage(float in)
{
  return in * 3.3;
}

float voltageToRH(float voltage)
{
  /*
  RH = (voltage - water_reading)/(air_reading - water_reading) * 100.0
  */
  return (voltage - water_reading) / (air_reading - water_reading) * 100.0;
}

UnbufferedRoverSerial<auger_status_t, auger_command_t> jetson(USBTX, USBRX, 57600);
auger_status_t auger_status;

void auger_init()
{
  drillSpin.period_ms(20);
  stageMove.period_ms(20);
  lidServo.period_ms(20);
}

float rh = 0;
float temp = 0;
int success = 0;

int main()
{
  auger_init();

  temp_thread.start(temp_sensor_main);

  while (1)
  {
    volatile double rh_reading = humiditySensorPin.read() * 3.3;

    if (jetson.readable())
    {
      auger_command_t command = jetson.read();

      // move components
      drillSpin.pulsewidth_us(floatToPulseWidth(command.drill_spin));
      stageMove.pulsewidth_us(floatToPulseWidth(command.stage_move));
      linAcc1.pulsewidth_us(floatToPulseWidth(command.lin_actuator));
      lidServo.write(command.lid_servo);

      // compute humidity and temp
      rh = voltageToRH(pinInputToVoltage(humiditySensorPin.read()));

      // make return message
      auger_status_t status;
      status.rh_reading = rh;
      status.c_temp = c_temp;
      status.f_temp = f_temp;
      // write it
      jetson.write(status);
    }
  }
}
