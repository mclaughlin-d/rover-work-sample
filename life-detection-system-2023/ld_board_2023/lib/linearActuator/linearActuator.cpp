#include "linearActuator.h"
#include <mbed.h>

bool posn6recieved;  // jank way to check if command was recieved and not queued

// Class to control a linear actuator hooked up to a SparkMini
LinearActuator::LinearActuator(PinName pwm, PinName wiper) : pwm(pwm), wiper(wiper) {
  this->pwm.period_ms(SPARK_MINI_PERIOD_MS);
}

void LinearActuator::drive(float percent) {
  // middle point between MIN and MAX width is moving in no direction, so
  // find the difference between max and min pulsewidth and / 2 since we want the amount
  // we can differ from middle in either direction
  const int midRange = (SPARK_MINI_MAX_PULSE_WIDTH - SPARK_MINI_MIN_PULSE_WIDTH) / 2;
  // find midpoint, and then add percentage to account for speed
  this->pwm.pulsewidth_us(SPARK_MINI_MAX_PULSE_WIDTH - midRange + midRange * percent);
}

void LinearActuator::goToPosition(float position, float deadzone) {
  float movementSpeed = 0.9;
  float currentPos = this->position();
  while (abs(currentPos - position) > deadzone) {
    if (currentPos > position) {
      this->drive(-movementSpeed);
    } else {
      this->drive(movementSpeed);
    }
    currentPos = this->position();
  }
  this->drive(0.0);
}

float LinearActuator::position() {
  return wiper.read();
}

void LinearActuator::set6Recieved(bool val) {
  posn6recieved = val;
}

bool LinearActuator::get6Recieved() {
  return posn6recieved;
}