/*
 * linearActuator.h - Linear Actuator library using spark mini
 *
 * Library for driving a linear actuator with a spark mini motor controller
 *
 * Spark Mini Docs:
 * https://docs.revrobotics.com/duo-control/adding-more-motors/sparkmini-motor-controller
 *
 * Wiring Example:
 * https://www.notion.so/nurover/Test-Linear-Actuator-with-Spark-Mini-a8f45148c0484c89b5ec3bb7ff39904d?pvs=4#668df350de7d47fa811f62f1e34e0e44
 *
 * Example Usage:
 *  LinearActuator actuator(PC_8, PC_3);
 *  actuator.drive(-1);
 */

#ifndef LinearActuator_h
#define LinearActuator_h

#include <mbed.h>

#define SPARK_MINI_MAX_PULSE_WIDTH 2500
#define SPARK_MINI_MIN_PULSE_WIDTH 500
// period is 50hz (https://docs.revrobotics.com/duo-control/adding-more-motors/sparkmini-motor-controller)
#define SPARK_MINI_PERIOD_MS 20

/// @brief Class to control a linear actuator hooked up to a SparkMini
class LinearActuator {
  PwmOut pwm;      // the output that controls the motor speed
  AnalogIn wiper;  // resistor wiper

 public:
  /// @brief create a new linear actuator and configure pwm
  /// @param pwm the pwm output to used to control a spark mini
  /// @param wiper the wiper pin that reads the potentiometer (position)
  LinearActuator(PinName pwm, PinName wiper);

  /// @brief drives the motor at the given percentage
  /// @param percent a value in [-1, 1] where -1 is full reverse and 1 is full speed forward. 0 is stop.
  void drive(float percent);

  // drive the linear actuator to a position between 0 and 1
  void goToPosition(float position, float deadzone = 0.01);

  /// @return the position of the linear actuator from 0 to 1
  float position();

  void set6Recieved(bool val);
  bool get6Recieved();
};

#endif
