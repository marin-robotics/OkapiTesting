#include "main.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include <string>

using namespace okapi;

void opcontrol() {
  // Initialize motor objects
  Motor leftBackMotor(1);
  Motor rightBackMotor(-11);
  Motor leftFrontMotor(8);
  Motor rightFrontMotor(-18);
  MotorGroup rightWheels({rightBackMotor, rightFrontMotor});
  MotorGroup leftWheels({leftBackMotor, leftFrontMotor});

  // Set up motor groups
  std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftWheels, rightWheels)
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .build();

  // Create a controller for driver input
  auto controller = Controller();

  while (true) {
    // Get joystick values
    float drivePower = controller.getAnalog(ControllerAnalog::leftY);
    float turnPower = controller.getAnalog(ControllerAnalog::rightX);
    if (drivePower != 0 || turnPower != 0 ){
      std::cout << "Driver Power: " << drivePower << "\n";
      std::cout << "Turn Power: " << turnPower << "\n"; 
    }
    // Calculate left and right motor speeds based on joystick values
    double leftSpeed = drivePower + turnPower;
    double rightSpeed = drivePower - turnPower;
    if (leftSpeed != 0 || rightSpeed != 0 ){
      std::cout << "Left Y: " << leftSpeed << "\n"; 
      std::cout << "Right X: " << rightSpeed << "\n"; 
    }

    // Set the motor speeds
    chassis->getModel()->tank(leftSpeed, rightSpeed);

    // Delay to prevent the loop from running too quickly
    pros::delay(10);
  }
}