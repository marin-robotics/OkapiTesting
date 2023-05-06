#include "main.h"
#include "display/lv_objx/lv_list.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#include <string>
#include <cmath>
using namespace std;

#define DIGITAL_SENSOR_PORT_A 'A'
#define DIGITAL_SENSOR_PORT_B 'B'
#define DIGITAL_SENSOR_PORT_C 'C'
#define DIGITAL_SENSOR_PORT_D 'D'
#define DIGITAL_SENSOR_PORT_E 'E'

// Variable
int left_x, left_y, right_x, right_y;

float Wheel_Diameter = 4;
float Wheel_Circumference = Wheel_Diameter * 3.1416;
float Turning_Diameter = 14.6;
float Turning_Circumference = Turning_Diameter * 3.1416;
float Turning_Distance, Wheel_Revolutions, Turn_Wheel_Rotation, Forward_Wheel_Rotation;
float Turn_Tuning_Factor = 1;
float Move_Tuning_Factor = 1.05;
bool wait = false; 

// New Code
bool standard_drive = true;
float y_current, x_current, y_direction, x_direction;
float y_true_step;
float x_true_step;
float up_step = 50;
float down_step = -50;
float turn_constant = 0.7;

// Defining Motors
pros::Motor left_front_motor(9,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_motor(20,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back_motor(1,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back_motor(11,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
// Set motor groups
pros::Motor_Group left_motors ({left_front_motor, left_back_motor});
pros::Motor_Group right_motors ({right_front_motor, right_back_motor});

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);  
  left_front_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  left_back_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_front_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_back_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

bool check_left_thresholds(float left_goal, float threshold) {
  if ((left_front_motor.get_position() > left_goal+threshold) || left_front_motor.get_position() < left_goal-threshold) {
    return true;
  }
  if ((left_back_motor.get_position() > left_goal+threshold) || left_back_motor.get_position() < left_goal-threshold) {
    return true;
  }
  return false;
}

bool check_right_thresholds(float right_goal, float threshold) {
  if ((right_front_motor.get_position() > right_goal+threshold) || right_front_motor.get_position() < right_goal-threshold) {
    return true;
  }
  if ((right_back_motor.get_position() > right_goal+threshold) || right_back_motor.get_position() < right_goal-threshold) {
    return true;
  }
  return false;
}

void turn(float angle, float velocity){
  wait = true; // Stop turn from running multiple times simultaneously
  
  Turning_Distance = angle/360 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Turn_Tuning_Factor*Wheel_Revolutions*360; // Degrees that the wheel should turn

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_relative(Turn_Wheel_Rotation, -velocity);
  right_motors.move_relative(Turn_Wheel_Rotation, velocity);
  while (check_left_thresholds(Turn_Wheel_Rotation, 10)) {
    pros::delay(2);
  }
  left_motors.tare_position();
  right_motors.tare_position();
  
  wait = false; // Allow the next movement request to run
}

// Pivot off left side
void left_pivot_turn(float angle, float velocity){
  wait = true; // Stop turn from running multiple times simultaneously
  
  Turning_Distance = angle/180 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Turn_Tuning_Factor * Wheel_Revolutions*360; // Degrees that the wheel should turn

  right_motors.tare_position();

  right_motors.move_relative(Turn_Wheel_Rotation, velocity);
  while (check_right_thresholds(-Turn_Wheel_Rotation, 10)) {
    pros::delay(2);
  }
  right_motors.tare_position();
  
  wait = false; // Allow the next movement request to run
}

void right_pivot_turn(float angle, float velocity){
  wait = true; // Stop turn from running multiple times simultaneously
  
  Turning_Distance = angle/180 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Turn_Tuning_Factor * Wheel_Revolutions*360; // Degrees that the wheel should turn

  left_motors.tare_position();

  left_motors.move_relative(Turn_Wheel_Rotation, velocity);
  while (check_left_thresholds(Turn_Wheel_Rotation, 10)) {
    pros::delay(2);
  }
  left_motors.tare_position();
  
  wait = false; // Allow the next movement request to run
}

void move(float inches, float velocity) { // Movement request to move the bot a set distance at a set speed.
  wait = true;
  
  Forward_Wheel_Rotation = Move_Tuning_Factor*(inches/Wheel_Circumference)*360;

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_relative(Forward_Wheel_Rotation,velocity);
  right_motors.move_relative(Forward_Wheel_Rotation,velocity);
  while (check_left_thresholds(Forward_Wheel_Rotation, 270)) {
    // Continue running this loop as long as the motors are not within +-5 units of its goal
    pros::lcd::set_text(1,"Running Delay Loop");
    pros::delay(2);
  }
  left_motors.tare_position();
  right_motors.tare_position();

  wait = false;
}

void autonomous() {
  pros::lcd::clear();
  pros::lcd::set_text(1, "Starting"); 
  // auton1();


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off. 
 */

}



void opcontrol() {
  while (true) {

    pros::lcd::clear();
    // Get joystick values
    float left_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    float left_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    float right_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    float right_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    
    if (abs(right_x) < abs(left_x)) {
      right_x = left_x;
    }
    // Drive Control Loop (LEFT)

    y_direction = sgn(left_y);
    x_direction = sgn(right_x);

    float y_goal = powf((abs(left_y) / 127), 2) * 127;
    float x_goal = powf((abs(right_x) / 127), 2) * 127;

//    float x_goal = powf((abs(right_x) / 127), (2 - (y_goal / 127))) * 127;

    float y_error = y_goal - y_current;
    float x_error = x_goal - x_current;

    if (((up_step > y_error) && (y_error > 0)) || ((down_step < y_error) && (y_error <= 0))) {
      y_true_step = y_error;
    } else if (y_goal > y_current) {
      y_true_step = up_step;
    } else if (y_goal < y_current) {
      y_true_step = down_step;
    } else {
      y_true_step = 0;
    }
    if (((up_step > x_error) && (x_error > 0)) || ((down_step < x_error) && (x_error <= 0))) {
      x_true_step = x_error;
    } else if (x_goal > x_current) {
      x_true_step = up_step;
    } else if (x_goal < x_current) {
      x_true_step = down_step;
    } else {
      x_true_step = 0;
    }
    y_current += y_true_step;
    x_current += x_true_step;

    if (standard_drive) {
      left_motors.move((y_current * y_direction) + (turn_constant * x_current * x_direction));
      right_motors.move((y_current * y_direction) - (turn_constant * x_current * x_direction));
    } else {
      left_motors.move((y_current * y_direction) - (turn_constant * x_current * x_direction));
      right_motors.move((y_current * y_direction) + (turn_constant * x_current * x_direction));
    }

    pros::delay(20);
  }
}