#include "main.h"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#define DEADZONE 15
#define DIGITAL_SENSOR_PORT 'A'

// Variables
bool launcher_toggle = false, snarfer_toggle = false;
int left_x, left_y, right_x, right_y, snarfer, firing_input, launcher;
int forward_table[] = {-127,-125,-123,-121,-119,-117,-115,-113,-112,-110,-108,-106,-104,-102,-101,-99,-97,-95,-94,-92,-90,-88,-87,-85,-84,-82,-80,-79,-77,-76,-74,-73,-71,-70,-68,-67,-65,-64,-62,-61,-60,-58,-57,-56,-54,-53,-52,-50,-49,-48,-47,-45,-44,-43,-42,-41,-40,-39,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-26,-25,-24,-23,-22,-21,-20,-20,-19,-18,-17,-17,-16,-15,-15,-14,-13,-13,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-5,-4,-4,-3,-3,-3,-3,-2,-2,-2,-2,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,5,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,15,15,16,17,17,18,19,20,20,21,22,23,24,25,26,26,27,28,29,30,31,32,33,34,35,36,37,39,40,41,42,43,44,45,47,48,49,50,52,53,54,56,57,58,60,61,62,64,65,67,68,70,71,73,74,76,77,79,80,82,84,85,87,88,90,92,94,95,97,99,101,102,104,106,108,110,112,113,115,117,119,121,123,125,127};
int turn_table[] = {-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-53,-52,-51,-50,-49,-48,-47,-46,-46,-45,-44,-43,-42,-41,-41,-40,-39,-38,-38,-37,-36,-35,-35,-34,-33,-32,-32,-31,-30,-30,-29,-28,-28,-27,-26,-26,-25,-24,-24,-23,-23,-22,-21,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-11,-10,-10,-9,-9,-9,-8,-8,-8,-7,-7,-7,-6,-6,-6,-5,-5,-5,-5,-4,-4,-4,-4,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,10,10,11,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,34,35,35,36,37,38,38,39,40,41,41,42,43,44,45,46,46,47,48,49,50,51,52,53,53,54,55,56,57,58,59,60,61,62,63};
int launcher_cycle[] = {-97, -107, -117, -127};
int launcher_power = 3; // default power level

// Defining Ports
pros::Motor left_front_motor(1);
pros::Motor right_front_motor(10);
pros::Motor left_back_motor(11);
pros::Motor right_back_motor(20);
pros::Motor snarfer_motor(2);
pros::Motor launcher_motor(13);
pros::ADIDigitalOut firing_pneumatic(DIGITAL_SENSOR_PORT);

// Joystick
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
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
void competition_initialize() {}

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
float Wheel_Diameter = 10;
float Wheel_Circumference = Wheel_Diameter * 3.1416;
float Turning_Diameter = 10;
float Turning_Circumference = Turning_Diameter * 3.1416;
float Turning_Power = 127;

void turn(float angle){
  float Turning_Distance = angle/360 * Turning_Circumference;
  float Wheel_Revolutions = Turning_Distance/Wheel_Diameter;
  //if (angle < 0) Wheel_Revolutions *= -1;
  
}

void autonomous() {
  
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
// toggles for the pnuematics
void firepnuematic() {
  if (firing_input == 1) {
    firing_pneumatic.set_value(true);
    pros::delay(780);
    firing_pneumatic.set_value(false);
    pros::delay(780);
  } else {
    firing_pneumatic.set_value(false);
  }
}

void opcontrol() {
  // Loop forever
  while (true) {
     
    // Get joystick values
    left_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    right_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    firing_input = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y));

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && launcher_power < 3) {
      launcher_power += 1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && launcher_power > 0) {
      launcher_power -= 1;
    }
    
    // Toggle launcher (X)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) == 1) {
      launcher_toggle = !launcher_toggle;
    }
    if (launcher_toggle) {
      launcher_motor.move(launcher_cycle[launcher_power]);
    } else {
      launcher_motor.move(0);
    }

    // Toggle snarfer (B)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == 1) {
      snarfer_toggle = !snarfer_toggle;
    }
    if (snarfer_toggle) {
      snarfer_motor.move(-127);
    } else {
      snarfer_motor.move(0);
    }

    // Fire pnuematic (Y)
    firepnuematic();
    



    // Working Motors
    left_front_motor.move(turn_table[right_x+127] + forward_table[left_y+127]);
    left_back_motor.move(turn_table[right_x+127] + forward_table[left_y+127]);
    right_front_motor.move(turn_table[right_x+127] - forward_table[left_y+127]);
    right_back_motor.move(turn_table[right_x+127] - forward_table[left_y+127]);
    
    // Flipped version of Motors
    //left_front_motor.move(forward_table[left_y+127] + turn_table[right_x+32]);
    //left_back_motor.move(forward_table[left_y+127] + turn_table[right_x+32]);
    //right_front_motor.move(forward_table[left_y+127] - turn_table[right_x+32]);
    //right_back_motor.move(forward_table[left_y+127] - turn_table[right_x+32]);
    
    
    // Wait a bit before continuing the loop
    pros::delay(20);
  }
}