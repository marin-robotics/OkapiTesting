#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
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
int launcher_cycle[] = {77,82,87,92,97,112, 117, 122, 127};
int launcher_power = 8; // default power level
bool one_stick = true;

// Defining ports
pros::Motor left_front_motor(1,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_motor(10,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back_motor(11,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back_motor(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor snarfer_motor(2);
pros::Motor secondary_snarfer_motor(4);
pros::Motor launcher_motor(13,pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor roller_motor(3);
pros::ADIDigitalOut firing_pneumatic(DIGITAL_SENSOR_PORT);

// Set motor groups
pros::Motor_Group left_motors ({left_front_motor, left_back_motor});
pros::Motor_Group right_motors ({right_front_motor, right_back_motor});

// Controller
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
  left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  


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
float Wheel_Diameter = 4.5;
float Wheel_Circumference = Wheel_Diameter * 3.1416;
float Turning_Diameter = 19;
float Turning_Circumference = Turning_Diameter * 3.1416;
//float Turning_Power = 127;
int v = 1;

void turn(float angle){
  float Turning_Distance = angle/360 * Turning_Circumference;
  float Wheel_Revolutions = Turning_Distance/(Wheel_Diameter*3.1416);
  float Wheel_Rotation = Wheel_Revolutions*360;

  left_motors.move_absolute(Wheel_Rotation, 60);
  right_motors.move_absolute(Wheel_Rotation, 60);
  /*while (!((left_motors.get_positions() < Wheel_Rotation+5) && (left_motors.get_positions() > Wheel_Rotation-5) && (right_motors.get_positions() < Wheel_Rotation+5) && (right_motors.get_positions() > Wheel_Rotation-5))) {
    // Continue running this loop as long as the motors are not within +-5 units of its goal
    pros::delay(2);
  left_motors.tare_position();
  right_motors.tare_position();
  */


  /*while (true){
    for (int count = 0; count <= std::size(left_motors); count++)
  }

  while (!((left_motors.get_positions() < Wheel_Rotation+5) && (left_motors.get_positions() > Wheel_Rotation-5) && (right_motors.get_positions() < Wheel_Rotation+5) && (right_motors.get_positions() > Wheel_Rotation-5))) {
    // Continue running this loop as long as the motors are not within +-5 units of its goal
    pros::delay(2);
  left_motors.tare_position();
  right_motors.tare_position();
  }*/
  
  
  
  //if (angle < 0) Wheel_Revolutions *= -1;
  /**left_front_motor.move_velocity(60);
  left_back_motor.move_velocity(60);
  right_front_motor.move_velocity(-60);
  right_back_motor.move_velocity(-60);
  pros::delay(1000*Wheel_Revolutions);
  left_front_motor.move_velocity(0);
  left_back_motor.move_velocity(0);
  right_front_motor.move_velocity(0);
  right_back_motor.move_velocity(0);**/
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
  if ((firing_input == 1) && (launcher_toggle)) {
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
  //turn(360);
  
  /*
  left_motors.move_relative(135, 100);
  right_motors.move_relative(135,100);
  pros::delay(1000);
  */

  while (true) {
    // Get joystick values
    left_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    left_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    right_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    right_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
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
      snarfer_motor = -127;
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        secondary_snarfer_motor = 127;
      } else {
        secondary_snarfer_motor = -127;
      }
    } else {
      snarfer_motor = 0;
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        secondary_snarfer_motor = 127;
      } else {
        secondary_snarfer_motor = 0;
      }
    }
    /*  
      snarfer_motor.move(-127);
      secondary_snarfer_motor.move(-127);
    } else {
      snarfer_motor.move(0);
      secondary_snarfer_motor.move(0);
    }
    if ((!snarfer_toggle) && (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))){
      secondary_snarfer_motor.move(127);
    }
    */

    if (!((controller.get_digital(DIGITAL_R1)) && (controller.get_digital(DIGITAL_R2)))) {
      if (controller.get_digital(DIGITAL_R1)) {
        roller_motor = -100;
      } else if (controller.get_digital(DIGITAL_R2)) {
        roller_motor = 100;
      } else {
        roller_motor = 0;
      }
    }


    // Fire pnuematic (Y)
    firepnuematic();

    // Drive Control Loop
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      one_stick = !one_stick;
    }
    if (one_stick){
      left_motors.move(forward_table[left_x+127] + forward_table[left_y+127]);
      right_motors.move(forward_table[left_x+127] - forward_table[left_y+127]);
    } else {
      left_motors.move(forward_table[left_y+127]);
      right_motors.move(-1*forward_table[right_y+127]);
    }
    
    // Working on braking RANDOM CODE SNIPPETS BELOW
    /*if (one_stick) {
      if ((forward_table[left_y] != 0) && (turn_table[left_x] != 0)) { // If one stick drive is enabled and joystick is actually moving then go.
        left_motors.move(turn_table[left_x+127] + forward_table[left_y+127]);
        right_motors.move(turn_table[left_x+127] - forward_table[left_y+127]);
      } else {
      left_motors.brake();
      right_motors.brake();
      }
    } else {
      if ((forward_table[left_y]) && (forward_table[right_y])) {}
        
    } 
    } if ((!one_stick) && (turn_table[left_y] = 0)) { // Tank Drive
      
      left_motors.move(forward_table[left_y+127]);
      
    } if ((!one_stick) && (turn_table[right_y] = 0))
      right_motors.move(-1*forward_table[right_y+127]);
      
      left_front_motor.move(forward_table[left_y+127]);
      left_back_motor.move(forward_table[left_y+127]);
      right_front_motor.move(-1*forward_table[right_y+127]);
      right_back_motor.move(-1*forward_table[right_y+127]);
      */

    // Wait a bit before continuing the loop
    pros::delay(20);
  }
}