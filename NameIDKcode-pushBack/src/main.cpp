#include "pros/rtos.hpp"
#include <type_traits> // IWYU pragma: keep
#pragma region ChassisBuild
#include "main.h"
#include "globals.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"


// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(16);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-14);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -7);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.375);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.75, // 12.75 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
  6.25,   // proportional gain (kP) - start lower for stability
  0,   // integral gain (kI) - keep at 0 for now
  0.8,   // derivative gain (kD) - start low
  0,   // anti windup
  1,   // small error range, in inches
  100, // small error range timeout, in milliseconds
  3,   // large error range, in inches
  500, // large error range timeout, in milliseconds
  20   // maximum acceleration (slew)
);

// angular PID controller (unchanged for now)
lemlib::ControllerSettings angular_controller(
  0.95,   // proportional gain (kP)
  0.0006,   // integral gain (kI)
  0.4,  // derivative gain (kD)
  1,   // anti windup
  1,   // small error range, in degrees
  100, // small error range timeout, in milliseconds
  3,   // large error range, in degrees
  500, // large error range timeout, in milliseconds
  20    // maximum acceleration (slew)
);

lemlib::PID pid(1, 1, 1, 1, false);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);



// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

#pragma endregion ChassisBuild
#pragma region intilization 

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    // Only calibrate here if NOT using competition control
    // chassis.calibrate();
    // Wait for IMU to finish calibrating if you calibrate here
    // while (imu.is_calibrating()) pros::delay(10);
    // chassis.setPose(0, 0, 0);

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
    
}

#pragma endregion intilization 

#pragma region pre-auton

/**
 * Runs while the robot is disabled
 */
void disabled() {
  
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
  pros::lcd::initialize();
  chassis.calibrate(); // recalibrate sensors
  // Wait for IMU to finish calibrating
  while (imu.is_calibrating()) pros::delay(10);
  chassis.setPose(0, 0, 0); // reset odometry
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

#pragma endregion pre-auton

#pragma region autonomous

void toggle_odom(){                  //odomLift 
  if(odom.is_extended()){
    odom.retract();
  }
  else{
    odom.extend();
  }
}

void toggle_pusher(){                  //pusher
  if(pusher.is_extended()){
    pusher.retract();
  }
  else{
    pusher.extend();
  }
}

void toggle_pto(){                  //pto
  if(pto.is_extended()){
    pto.retract();
  }
  else{
    pto.extend();
  }
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */



#pragma endregion autonomous

#pragma region autoncallback

void redLeft() {
  chassis.setPose(0, 0, -10); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(-6, 16.5, -10, 1500, {.forwards = true, .maxSpeed = 127}, false);
  chassis.moveToPose(-6, 32, -10, 1500, {.forwards = true, .maxSpeed = 50}, false);
  chassis.turnToHeading(-70, 1000);
  pusher.extend(); // extend pusher
  chassis.moveToPose(-100, -17, -70, 1500, {.forwards = true, .maxSpeed = 127}, false);
  chassis.turnToHeading(182, 1000, {}, false);
  chassis.moveToPose(-28, -55, 182, 1500, {.forwards = true, .maxSpeed = 127}, false);
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(-28, -37, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(-28, -43, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(300);
  chassis.moveToPose(-28, -10, 182, 1200, {.forwards = false, .maxSpeed = 127}, false);
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {}, false); // turn to face goal
  liftMotor.move(127); // lift up
  pros::delay(2000); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(-28, 20, 0, 1500, {.forwards = true, .maxSpeed = 127}, false);
  intakeMotor.move(-127); // start intake out
}

void redRight() {
  chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(0, 22.5, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);
  chassis.moveToPose(0, 36, 0, 3000, {.forwards = true, .maxSpeed = 70}, false);
  chassis.turnToHeading(145, 2000); // mirrored angle
  pusher.extend(); // extend pusher
  chassis.moveToPose(26, -34, 169, 4000, {.forwards = true, .maxSpeed = 127}, false); // mirrored X and heading
  pros::delay(2000);
  intakeMotor.move(0); // stop intake
  chassis.moveToPose(23, -30, 169, 2000, {.forwards = true, .maxSpeed = 127}, false); // mirrored X and heading
  chassis.turnToHeading(20, 2000, {}, false); // mirrored angle
  pusher.retract(); // retract pusher
  liftMotor.move(127); // lift up
  pros::delay(1000); // wait for lift to finish
  liftMotor.move(0); // stop lift
  chassis.moveToPose(24, 36, 0, 2500, {.forwards = true, .maxSpeed = 100}, false); // mirrored X
  intakeMotor.move(-127); // start intake in
}

void blueLeft() {
  chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(0, 22.5, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);
  chassis.moveToPose(0, 36, 0, 3000, {.forwards = true, .maxSpeed = 70}, false);
  chassis.turnToHeading(-145, 2000);
  pusher.extend(); // extend pusher
  chassis.moveToPose(-26, -34, -169, 4000, {.forwards = true, .maxSpeed = 127}, false);
  pros::delay(2000);
  intakeMotor.move(0); // stop intake
  chassis.moveToPose(-23, -30, -169, 2000, {.forwards = true, .maxSpeed = 127}, false);
  chassis.turnToHeading(-20, 2000, {}, false); // turn to face goal
  pusher.retract(); // retract pusher
  liftMotor.move(127); // lift up
  pros::delay(1000); // wait for lift to finish
  liftMotor.move(0); // stop lift
  chassis.moveToPose(-24, 36, 0, 2500, {.forwards = true, .maxSpeed = 100}, false);
  intakeMotor.move(-127); // start intake in
}

void blueRight() {
  chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(0, 22.5, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);
  chassis.moveToPose(0, 36, 0, 3000, {.forwards = true, .maxSpeed = 70}, false);
  chassis.turnToHeading(145, 2000); // mirrored angle
  pusher.extend(); // extend pusher
  chassis.moveToPose(26, -34, 169, 4000, {.forwards = true, .maxSpeed = 127}, false); // mirrored X and heading
  pros::delay(2000);
  intakeMotor.move(0); // stop intake
  chassis.moveToPose(23, -30, 169, 2000, {.forwards = true, .maxSpeed = 127}, false); // mirrored X and heading
  chassis.turnToHeading(20, 2000, {}, false); // mirrored angle
  pusher.retract(); // retract pusher
  liftMotor.move(127); // lift up
  pros::delay(1000); // wait for lift to finish
  liftMotor.move(0); // stop lift
  chassis.moveToPose(24, 36, 0, 2500, {.forwards = true, .maxSpeed = 100}, false); // mirrored X
  intakeMotor.move(-127); // start intake in
}

void autonTest() {
  chassis.setPose(0, 0, 0); // reset odometry
  chassis.moveToPose(0, 24, 0, 2000);

}

void autonomous() {
  liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set lift brake mode to hold

  redLeft(); // run red left autonomous routine
  //redRight(); // run red right autonomous routine
  //blueLeft(); // run blue left autonomous routine
  //blueRight(); // run blue right autonomous routine
  //autonTest(); // run auton test routine
  //skillsAuton(); // run skills autonomous routine
}

#pragma endregion autoncallback


#pragma region userControl

/**
 * Runs in driver control
 */
void opcontrol() {
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // loop to continuously update motors

    while (true) {

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            toggle_odom();
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            toggle_pusher();
        }
        

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeMotor.move(127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeMotor.move(-127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftMotor.move(127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftMotor.move(-127);
        } else {
            intakeMotor.move(0);
            liftMotor.move(0);
        }

        

        

		// delay to save resources
        pros::delay(10);
        
    }
}

#pragma endregion userControl