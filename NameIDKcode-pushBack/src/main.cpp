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
  1.7,   // proportional gain (kP) - start lower for stability
  0.015,   // integral gain (kI) - keep at 0 for now
  .22,   // derivative gain (kD) - start low
  25,   // anti windup
  1,   // small error range, in inches
  100, // small error range timeout, in milliseconds
  3,   // large error range, in inches
  500, // large error range timeout, in milliseconds
  20   // maximum acceleration (slew)
);

// angular PID controller (unchanged for now)
lemlib::ControllerSettings angular_controller(
  .9,   // proportional gain (kP)
  0,   // integral gain (kI)
  0.35,  // derivative gain (kD)
  15,   // anti windup
  1,   // small error range, in degrees
  100, // small error range timeout, in milliseconds
  3,   // large error range, in degrees
  500, // large error range timeout, in milliseconds
  20    // maximum acceleration (slew)
);


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

void redLeftTEST() {
  chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(-12.75, 16.5, -10, 1500, {.forwards = true, .minSpeed = 25}, false);
  chassis.moveToPose(-12.75, 37, -10, 2000, {.forwards = true, .minSpeed = 30}, false);
  pusher.extend(); // extend pusher
  pros::delay(1000);
  chassis.turnToHeading(-80, 1000, {.minSpeed = 70, .earlyExitRange = 3}); // added exit conditions
  chassis.moveToPose(-37.5, -35, 184, 3500, {.forwards = true, .minSpeed = 62}, false);
  pros::delay(1000);
  intakeMotor.move(127);
  chassis.moveToPose(-38, -43, 183.76, 1550, {.forwards = true, .minSpeed = 90}, false);
  pros::delay(1750);
  chassis.moveToPose(-28, -30, 180, 1000, {.forwards = false, .minSpeed= 60}, true);
  intakeMotor.move(-120);
  pros::delay(900);
  intakeMotor.move(127);
  pusher.toggle();
  chassis.turnToHeading(0, 1000, {.minSpeed = 70}, false);
  chassis.moveToPose(-34, 5, 0, 1500, {.minSpeed = 80}, false);
  liftMotor.move(127); // lift up
  pros::delay(1000); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(-34, 30, 0, 1300, {.forwards = true, .minSpeed = 100}, true);
  intakeMotor.move(-127); // start intake out*/


  //chassis.turnToHeading(180,1000);
  /*chassis.moveToPose(-28, -33, -180, 3000, {.forwards = true, .minSpeed = 62}, false);
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(-28, -37, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(-28, -43, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(3000);
  intakeMotor.move(0);
  /*chassis.moveToPose(-30, -25, 181, 1200, {.forwards = false, .minSpeed = 127}, false);
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {.minSpeed = 127}, false); // turn to face goal
  liftMotor.move(127); // lift up
  pros::delay(1600); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(-28, 20, 0, 1300, {.forwards = true, .minSpeed = 127}, false);
  intakeMotor.move(-127); // start intake out*/
}

void redRight() {
  chassis.setPose(0, 0, 10); // reset odometry (flipped angle)
  intakeMotor.move(127); // start intake
  chassis.moveToPose(6, 16.5, 10, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X and angle
  chassis.moveToPose(6, 32, 10, 1500, {.forwards = true, .maxSpeed = 50}, false); // flipped X and angle
  chassis.turnToHeading(70, 1000); // flipped angle
  pusher.extend(); // extend pusher
  chassis.moveToPose(100, -17, 70, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X and angle
  chassis.turnToHeading(-182, 1000, {}, false); // flipped angle
  chassis.moveToPose(28, -55, -182, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X and angle
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(28, -37, -182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(28, -43, -182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(300);
  chassis.moveToPose(28, -10, -182, 1200, {.forwards = false, .maxSpeed = 127}, false); // flipped X and angle
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {}, false); // turn to face goal (no flip needed)
  liftMotor.move(127); // lift up
  pros::delay(2000); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(28, 20, 0, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X
  intakeMotor.move(-127); // start intake out
}

void redRightTEST() {
  chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(12.75, 16.5, 10, 1500, {.forwards = true, .minSpeed = 25}, false); // flipped X and angle
  chassis.moveToPose(12.75, 37, 10, 2000, {.forwards = true, .minSpeed = 30}, false); // flipped X and angle
  pusher.extend(); // extend pusher
  pros::delay(1000);
  chassis.turnToHeading(80, 1000, {.minSpeed = 70, .earlyExitRange = 3}); // flipped angle
  chassis.moveToPose(37.5, -35, -184, 3500, {.forwards = true, .minSpeed = 62}, false); // flipped X and angle
  pros::delay(1000);
  intakeMotor.move(127);
  //chassis.moveToPose(38, -43, -183.76, 1550, {.forwards = true, .minSpeed = 90}, false); // flipped X and angle
  pros::delay(1000);
  chassis.moveToPose(38, -18, -180, 1000, {.forwards = false, .minSpeed= 60}, true); // flipped X and angle
  intakeMotor.move(-120);
  pros::delay(900);
  intakeMotor.move(127);
  pusher.toggle();
  chassis.turnToHeading(30, 1000, {.minSpeed = 70}); // flipped angle
  pros::delay(1200);
  chassis.moveToPose(34, 8, 0, 1500, {.minSpeed = 80}, true); // flipped X
  liftMotor.move(127); // lift up
  pros::delay(1000); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(34, 30, 0, 1300, {.forwards = true, .minSpeed = 100}, false); // flipped X
  intakeMotor.move(-127); // start intake out
}

void redRightELIM() {
    chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(6, 16, -10, 1500, {.forwards = true, .minSpeed = 25}, false);
  chassis.moveToPose(10, 29, -10, 2000, {.forwards = true, .minSpeed = 30}, false);
  chassis.moveToPose(-18, 34.5, -41, 2000, {.forwards = true, .minSpeed = 30}, false);
  intakeMotor.move(-80);
  /*pusher.extend(); // extend pusher
  pros::delay(1000);
  chassis.turnToHeading(-80, 1000, {.minSpeed = 70, .earlyExitRange = 3}); // added exit conditions
  chassis.moveToPose(37.5, -35, 184, 3500, {.forwards = true, .minSpeed = 62}, false);
  pros::delay(1000);
  intakeMotor.move(127);
  chassis.moveToPose(38, -43, 183.76, 1550, {.forwards = true, .minSpeed = 90}, false);
  for(int i = 0; i < 4; i++) {
    chassis.moveToPose(37.5, -43, 183.2, 250, {.forwards = true, .minSpeed = 90}, false);
    chassis.moveToPose(37.5, -40, 183.2, 250, {.forwards = true, .minSpeed = 90}, false);
  }
  chassis.moveToPose(30, -30, 180, 1000, {.forwards = false, .minSpeed= 60}, true);
  intakeMotor.move(-120);
  pros::delay(900);
  intakeMotor.move(127);
  pusher.toggle();
  chassis.turnToHeading(20, 1000, {.minSpeed = 70});
  pros::delay(1200);
  chassis.moveToPose(34, 8, 0, 1500, {.minSpeed = 80}, false);
  liftMotor.move(127); // lift up
  pros::delay(1000); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(34, 30, 0, 1300, {.forwards = true, .minSpeed = 100}, true);
  intakeMotor.move(-127); // start intake out


  //chassis.turnToHeading(180,1000);
  chassis.moveToPose(-28, -33, -180, 3000, {.forwards = true, .minSpeed = 62}, false);
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(-28, -37, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(-28, -43, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(3000);
  intakeMotor.move(0);
  chassis.moveToPose(-30, -25, 181, 1200, {.forwards = false, .minSpeed = 127}, false);
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {.minSpeed = 127}, false); // turn to face goal
  liftMotor.move(127); // lift up
  pros::delay(1600); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(-28, 20, 0, 1300, {.forwards = true, .minSpeed = 127}, false);
  intakeMotor.move(-127); // start intake out*/
}

void redRightSKIPLOADER() {
  chassis.setPose(0, 0, 0); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(8, 16, 10, 2500, {.forwards = true, .minSpeed = 25}, false);
  chassis.moveToPose(11, 36, 0, 2750, {.forwards = true, .minSpeed = 30}, false);
  chassis.moveToPose(4, 0, 17, 2000, {.forwards = false, .minSpeed = 50}, false);
  chassis.moveToPose(37, 10, 0, 2500, {.forwards = true, .minSpeed = 50}, false);
  liftMotor.move(127); // lift up
  pros::delay(2000);
  liftMotor.move(5); // stop lift
  chassis.turnToHeading(0, 1000, {.minSpeed = 40}, false);
  chassis.moveToPose(34, 30, 0, 1500, {.forwards = true, .minSpeed = 50}, false);
  intakeMotor.move(-127); // start intake out
}

void blueLeft() {
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

void blueLeftTest() {
  chassis.setPose(0, 0, -10); // reset odometry
  intakeMotor.move(127); // start intake
  chassis.moveToPose(-6, 16.5, -10, 1500, {.forwards = true, .maxSpeed = 120}, false);
  chassis.moveToPose(-6, 32, -10, 1500, {.forwards = true, .maxSpeed = 50}, false);
  chassis.turnToHeading(-70, 1000, {.minSpeed = 127, .earlyExitRange = 3}); // added exit conditions
  pusher.extend(); // extend pusher
  chassis.moveToPose(-28, -39, 181, 2000, {.forwards = true, .minSpeed = 115}, false);
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(-28, -37, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(-28, -43, 182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(300);
  chassis.moveToPose(-28, -10, 182, 1200, {.forwards = false, .minSpeed = 127}, false);
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {.minSpeed = 127}, false); // turn to face goal
  liftMotor.move(127); // lift up
  pros::delay(1600); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(-28, 20, 0, 1300, {.forwards = true, .minSpeed = 127}, false);
  intakeMotor.move(-127); // start intake out
}

void blueRight() {
  chassis.setPose(0, 0, 10); // reset odometry (flipped angle)
  intakeMotor.move(127); // start intake
  chassis.moveToPose(6, 16.5, 10, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X and angle
  chassis.moveToPose(6, 32, 10, 1500, {.forwards = true, .maxSpeed = 50}, false); // flipped X and angle
  chassis.turnToHeading(70, 1000); // flipped angle
  pusher.extend(); // extend pusher
  chassis.moveToPose(100, -17, 70, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X and angle
  chassis.turnToHeading(-182, 1000, {}, false); // flipped angle
  chassis.moveToPose(28, -55, -182, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X and angle
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(28, -37, -182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(28, -43, -182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(300);
  chassis.moveToPose(28, -10, -182, 1200, {.forwards = false, .maxSpeed = 127}, false); // flipped X and angle
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {}, false); // turn to face goal (no flip needed)
  liftMotor.move(127); // lift up
  pros::delay(2000); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(28, 20, 0, 1500, {.forwards = true, .maxSpeed = 127}, false); // flipped X
  intakeMotor.move(-127); // start intake out
}

void blueRightTEST() {
  chassis.setPose(0, 0, 10); // reset odometry (flipped angle)
  intakeMotor.move(127); // start intake
  chassis.moveToPose(6, 16.5, 10, 1500, {.forwards = true, .maxSpeed = 120}, false); // flipped X and angle
  chassis.moveToPose(6, 32, 10, 1500, {.forwards = true, .maxSpeed = 50}, false); // flipped X and angle
  chassis.turnToHeading(70, 1000, {.minSpeed = 127, .earlyExitRange = 3}); // flipped angle
  pusher.extend(); // extend pusher
  chassis.moveToPose(28, -41, -178, 2000, {.forwards = true, .minSpeed = 115}, false); // flipped X and angle
  //for (int i = 0; i < 3; i++)
  //{
    //chassis.moveToPose(28, -37, -182, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.moveToPose(28, -43, -182, 1000, {.forwards = true, .maxSpeed = 127}, false);
  //}
  pros::delay(300);
  chassis.moveToPose(28, -10, -182, 1200, {.forwards = false, .minSpeed = 127}, false); // flipped X and angle
  pusher.retract(); // retract pusher
  chassis.turnToHeading(0, 1000, {.minSpeed = 127}, false); // turn to face goal (no flip needed)
  liftMotor.move(127); // lift up
  pros::delay(1600); // wait for lift to finish
  liftMotor.move(5); // stop lift
  chassis.moveToPose(28, 20, 0, 1300, {.forwards = true, .minSpeed = 127}, false); // flipped X
  intakeMotor.move(-127); // start intake
}

void autonTest() {
  chassis.setPose(0, 0, 0); // reset odometry
  //chassis.moveToPose(0, 48, 0, 5000, {});
  //pros::delay(4000);
}

void autonomous() {
  liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set lift brake mode to hold

  //redLeft(); // run red left autonomous routine
  redLeftTEST(); // run red left autonomous routine test
  //redRight(); // run red right autonomous routine
  //redRightTEST(); // run red right autonomous routine test
  //redRightELIM();
  //blueLeft(); // run blue left autonomous routine
  //blueLeftTEST(); // run blue left autonomous routine test
  //blueRight(); // run blue right autonomous routine
  //blueRightTEST(); // run blue right autonomous routine test
  //autonTest(); // run auton test routine
  //skillsAuton(); // run skills autonomous routine
  //redRightSKIPLOADER();
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