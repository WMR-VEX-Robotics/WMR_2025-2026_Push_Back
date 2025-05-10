#include "pros/rtos.hpp"
#include <type_traits> // IWYU pragma: keep
#pragma region ChassisBuild
#include "main.h"
#include "globals.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"


// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(18);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(16);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.083);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(
    6.8,  // slightly lower kP
    0,    
    9,    
    3,
    1,
    250,
    3,
    500,
    30  // softer acceleration
);

// angular motion controller
lemlib::ControllerSettings angularController(
    3.8,   // ↓ kP to reduce aggressive turning
    0.0,   // Remove kI (not useful here unless you see drift)
    30,    // ↑ kD slightly to add more braking/damping
    2,     // anti-windup lower (I is gone)
    2,     // keep tighter small error range
    700,   // faster settling time
    5,     // slightly tighter large error range
    900,   // slightly faster large error timeout
    35     // slightly softer slew for smoother approach
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0, 0, 0);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
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

}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

#pragma endregion pre-auton

#pragma region autonomous

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */



#pragma endregion autonomous

#pragma region autoncallback

void autonomous() {
    
}

#pragma endregion autoncallback


#pragma region userControl

/**
 * Runs in driver control
 */
void opcontrol() {
    wsMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // loop to continuously update motors

    while (true) {
        

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        

		// delay to save resources
        pros::delay(10);
        
    }
}

#pragma endregion userControl