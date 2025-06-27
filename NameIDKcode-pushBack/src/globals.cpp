#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-18, -20, -19},pros::MotorGearset::blue); // left motor group 
pros::MotorGroup rightMotors({13, 12, 11}, pros::MotorGearset::blue); // right motor group

//motors
pros::Motor intakeMotor(9, pros::v5::MotorGearset::blue);
pros::Motor liftMotor(7, pros::v5::MotorGears::red);

//pneumatics
pros::adi::Pneumatics pto('D', false);
pros::adi::Pneumatics odom('E', false);
pros::adi::Pneumatics pusher('F', false);

// Inertial Sensor on port 9
pros::Imu imu(7);

//Color Sensor 
pros::Optical colorSensor(2);

//distance sensor
pros::Distance distanceSensor(12);
