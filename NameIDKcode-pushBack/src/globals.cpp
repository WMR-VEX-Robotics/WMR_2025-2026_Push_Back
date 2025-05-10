#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-12, -20, -19},pros::MotorGearset::blue); // left motor group 
pros::MotorGroup rightMotors({5, 10, 11}, pros::MotorGearset::blue); // right motor group

//motors
pros::MotorGroup intakeMotor({-4, 13}, pros::v5::MotorGearset::green);
pros::Motor wsMotor(-17, pros::v5::MotorGears::green);

//pneumatics
pros::adi::Pneumatics solenoidClamp('F', true);
pros::adi::Pneumatics solenoidColorSort('E', false);
pros::adi::Pneumatics solenoidHang('G', false);

// Inertial Sensor on port 9
pros::Imu imu(9);

//Color Sensor 
pros::Optical colorSensor(2);

//distance sensor
pros::Distance distanceSensor(14);
