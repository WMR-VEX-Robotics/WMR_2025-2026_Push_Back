#pragma once
#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include "pros/optical.hpp"


// controller
extern pros::Controller controller;

// motor groups
extern pros::MotorGroup leftMotors; // left motor group 
extern pros::MotorGroup rightMotors; // right motor group 

//motors
extern pros::Motor intakeMotor;
extern pros::Motor liftMotor;

//pneumatics
extern pros::adi::Pneumatics odom;
extern pros::adi::Pneumatics pto;
extern pros::adi::Pneumatics pusher;

// Inertial Sensor
extern pros::Imu imu;

//Color Sensor
extern pros::Optical colorSensor;

//distance sensor
//extern pros::Distance distanceSensor;