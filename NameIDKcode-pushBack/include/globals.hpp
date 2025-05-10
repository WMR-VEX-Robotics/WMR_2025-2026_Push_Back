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
extern pros::MotorGroup intakeMotor;
extern pros::Motor wsMotor;

//pneumatics
extern pros::adi::Pneumatics solenoidClamp;
extern pros::adi::Pneumatics solenoidColorSort;
extern pros::adi::Pneumatics solenoidHang;

// Inertial Sensor
extern pros::Imu imu;

//Color Sensor
extern pros::Optical colorSensor;

//distance sensor
extern pros::Distance distanceSensor;