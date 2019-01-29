/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/shuffleboard/Shuffleboard.h>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  // frc::SmartDashboard::PutNumber("Front left encoder");
  frc::Shuffleboard::GetTab("Encoders").Add("Front left encoder position", fl_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Rear left encoder position", rl_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Front right encoder position", fr_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Rear right encoder position", rr_encoder.GetPosition());
  //frc::Shuffleboard::GetTab("Encoders").Add("Hinge motor control mode", hingeMotor.GetControlMode());
  frc::Shuffleboard::GetTab("Encoders").Add("Hinge motor Description", hingeMotor.GetDescription());
  // frc::Shuffleboard::GetTab("Encoders").Add("Front left encoder velocity", fl_encoder.GetVelocity());
  //frontRight.SetInverted(true);
  //rearRight.SetInverted(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
}

#define DEADBAND 0.1
void Robot::TeleopPeriodic() {
  //mecanum drive
  m_robotDrive.DriveCartesian(deadBand(m_Xbox.GetRawAxis(0)), deadBand(-m_Xbox.GetRawAxis(1)), deadBand(m_Xbox.GetRawAxis(4)));

  frc::SmartDashboard::PutNumber("Front left encoder position", fl_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Front left encoder velocity", fl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Rear left encoder position", rl_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Rear left encoder velocity", rl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Front right encoder position", fr_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Front right encoder velocity", fr_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Rear right encoder position", rr_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Rear right encoder velocity", rr_encoder.GetVelocity());
  frc::SmartDashboard::PutString("Hinge motor Description", hingeMotor.GetDescription());
  
  

  //A button (hatch panel pneumatics)
  if (m_Xbox.GetAButton()) {
    hatchPanel.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else {
    hatchPanel.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  //triggers (ball grabber)
  if (m_Xbox.GetRawAxis(2)>DEADBAND) {
    ballMotor.Set(ControlMode::PercentOutput, m_Xbox.GetRawAxis(2));
  }
  else if (m_Xbox.GetRawAxis(3)>DEADBAND) {
     ballMotor.Set(ControlMode::PercentOutput, -m_Xbox.GetRawAxis(3));
    }
  else {
     ballMotor.Set(ControlMode::PercentOutput, 0.0);
    }
  
  //bumpers (hinge)
  if (m_Xbox.GetRawButton(5)) {
     hingeMotor.Set(ControlMode::PercentOutput, 0.5);
  }
  else if (m_Xbox.GetRawButton(6)) {
     hingeMotor.Set(ControlMode::PercentOutput, -0.5);
  }
    else {
      hingeMotor.Set(ControlMode::PercentOutput, 0.0);
    }

}

void Robot::TestPeriodic() {}

double Robot::deadBand(double val) {
  if (val > -DEADBAND && val < DEADBAND)
     return 0.0;

  // if (val >= 0) {
  //   return pow(val, 1.25);
  // } else{
  //   return -pow(val, 1.25);
  // }

  return val;
	
}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
