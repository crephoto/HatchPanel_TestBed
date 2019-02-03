/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_robotDrive.SetSafetyEnabled(false);
  // frc::SmartDashboard::PutNumber("Front left encoder");
  frc::Shuffleboard::GetTab("Sensors").Add("Front left drive", fl_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Sensors").Add("Rear left drive", rl_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Sensors").Add("Front right drive", fr_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Sensors").Add("Rear right drive", rr_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Sensors").Add("Gyro Angle", gyro.GetAngle());
  frc::Shuffleboard::GetTab("Light").Add("Light", 0);//.GetEntry();
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
  frc::SmartDashboard::PutNumber("Front left drive", fl_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Rear left drive", rl_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Front right drive", fr_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Rear right drive", rr_encoder.GetPosition());
  
  frc::SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
  try{
    //mecanum drive
    m_robotDrive.DriveCartesian(deadBand(m_Xbox.GetRawAxis(0)), -deadBand(m_Xbox.GetRawAxis(1)), deadBand(m_Xbox.GetRawAxis(4)), -(gyro.GetAngle()));
  }
  catch(std::exception ex){
    std::string err_string = "Error communicating with Drive System:  ";
                err_string += ex.what();
    DriverStation::ReportError(err_string.c_str());

  }  

  //A button (hatch panel pneumatics)
  if (m_Xbox.GetAButton()) {
    hatchPanel.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else {
    hatchPanel.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  //X, Y, B button (testing)
  /*double inputPercent;
  if (m_Xbox.GetXButton()) {
    inputPercent = 1.0;
  }
  else if (m_Xbox.GetYButton()) {
    inputPercent = 0.5;
  }
  else if (m_Xbox.GetBButton()) {
    inputPercent = 0.25;
  }
  else {
    inputPercent = 0.0;
  }*/
  /*frontLeft.Set(inputPercent);
  rearLeft.Set(inputPercent);
  frontRight.Set(-inputPercent);
  rearRight.Set(-inputPercent);*/

  //triggers (ball grabber)
 /*if (m_Xbox.GetRawAxis(2)>DEADBAND) {
   ballMotor.Set(ControlMode::PercentOutput, m_Xbox.GetRawAxis(2));
  }
  else if (m_Xbox.GetRawAxis(3)>DEADBAND) {
     ballMotor.Set(ControlMode::PercentOutput, -m_Xbox.GetRawAxis(3));
    }
  else {
     ballMotor.Set(ControlMode::PercentOutput, 0.0);
    }*/

    //triggers (lift motor)
      if (m_Xbox.GetRawAxis(2)>DEADBAND) {
   liftMotor.Set(ControlMode::PercentOutput, m_Xbox.GetRawAxis(2));
  }
  else if (m_Xbox.GetRawAxis(3)>DEADBAND) {
     liftMotor.Set(ControlMode::PercentOutput, -m_Xbox.GetRawAxis(3));
    }
  else {
     liftMotor.Set(ControlMode::PercentOutput, 0.0);
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

  //ballHatchLight
    /*frc::SendableChooser<class T>;
      .AddDefault("Do Nothing", ballHatchLight.Set(frc::Relay::Value::kOff));
		frc::SendableChooser<class T>;
      .AddObject("Hatch Panel", ballHatchLight.Set(frc::Relay::Value::kOn));
    frc::SendableChooser<class T>;
      .AddObject("Ball", ballHatchLight.Set(frc::Relay::Value::kOff));*/
    //frc::SmartDashboard::GetNumber("Light", 0);
    if (m_Xbox.GetXButton()) {
      ballHatchLight.Set(frc::Relay::Value::kOn);
      frc::SmartDashboard::PutString("Light", "on");
    }
    else if (m_Xbox.GetYButton()) {
      ballHatchLight.Set(frc::Relay::Value::kOff);
      frc::SmartDashboard::PutString("Light", "off");
    }
    /*else {
      ballHatchLight.Set(frc::Relay::Value::kOff);
      frc::SmartDashboard::PutString("Light", "off");
    }*/
    

    //need shuffleboard input to change lights
    //frc::Shuffleboard::
    //frc::SendableChooser<class T>.Add("Light", 0);
   // NetworkTableEntry("Light", 0);
   // frc::SmartDashboard::("Light", mySendable;
    //ballHatchLight.Set();
  

}

void Robot::TestPeriodic() {}

  double Robot::deadBand(double val) {
  double newVal;
  if (val > -DEADBAND && val < DEADBAND) {
    return 0.0;
  }
  if (val != 0) {
   newVal = pow(val, 2);
   printf("newVal: %f", newVal);
  } 
  if (val >= 0) {
    return newVal;
  }
  else {
    return -newVal;
  }

	//return val;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
