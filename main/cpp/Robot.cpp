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

#include "Constants.h"


void Robot::RobotInit() {
  hingePIDMode = false;
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  // frc::SmartDashboard::PutNumber("Front left encoder");
  frc::Shuffleboard::GetTab("Encoders").Add("Front Left", fl_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Rear Left", rl_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Front Right", fr_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Rear Right", rr_encoder.GetPosition());
  frc::Shuffleboard::GetTab("Encoders").Add("Hinge", Robot::hingeMotor.Get() );
  // frc::Shuffleboard::GetTab("Encoders").Add("Front left encoder velocity", fl_encoder.GetVelocity());
  // PID Hinge Configuration
/* lets grab the 360 degree position of the MagEncoder's absolute position */
		int absolutePosition = hingeMotor.GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		hingeMotor.SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		// Reset the starting configuration to zero
    // hingeMotor.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		/* choose the sensor and sensor direction */
		hingeMotor.ConfigSelectedFeedbackSensor(
				//FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
				FeedbackDevice::QuadEncoder, kPIDLoopIdx, kTimeoutMs);
		hingeMotor.SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		hingeMotor.ConfigNominalOutputForward(0, kTimeoutMs);
		hingeMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
		hingeMotor.ConfigPeakOutputForward(1, kTimeoutMs);
		hingeMotor.ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		hingeMotor.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		hingeMotor.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		hingeMotor.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		hingeMotor.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
    hingeMotor.ConfigClosedloopRamp(.2); // Seconds from neutral to full
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
  hingePIDMode = false;
}

#define DEADBAND 0.1
void Robot::TeleopPeriodic() {
  //mecanum drive
  // m_robotDrive.DriveCartesian(deadBand(m_Xbox.GetRawAxis(0)), deadBand(-m_Xbox.GetRawAxis(1)), deadBand(m_Xbox.GetRawAxis(4)));
  // Disable Drive
  m_robotDrive.DriveCartesian(0.0, 0.0, 0.0);

  frc::SmartDashboard::PutNumber("Front Left", fl_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Front left encoder velocity", fl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Rear Left", rl_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Rear left encoder velocity", rl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Front Right", fr_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Front right encoder velocity", fr_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Rear Right", rr_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Rear right encoder velocity", rr_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Hinge", Robot::hingeMotor.Get());
  
  //A button (hatch panel pneumatics)
  //if (m_Xbox.GetAButton()) {
  if (m_Xbox.GetRawButton(1)) {
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
     hingeMotor.Set(ControlMode::PercentOutput, .25);
  }
  else if (m_Xbox.GetRawButton(6)) {
     hingeMotor.Set(ControlMode::PercentOutput, -.5);
  }
  else if (m_Xbox.GetRawButton(7)) {
     hingeMotor.Set(ControlMode::Position, HINGE_INTAKE_POSITION);
     printf("Setting Hinge Position to %d\n", HINGE_INTAKE_POSITION);
     //hingeMotor.Set(ControlMode::Velocity, 140.0);
     // printf("Setting Hinge Velocity to %f\n", 140.0);
  }
  else if (m_Xbox.GetRawButton(8)) {
     hingeMotor.Set(ControlMode::Position, HINGE_CLOSED_POSITION);
     printf("Setting Hinge Position to %d\n", HINGE_CLOSED_POSITION);
     //hingeMotor.Set(ControlMode::Velocity, -140.0);
     //printf("Setting Hinge Velocity to %f\n", -140.0);
  }
  else {
		int currentPosition = hingeMotor.GetSelectedSensorPosition(0);
     hingeMotor.Set(ControlMode::Position, currentPosition);
  }
		// Reset the starting configuration to zero 
    /*
  else if (m_Xbox.GetRawButton(10)) {
      hingeMotor.Set(ControlMode::PercentOutput, 0.0);
    }
  */
}

void Robot::TestPeriodic() {
  frc::SmartDashboard::PutNumber("Front Left", fl_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Front left encoder velocity", fl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Rear Left", rl_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Rear left encoder velocity", rl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Front Right", fr_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Front right encoder velocity", fr_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Rear Right", rr_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Rear right encoder velocity", rr_encoder.GetVelocity());
  // frc::SmartDashboard::PutString("Hinge Motor", hingeMotor.GetDescription());

}


double Robot::deadBand(double val) {
  if (val > -DEADBAND && val < DEADBAND)
     return 0.0;

	return val;
}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
