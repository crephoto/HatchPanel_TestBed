/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/WPILib.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

// Pnuematic Hatch Panel
#define EXTEND 1
#define RETRACT 0

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  double deadBand(double);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";

  // Motor IDS
  //static const int front_left_id = 2, rear_left_id = 5, front_right_id = 1, rear_right_id = 4;
  #define BRUSHLESS rev::CANSparkMax::MotorType::kBrushless

  #define HINGE_INTAKE_POSITION 2180 // For injecting cargo ball
  #define HINGE_EJECT_POSITION 1090 // For ejecting cargo ball
  #define HINGE_CLOSED_POSITION 0 // Starting configuration

  std::string m_autoSelected;
  frc::DoubleSolenoid hatchPanel{0,1};
  frc::XboxController m_Xbox{0};

  // Drive base motor
  rev::CANSparkMax frontLeft{2, BRUSHLESS};
  rev::CANSparkMax rearLeft{5, BRUSHLESS};
  rev::CANSparkMax frontRight{1, BRUSHLESS};
  rev::CANSparkMax rearRight{4, BRUSHLESS};

  // Drive encoders
  rev::CANEncoder fl_encoder = frontLeft.GetEncoder();
  rev::CANEncoder rl_encoder = rearLeft.GetEncoder();
  rev::CANEncoder fr_encoder = frontRight.GetEncoder();
  rev::CANEncoder rr_encoder = rearRight.GetEncoder();

  // Subsystem motors
  WPI_TalonSRX ballMotor{6};
  WPI_TalonSRX hingeMotor{7};
  bool hingePIDMode;
  frc::MecanumDrive m_robotDrive{frontLeft, rearLeft, frontRight, rearRight};
};
