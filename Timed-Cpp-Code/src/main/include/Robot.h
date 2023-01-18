// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/PS4Controller.h>
#include <frc/controller/PIDController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  void DriveInit();
  void DrivePeriodic();



 private:
  static const int leftLeadDeviceID = 51, leftFollowDeviceID = 52, rightLeadDeviceID = 50, rightFollowDeviceID = 53, centerMotorLeadID = 2, centerMotorFollowID = 1;
  // Add motors
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  ctre::phoenix::motorcontrol::can::TalonSRX     center_motor_lead{centerMotorLeadID};
  ctre::phoenix::motorcontrol::can::TalonSRX     center_motor_follow{centerMotorFollowID};
  bool flip = false;
  // Add encoders 
  rev::SparkMaxRelativeEncoder m_encoderL1 = m_leftLeadMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderR1 = m_rightLeadMotor.GetEncoder();

  rev::SparkMaxPIDController m_pidControllerL1 = m_leftLeadMotor.GetPIDController();
  rev::SparkMaxPIDController m_pidControllerR1 = m_rightLeadMotor.GetPIDController();


  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::PS4Controller PS4Controller{0};
};
