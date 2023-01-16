// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Constants.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>




void Robot::RobotPeriodic() {}

  void Robot::RobotInit() {
    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();

    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);

    m_pidControllerL1.SetP(kP);
    m_pidControllerL1.SetI(kI);
    m_pidControllerL1.SetD(kD);
    m_pidControllerL1.SetIZone(kIz);
    m_pidControllerL1.SetFF(kFF);
    m_pidControllerL1.SetOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerL2.SetP(kP);
    m_pidControllerL2.SetI(kI);
    m_pidControllerL2.SetD(kD);
    m_pidControllerL2.SetIZone(kIz);
    m_pidControllerL2.SetFF(kFF);
    m_pidControllerL2.SetOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerR1.SetP(kP);
    m_pidControllerR1.SetI(kI);
    m_pidControllerR1.SetD(kD);
    m_pidControllerR1.SetIZone(kIz);
    m_pidControllerR1.SetFF(kFF);
    m_pidControllerR1.SetOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerR2.SetP(kP);
    m_pidControllerR2.SetI(kI);
    m_pidControllerR2.SetD(kD);
    m_pidControllerR2.SetIZone(kIz);
    m_pidControllerR2.SetFF(kFF);
    m_pidControllerR2.SetOutputRange(kMinOutput, kMaxOutput);
  }

  void Robot::TeleopPeriodic() {
        // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    if((p != kP)) { m_pidControllerL1.SetP(p); kP = p; }
    if((i != kI)) { m_pidControllerL1.SetI(i); kI = i; }
    if((d != kD)) { m_pidControllerL1.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerL1.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerL1.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControllerL1.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    m_pidControllerR1.SetSmartMotionMaxVelocity(MaxRPM); 
    m_pidControllerR1.SetSmartMotionMaxAccel(MaxAccel); 


    if((p != kP)) { m_pidControllerR1.SetP(p); kP = p; }
    if((i != kI)) { m_pidControllerR1.SetI(i); kI = i; }
    if((d != kD)) { m_pidControllerR1.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerR1.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerR1.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControllerR1.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    m_pidControllerR1.SetSmartMotionMaxVelocity(MaxRPM); 
    m_pidControllerR1.SetSmartMotionMaxAccel(MaxAccel); 


    double SetPoint = -(MaxRPM*PS4Controller.GetLeftX());

    m_pidControllerR1.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);
    m_pidControllerL1.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);
    m_robotDrive.ArcadeDrive(PS4Controller.GetLeftX()*.2,PS4Controller.GetLeftY()*.2,false);
  }
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}



void Robot::DisabledInit(){}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
