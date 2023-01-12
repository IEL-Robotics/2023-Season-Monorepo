/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
  void Robot::RobotInit() {

    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();
    

    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
  }

  void Robot::TeleopPeriodic() {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(PS4Controller.GetLeftY(), PS4Controller.GetLeftX());
  }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
