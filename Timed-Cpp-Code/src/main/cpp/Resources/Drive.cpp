#include "Robot.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>


void Robot::DriveInit()
{
    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();

    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
    center_motor_follow.Follow(center_motor_lead);

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

  void Robot::DrivePeriodic() {
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
   
   if(!PS4Controller.GetCircleButton()){ 
    // if (!flip) m_robotDrive.ArcadeDrive(PS4Controller.GetLeftX()*.2,PS4Controller.GetLeftY()*.2,false);
    // else
          m_robotDrive.ArcadeDrive(PS4Controller.GetLeftX()*.17,(PS4Controller.GetR2Axis()-PS4Controller.GetL2Axis())*.17,false);
    center_motor_lead.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput
,PS4Controller.GetRightX() );
    }
         
   else{ 
    // if (!flip) m_robotDrive.ArcadeDrive(PS4Controller.GetLeftX()*.4,PS4Controller.GetLeftY()*.4,false);
    // else
          m_robotDrive.ArcadeDrive(PS4Controller.GetLeftX()*.4,(PS4Controller.GetR2Axis()-PS4Controller.GetL2Axis())*.4,false);
    center_motor_lead.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput
,PS4Controller.GetRightX() );
    }

    if(PS4Controller.GetTouchpadReleased()){
        flip = !flip;
    }
    
}