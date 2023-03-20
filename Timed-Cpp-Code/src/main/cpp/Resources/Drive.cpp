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




    // set PID coefficients
    m_pidControllerL1.SetP(kP);
    m_pidControllerL1.SetI(kI);
    m_pidControllerL1.SetD(kD);
    m_pidControllerL1.SetIZone(kIz);
    m_pidControllerL1.SetFF(kFF);
    m_pidControllerL1.SetOutputRange(kMinOutput, kMaxOutput);
    
    
    m_pidControllerR1.SetP(kP);
    m_pidControllerR1.SetI(kI);
    m_pidControllerR1.SetD(kD);
    m_pidControllerR1.SetIZone(kIz);
    m_pidControllerR1.SetFF(kFF);
    m_pidControllerR1.SetOutputRange(kMinOutput, kMaxOutput);


    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);



    // button to toggle between velocity and smart motion modes
    frc::SmartDashboard::PutBoolean("Mode", true);
}
   void Robot::DrivePeriodic()
   {

        // read PID coefficients from SmartDashboard
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);



    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidControllerR1.SetP(p); kP = p; }
    if((i != kI)) { m_pidControllerR1.SetI(i); kI = i; }
    if((d != kD)) { m_pidControllerR1.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerR1.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerR1.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControllerR1.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

        // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidControllerL1.SetP(p); kP = p; }
    if((i != kI)) { m_pidControllerL1.SetI(i); kI = i; }
    if((d != kD)) { m_pidControllerL1.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerL1.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerL1.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControllerL1.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    double SetPoint = (PS4Controller.GetL2Axis()-PS4Controller.GetR2Axis())*.17;
    m_pidControllerL1.SetReference(SetPoint, rev::CANSparkMax::ControlType::kVelocity);
    m_pidControllerR1.SetReference(SetPoint, rev::CANSparkMax::ControlType::kVelocity);

    
    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_encoderL1.GetVelocity());

   if(!PS4Controller.GetCircleButton()){ 
              m_robotDrive.ArcadeDrive((PS4Controller.GetR2Axis()-PS4Controller.GetL2Axis())*.25,PS4Controller.GetLeftY()*.3,false);
    center_motor_lead.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,PS4Controller.GetLeftX()*.5);
    }
         
   else{ 
          m_robotDrive.ArcadeDrive((PS4Controller.GetR2Axis()-PS4Controller.GetL2Axis())*.6,PS4Controller.GetLeftY()*.6,false);
    center_motor_lead.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,PS4Controller.GetLeftX());
    }

    if(PS4Controller.GetTouchpadReleased()){
        flip = !flip; 
    }
    
}