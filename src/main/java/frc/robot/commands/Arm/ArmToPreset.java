package frc.robot.commands.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PistonSubsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToPreset extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private final PistonSubsystem m_pistons;

  private double goalVal;

  public ArmToPreset(ArmSubsystem m_arm, PistonSubsystem m_pistons, double goalVal) {
    this.m_arm = m_arm;
    this.m_pistons = m_pistons;

    addRequirements(m_arm);
    addRequirements(m_pistons);

    this.goalVal = goalVal;
  }

  @Override
  public void initialize() {
    m_arm.leftPID.reset();
    m_arm.leftPID.setSetpoint(goalVal);
  }

  @Override
  public void execute() {
    double supposedOutput = m_arm.leftPID.calculate(m_arm.m_LeftMotor.getSelectedSensorPosition());

    if(Math.abs(supposedOutput) >= ArmConstants.maxSpeedPid){supposedOutput = (supposedOutput >= 0) ? ArmConstants.maxSpeedPid : -ArmConstants.maxSpeedPid;}

    m_arm.m_LeftMotor.set(TalonSRXControlMode.PercentOutput, supposedOutput);

    System.out.println("ARM TO PRESET: EXECUTE");

  }

  @Override
  public void end(boolean interrupted) {
    m_arm.m_LeftMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {

    // Serious question tho, is that how we end commands? EDIT: Yesss it is :d
    System.out.println("ARM TO PRESET: IS-FINISHED???");

    if(Math.abs(goalVal - m_arm.m_LeftMotor.getSelectedSensorPosition()) < ArmConstants.errorRange){
      
      if(goalVal > 650){m_pistons.ankleOpen();} // might also be vice versa
      else if(goalVal < -1500){m_pistons.ankleClose();}

      return true;
    }

    return false;
  }
}
