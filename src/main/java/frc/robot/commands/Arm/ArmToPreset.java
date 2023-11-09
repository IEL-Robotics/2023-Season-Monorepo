package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToPreset extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private final double goalValue;

  private final Timer timer = new Timer();
  private boolean isFinished = false;

  public ArmToPreset(ArmSubsystem m_arm, double goalValue) {
    this.goalValue = goalValue;
    this.m_arm = m_arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    isFinished = false; // Be very careful
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    SmartDashboard.putBoolean("StillExecuted", true);
    m_arm.rightPID.setSetpoint(goalValue);
    double supposedOutput = m_arm.rightPID.calculate(m_arm.m_RightMotor.getSelectedSensorPosition());

    if((m_arm.m_RightMotor.getSelectedSensorPosition() < 500) && supposedOutput > 0.75){supposedOutput = 1;} //bypass
    else if((m_arm.m_RightMotor.getSelectedSensorPosition() > 500) && supposedOutput > 0.75){supposedOutput = 0.75;}
    else if((m_arm.m_RightMotor.getSelectedSensorPosition() < 700) && supposedOutput < -0.5){supposedOutput = -0.5;}
    else if((m_arm.m_RightMotor.getSelectedSensorPosition() > 700) && supposedOutput < -0.75){supposedOutput = -1;}

    m_arm.armSet(supposedOutput);

    if(Math.abs(goalValue - m_arm.m_RightMotor.getSelectedSensorPosition())<40)
    {
      m_arm.armSet(0);
      isFinished = true;
      end(true);
    }

    System.out.println("PresetRunning: "+ timer.get());
    if(timer.get() > 3.5){
      isFinished = true;
      end(true);
    }

  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("StillExecuted", false);
    m_arm.armSet(0);
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
