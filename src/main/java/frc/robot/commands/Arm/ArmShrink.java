package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmShrink extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;

  public ArmShrink(ArmSubsystem m_arm) {
    this.m_arm = m_arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.armShrink();
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.armSet(0);
  }

  @Override
  public boolean isFinished() {return false;}
}
