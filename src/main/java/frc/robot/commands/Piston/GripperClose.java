package frc.robot.commands.Piston;

import frc.robot.subsystems.PistonSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GripperClose extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PistonSubsystem m_pistons;
  private boolean isFinished = false;

  public GripperClose(PistonSubsystem m_pistons) {
    this.m_pistons = m_pistons;
  }

  @Override
  public void initialize() {
    isFinished = false; // Be very careful
  }

  @Override
  public void execute() {
    m_pistons.tempFwdGripper();
    isFinished = true;
    end(true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return isFinished;}
}

