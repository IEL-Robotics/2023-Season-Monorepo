package frc.robot.commands.Pistons;

import frc.robot.subsystems.PistonSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Grab extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PistonSubsystem m_pistons;

  public Grab(PistonSubsystem m_pistons) {
    this.m_pistons = m_pistons;
    addRequirements(m_pistons);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_pistons.gripperGrab();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return false;}
}

