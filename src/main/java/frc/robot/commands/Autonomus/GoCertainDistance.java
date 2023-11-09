package frc.robot.commands.Autonomus;

import frc.robot.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoCertainDistance extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ChassisSubsystem m_chassis;
  private final double distance;

  public GoCertainDistance(ChassisSubsystem m_chassis, double distance) {
    this.m_chassis = m_chassis;
    this.distance = distance;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {
    m_chassis.leftPidController.setSetpoint(m_chassis.leftEncoder.getPosition() + distance);
  }

  @Override
  public void execute() {
    System.out.println("GoCertainDistance Running");
    double inVal = m_chassis.leftPidController.calculate(m_chassis.leftEncoder.getPosition());
    m_chassis.m_drive.arcadeDrive(inVal, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_chassis.m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(distance - m_chassis.leftEncoder.getPosition()) > distance){
      return true;
    }

    return false;
  }
}
