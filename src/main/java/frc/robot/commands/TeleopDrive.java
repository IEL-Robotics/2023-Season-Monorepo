package frc.robot.commands;

import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ChassisSubsystem m_chassis;
  private final PS4Controller joystick;

  public TeleopDrive(ChassisSubsystem m_chassis, PS4Controller joystick) {
    this.m_chassis = m_chassis;
    this.joystick = joystick;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    System.out.println("Debug Issue - TeleopCommand Working CONTINUOUSLY!!");

    m_chassis.m_drive.setMaxOutput(ChassisConstants.lowerOutput);

    if(joystick.getR1Button()){
      m_chassis.m_drive.setMaxOutput(ChassisConstants.higherOutput);
    }

    m_chassis.m_drive.arcadeDrive(-joystick.getRawAxis(1), -joystick.getRawAxis(0));
  }

  @Override
  public void end(boolean interrupted) {
    m_chassis.m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {return false;}
}
