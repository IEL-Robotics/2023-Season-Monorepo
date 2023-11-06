package frc.robot.commands.Chassis;

import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ChassisSubsystem m_chassis;
  private final PS4Controller joystick;
  private final VisionSubsystem m_vision;

  public TeleopDrive(ChassisSubsystem m_chassis, PS4Controller joystick, VisionSubsystem m_vision) {
    this.m_chassis = m_chassis;
    this.joystick = joystick;
    this.m_vision = m_vision;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_chassis.m_drive.setMaxOutput(ChassisConstants.lowerOutput);

     if(joystick.getR1Button()){
       m_chassis.m_drive.setMaxOutput(ChassisConstants.higherOutput);
     }

    SmartDashboard.putNumber("X Value Controller", joystick.getLeftX());
    SmartDashboard.putNumber("Y Value Controller", joystick.getLeftY());
    SmartDashboard.putBoolean("L1", joystick.getL1Button());
    SmartDashboard.putNumber("L2", joystick.getL2Axis());
    SmartDashboard.putBoolean("R1", joystick.getR1Button());
    SmartDashboard.putNumber("R2", joystick.getR2Axis());

    //m_chassis.m_drive.arcadeDrive(-joystick.getRawAxis(1), -joystick.getRawAxis(0));

    m_chassis.m_drive.arcadeDrive(((joystick.getR2Axis()+1)/2) - ((joystick.getL2Axis()+1)/2), -joystick.getRawAxis(0));

    SmartDashboard.putNumber("Tag ID", m_vision.getTagID());

    m_vision.getFieldPosition();

  }

  @Override
  public void end(boolean interrupted) {
    m_chassis.m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {return false;}
}
