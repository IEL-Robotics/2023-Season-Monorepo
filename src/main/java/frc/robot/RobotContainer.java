package frc.robot;

import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final PS4Controller driverController = new PS4Controller(0);
  public final VisionSubsystem m_vision = new VisionSubsystem();

  public final ChassisSubsystem m_chassisSubsystem = new ChassisSubsystem();

  private final TeleopDrive m_teleopDrive = new TeleopDrive(m_chassisSubsystem, driverController, m_vision);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
  }

  public Command getTeleopCommand() {
    System.out.println("Test - Teleop Working");
    return m_teleopDrive;
  }

  public Command getAutonomousCommand() {
    return m_teleopDrive;
  }
}
