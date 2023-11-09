package frc.robot;

import frc.robot.commands.Arm.ArmLower;
import frc.robot.commands.Arm.ArmRaise;
import frc.robot.commands.Arm.ArmToPreset;
import frc.robot.commands.Autonomus.GoCertainDistance;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Piston.AnkleClose;
import frc.robot.commands.Piston.AnkleOpen;
import frc.robot.commands.Piston.GripperClose;
import frc.robot.commands.Piston.GripperOpen;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.PistonSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final PS4Controller driverController = new PS4Controller(0);
  private final PS4Controller copilotController = new PS4Controller(1);

  public final VisionSubsystem m_vision = new VisionSubsystem();

  public final ChassisSubsystem m_chassisSubsystem = new ChassisSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final PistonSubsystem m_piston = new PistonSubsystem();

  private final TeleopDrive m_teleopDrive = new TeleopDrive(m_chassisSubsystem, driverController, m_vision);
  private final ArmRaise m_armRaise = new ArmRaise(m_arm);
  private final ArmLower m_armLower = new ArmLower(m_arm);
  private final AnkleClose m_ankleClose = new AnkleClose(m_piston);
  private final AnkleOpen m_ankleOpen = new AnkleOpen(m_piston);
  private final GripperClose m_gripperClose = new GripperClose(m_piston);
  private final GripperOpen m_gripperOpen = new GripperOpen(m_piston);

  private final ArmToPreset m_firstPreset = new ArmToPreset(m_arm, -225);
  private final ArmToPreset m_secondPreset = new ArmToPreset(m_arm, 500);
  private final ArmToPreset m_thirdPreset = new ArmToPreset(m_arm, 744); 
  private final ArmToPreset m_fourthPreset = new ArmToPreset(m_arm, 1201);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    new JoystickButton(copilotController, 5).whileTrue(m_armLower);
    new JoystickButton(copilotController, 6).whileTrue(m_armRaise);
    new JoystickButton(driverController, 1).whileTrue(m_ankleClose); 
    new JoystickButton(driverController, 3).whileTrue(m_ankleOpen);
    new JoystickButton(driverController, 5).whileTrue(m_gripperClose); 
    new JoystickButton(driverController, 6).whileTrue(m_gripperOpen);
    new JoystickButton(copilotController, 1).onTrue(m_firstPreset);//asagi-kare
    new JoystickButton(copilotController, 2).onTrue(m_secondPreset);// hafif egik-x
    new JoystickButton(copilotController, 3).onTrue(m_thirdPreset); //dik-o 
    new JoystickButton(copilotController, 4).onTrue(m_fourthPreset);//kup birak-ucgen
}

  public Command optionalAutonomusFunc(){ // More can be easily added
    return new SequentialCommandGroup(
      m_fourthPreset,
      m_ankleOpen,
      m_gripperOpen,
      m_secondPreset,
      new GoCertainDistance(m_chassisSubsystem, 4.0)
      );
  }

  public Command getTeleopCommand() {
    System.out.println("Test - Teleop Working");
    return m_teleopDrive;
  }

  public Command getAutonomousCommand() {
    return optionalAutonomusFunc();
  }
}
