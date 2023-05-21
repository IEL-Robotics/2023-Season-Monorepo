package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Arm.ArmExtend;
import frc.robot.commands.Arm.ArmLower;
import frc.robot.commands.Arm.ArmRaise;
import frc.robot.commands.Arm.ArmShrink;
import frc.robot.commands.Arm.ArmToPreset;
import frc.robot.commands.Autonomus.GoCertainDistance;
import frc.robot.commands.Pistons.AnkleClose;
import frc.robot.commands.Pistons.AnkleOpen;
import frc.robot.commands.Pistons.Grab;
import frc.robot.commands.Pistons.Leave;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.PistonSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final PS4Controller driverController = new PS4Controller(ControllerConstants.idMainController);
  private final PS4Controller copilotController = new PS4Controller(ControllerConstants.idCopilotController);

  public final ChassisSubsystem m_chassis = new ChassisSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final PistonSubsystem m_piston = new PistonSubsystem();

  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_chassis, driverController);
  private final ArmLower m_armLower = new ArmLower(m_arm);
  private final ArmRaise m_armRaise = new ArmRaise(m_arm);
  private final ArmExtend m_armExtend = new ArmExtend(m_arm);
  private final ArmShrink m_armShrink = new ArmShrink(m_arm);

  private final ArmToPreset m_armToGround = new ArmToPreset(m_arm, m_piston, ArmConstants.groundPos);
  private final ArmToPreset m_armUp = new ArmToPreset(m_arm, m_piston, ArmConstants.upPos);
  private final ArmToPreset m_armToGrid = new ArmToPreset(m_arm, m_piston, ArmConstants.gridPos);

  private final AnkleClose m_ankleClose = new AnkleClose(m_piston);
  private final AnkleOpen m_ankleOpen = new AnkleOpen(m_piston);
  private final Grab m_grab = new Grab(m_piston);
  private final Leave m_leave = new Leave(m_piston);

  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverController, 1).onTrue(m_armToGround);
    new JoystickButton(driverController, 4).onTrue(m_armUp);
    new JoystickButton(driverController, 3).onTrue(m_armToGrid);
    new JoystickButton(driverController, 5).onTrue(m_grab);
    new JoystickButton(driverController, 7).onTrue(m_leave);
    new JoystickButton(driverController, 6).onTrue(m_ankleOpen);
    new JoystickButton(driverController, 8).onTrue(m_ankleClose);

    new JoystickButton(copilotController, 5).whileTrue(m_armRaise);
    new JoystickButton(copilotController, 7).whileTrue(m_armLower);
    new JoystickButton(copilotController, 6).whileTrue(m_armExtend);
    new JoystickButton(copilotController, 8).whileTrue(m_armShrink);
  }

  public Command getAutonomousCommand() {
     return optionalAutonomusFunc(); // Change it depending on strategy
  }

  public Command getTeleopCommand() { // Wrote it exclusively and changed the Robot.java, might also not work. But god, i hope it does
    System.out.println("Debug Issue - TeleopCommand Working"); // EDIT: Yes it does
    return m_teleopCommand;
  }

  public Command optionalAutonomusFunc(){ // More can be easily added
    return new SequentialCommandGroup(
      m_armToGrid,
      m_leave,
      m_armUp,
      new GoCertainDistance(m_chassis, ChassisConstants.taxiDistance)
     );
  }
}
