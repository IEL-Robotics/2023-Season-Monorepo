// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.ArmLower;
import frc.robot.commands.Arm.ArmRaise;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Gripper.GripperBackward;
import frc.robot.commands.Gripper.GripperForward;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gripper;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed


  // Driver Joystick Definitions
  private final PS4Controller m_driver_controller = new PS4Controller(OperatorConstants.kDriverControllerPort);

  // Subsystem Definitions
  public final Chassis m_chassis = new Chassis();
  public final Gripper m_gripper = new Gripper();

  // Command definitions
  private final Command m_teleopCommand = new TeleopDrive(m_chassis, m_driver_controller);
  private final GripperForward m_gripper_forward = new GripperForward(m_gripper);
  private final GripperBackward m_gripper_backward = new GripperBackward(m_gripper);

  private final Arm m_arm = new Arm();
  private final ArmRaise m_raise_arm  = new ArmRaise(m_arm);
  private final ArmLower m_lower_arm = new ArmLower(m_arm);

  public final PneumaticsControlModule m_pcm = new PneumaticsControlModule(31);
  public final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    new JoystickButton(m_driver_controller, PS4Controller.Button.kCircle.value).onTrue(m_gripper_forward);
    new JoystickButton(m_driver_controller, PS4Controller.Button.kSquare.value).onTrue(m_gripper_backward);
    new JoystickButton(m_driver_controller, PS4Controller.Button.kR1.value).whileTrue(m_raise_arm);
    new JoystickButton(m_driver_controller, PS4Controller.Button.kL1.value).whileTrue(m_lower_arm);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  String trajectoryJSON = "paths/YourPath.wpilib.json";
  Trajectory trajectory = new Trajectory();
  public Command getAutonomousCommand() {

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    var autoVoltageConstraint =

        new DifferentialDriveVoltageConstraint(

            new SimpleMotorFeedforward(

                DriveConstants.ksVolts,

                DriveConstants.kvVoltSecondsPerMeter,

                DriveConstants.kaVoltSecondsSquaredPerMeter),

            DriveConstants.kDriveKinematics,

            10);

    // Create config for trajectory

    TrajectoryConfig config =

        new TrajectoryConfig(

            DriveConstants.kMaxSpeedMetersPerSecond,

            DriveConstants.kMaxAccelerationMetersPerSecondSquared)

            // Add kinematics to ensure max speed is actually obeyed

            .setKinematics(DriveConstants.kDriveKinematics)

            // Apply the voltage constraint

            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.

    

    RamseteCommand ramseteCommand =

        new RamseteCommand(

            trajectory,

            m_chassis::getPose,

            new RamseteController(DriveConstants.kRamseteB,DriveConstants.kRamseteZeta),

            new SimpleMotorFeedforward(

                DriveConstants.ksVolts,

                DriveConstants.kvVoltSecondsPerMeter,

                DriveConstants.kaVoltSecondsSquaredPerMeter),

            DriveConstants.kDriveKinematics,

            m_chassis::getWheelSpeeds,

            new PIDController(DriveConstants.kPDriveVel, 0, 0),

            new PIDController(DriveConstants.kPDriveVel, 0, 0),

            // RamseteCommand passes volts to the callback

            m_chassis::tankDriveVolts,

            m_chassis);

    // Reset odometry to the starting pose of the trajectory.

    m_chassis.reset_odometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.

    return ramseteCommand.andThen(() -> m_chassis.tankDriveVolts(0, 0));
  }

  public Command getTeleopCommand() {
    m_pcm.enableCompressorDigital();
    m_compressor.enableDigital();
    return m_teleopCommand;
  }
}
