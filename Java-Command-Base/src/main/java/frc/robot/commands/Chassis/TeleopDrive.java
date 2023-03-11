// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class TeleopDrive extends CommandBase {
  private final PS4Controller m_joystick;
  private final Chassis m_chassis;

  /** Creates a new Drive. */
  public TeleopDrive(Chassis m_chassis, PS4Controller m_joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
    this.m_chassis = m_chassis;
    this.m_joystick = m_joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  double x_, y_;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean pressed = false;
    this.m_chassis.m_drive.setMaxOutput(0.5);
    if (this.m_joystick.getR1Button()) {
      pressed = true;
      this.m_chassis.m_drive.setMaxOutput(1);
    }

    // TODO: Check axises
    double x_ax = this.m_joystick.getRawAxis(2), y_ax = this.m_joystick.getRawAxis(0),
        angle = this.m_chassis.m_gyro.getAngle() % 360;
    x_ = x_ax * Math.cos(angle) - y_ax * Math.sin(angle);

    y_ = x_ax * Math.sin(angle)
        + y_ax * Math.cos(angle);

    m_chassis.drive_mid_motor(x_, pressed);
    this.m_chassis.m_drive.arcadeDrive(
        y_,
        this.m_joystick.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_chassis.m_drive.setMaxOutput(1.0);
    this.m_chassis.m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
