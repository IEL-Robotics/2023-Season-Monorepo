// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // TODO: Check angle calculation
    double x_ax = this.m_joystick.getRawAxis(2), y_ax = this.m_joystick.getRawAxis(5),
        yaw = Math.toRadians(this.m_chassis.m_gyro.getYaw()), z_ax = this.m_joystick.getRawAxis(0);
    double cosYaw = Math.cos(yaw);
    double sinYaw = Math.sin(yaw);
    x_ = x_ax * cosYaw - y_ax * sinYaw;

    y_ = x_ax * sinYaw
        + y_ax * cosYaw;

    SmartDashboard.putNumber("Joystick X", x_ax);
    SmartDashboard.putNumber("Joystick y", y_ax);
    SmartDashboard.putNumber("Joystick Rotation", z_ax);
    SmartDashboard.putNumber("Computed x", x_);
    SmartDashboard.putNumber("Computed y", y_);
    SmartDashboard.putNumber("Yaw", yaw);

    m_chassis.drive_mid_motor(x_, false);
    this.m_chassis.m_drive.arcadeDrive(
        z_ax,
        y_);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_chassis.m_drive.setMaxOutput(1.0);
    this.m_chassis.m_drive.arcadeDrive(0, 0);
    this.m_chassis.drive_mid_motor(x_, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
