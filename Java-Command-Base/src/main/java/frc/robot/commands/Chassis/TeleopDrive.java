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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_chassis.m_drive.setMaxOutput(0.5);
    if (this.m_joystick.getR2Button()) {
      this.m_chassis.m_drive.setMaxOutput(0.5 + this.m_joystick.getR2Axis() * 0.5);
    }
    this.m_chassis.m_drive.arcadeDrive(
        -this.m_joystick.getRawAxis(0),
        -this.m_joystick.getRawAxis(1),
        true);
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
