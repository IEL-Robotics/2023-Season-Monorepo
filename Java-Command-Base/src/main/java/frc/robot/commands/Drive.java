// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Chassis;

public class Drive extends CommandBase {
  private final CommandPS4Controller m_joystick;
  private final Chassis m_chassis;
  /** Creates a new Drive. */
  public Drive(Chassis m_chassis, CommandPS4Controller m_joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
    this.m_chassis = m_chassis;
    this.m_joystick = m_joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_chassis.m_drive.setMaxOutput(0.8);
    this.m_chassis.m_drive.arcadeDrive(
        this.m_joystick.getRawAxis(0),
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
