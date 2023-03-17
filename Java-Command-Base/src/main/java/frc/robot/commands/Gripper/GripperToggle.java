// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GripperToggle extends CommandBase {
  private final Gripper m_gripper;

  /** Creates a new GripperToggle. */
  public GripperToggle(Gripper m_gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_gripper);
    this.m_gripper = m_gripper;
    m_gripper.sld_fwd();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.m_gripper.sld_toggle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
