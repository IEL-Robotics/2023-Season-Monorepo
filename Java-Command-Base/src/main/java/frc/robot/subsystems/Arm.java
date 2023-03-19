// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final CANSparkMax m_arm_master, m_arm_slave;

  public Arm() {
    m_arm_master = new CANSparkMax(RobotConstants.kArmRightMotorPort, MotorType.kBrushless);
    m_arm_slave = new CANSparkMax(RobotConstants.kArmLeftMotorPort, MotorType.kBrushless);
    m_arm_slave.follow(m_arm_master);
    m_arm_master.setIdleMode(IdleMode.kBrake);
    m_arm_slave.setIdleMode(IdleMode.kBrake);
  }

  public void arm_raise() {
    this.m_arm_master.set(0.7);
  }

  public void arm_lower() {
    this.m_arm_master.set(-0.7);
  }

  public void arm_hold() {
    this.m_arm_master.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
