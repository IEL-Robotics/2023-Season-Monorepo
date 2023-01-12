// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  public final DifferentialDrive m_drive;
  
  /** Creates a new Chassis. */
  public Chassis() {
    CANSparkMax leftFront = new CANSparkMax(52, MotorType.kBrushless);
    CANSparkMax leftBack = new CANSparkMax(51, MotorType.kBrushless);
    CANSparkMax rightBack = new CANSparkMax(53, MotorType.kBrushless);
    CANSparkMax rightFront =  new CANSparkMax(50, MotorType.kBrushless);
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    leftBack.close();
    rightBack.close();

    this.m_drive = new DifferentialDrive(leftFront, rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.m_drive.feed();
  }
}
