// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  private final DoubleSolenoid m_doublesolenoid  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  /** Creates a new Gripper. */
  public Gripper() {
    m_doublesolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void sld_fwd() {
    m_doublesolenoid.set(Value.kForward);
  }

  public void sld_bwd() {
    m_doublesolenoid.set(Value.kReverse);
  }

  public void sld_off() {
    m_doublesolenoid.set(Value.kOff);
  }

  public void sld_toggle() {
    m_doublesolenoid.toggle();
  }
}
