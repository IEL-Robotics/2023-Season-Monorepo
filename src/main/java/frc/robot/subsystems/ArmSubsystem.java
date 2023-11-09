package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  public TalonSRX m_RightMotor,m_LeftMotor; // responsible for extending // responsible for indir/kaldır stuff

  public PIDController leftPID, rightPID; // We prolly won't use 'rightPID' anyway

  public ArmSubsystem() {

    this.m_RightMotor = new TalonSRX(ArmConstants.idRightTelescope);
    this.m_LeftMotor = new TalonSRX(ArmConstants.idLeftTelescope);

    this.m_LeftMotor.enableCurrentLimit(true);
    this.m_RightMotor.enableCurrentLimit(true);

    this.m_LeftMotor.follow(m_RightMotor);

    this.m_RightMotor.setInverted(false);
    this.m_LeftMotor.setInverted(true);

    this.m_LeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //this.m_LeftMotor.setSelectedSensorPosition(0);
    this.m_RightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //this.m_RightMotor.setSelectedSensorPosition(0);

    //this.leftPID = new PIDController(0.005, 0.001, 0.0);
    this.rightPID = new PIDController(0.005, 0.0, 0.0);
    //this.leftPID.setTolerance(30);
    this.rightPID.setTolerance(35);

  }

  public void armSet(double speed) {
    m_LeftMotor.set(TalonSRXControlMode.PercentOutput, speed);
    m_RightMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public double getLeftEncoderVal() {
    return m_LeftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderVal() {
    return m_LeftMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    //System.out.println("Sol Encoder: " + m_LeftMotor.getSelectedSensorPosition());
    //System.out.println("Sag Encoder " + m_RightMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("MagEncoder Left Value", m_LeftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("MagEncoder Right Value", m_RightMotor.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
  }
}
