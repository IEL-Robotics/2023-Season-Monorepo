package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_RightMotor; // responsible for extending
  public TalonSRX m_LeftMotor; // responsible for indir/kaldır stuff

  private RelativeEncoder rightEncoder; // Because it's a CanEncoder, we define it exclusively

  public PIDController leftPID, rightPID; // We prolly won't use 'rightPID' anyway

  public ArmSubsystem() {

    this.m_RightMotor = new CANSparkMax(ArmConstants.idRightTelescope, MotorType.kBrushless);
    this.m_LeftMotor = new TalonSRX(ArmConstants.idLeftTelescope);
    this.m_RightMotor.setInverted(false);
    this.m_LeftMotor.setInverted(false);

    this.m_LeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); // Is relative an option?
    this.m_LeftMotor.setSelectedSensorPosition(0);
    this.rightEncoder = this.m_RightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    this.leftPID = new PIDController(0.3, 0.05, 0.01);
    this.rightPID = new PIDController(0.15, 0.000, 0.015);
    this.leftPID.setTolerance(30);
    this.rightPID.setTolerance(5);

  }

  public void armExtend() {
    m_RightMotor.set(0.8);
  }

  public void armShrink() {
    m_RightMotor.set(0.8);
  }

  public void armSet(double speed) {
    m_LeftMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public double getLeftEncoderVal() {
    return m_LeftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderVal() {
    return rightEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Left Encoder Value", getLeftEncoderVal());
    SmartDashboard.putNumber("Arm Right Encoder Value", getRightEncoderVal());
  }

  @Override
  public void simulationPeriodic() {
  }
}
