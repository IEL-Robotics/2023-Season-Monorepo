// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDCoefficients;

public class Chassis extends SubsystemBase {
  public final DifferentialDrive m_drive;

  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final SparkMaxPIDController m_left_pid, m_right_pid, m_mid_pid;
  private final RelativeEncoder m_left_encoder, m_right_encoder, m_mid_encoder;
  private final CANSparkMax m_left_lead = new CANSparkMax(Constants.RobotConstants.kLeftFrontMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_right_lead = new CANSparkMax(Constants.RobotConstants.kRightFrontMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_left_follow = new CANSparkMax(Constants.RobotConstants.kLeftBackMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_right_follow = new CANSparkMax(Constants.RobotConstants.kRightBackMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_mid_lead = new CANSparkMax(Constants.RobotConstants.kMidFirstMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_mid_slave = new CANSparkMax(Constants.RobotConstants.kMidSecondMotorPort,
      MotorType.kBrushless);
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new Chassis. */
  public Chassis() {

    /*
     * Initialize the motors
     */

    /*
     * MotoSetting the Motors
     */

    m_left_follow.follow(m_left_lead);
    m_right_follow.follow(m_right_lead);
    m_mid_slave.follow(m_mid_lead);

    m_right_lead.setIdleMode(IdleMode.kBrake);
    m_left_lead.setIdleMode(IdleMode.kBrake);
    m_right_follow.setIdleMode(IdleMode.kCoast);
    m_left_follow.setIdleMode(IdleMode.kCoast);
    m_mid_lead.setIdleMode(IdleMode.kBrake);
    m_mid_slave.setIdleMode(IdleMode.kBrake);

    /*
     * PID
     */

    this.m_right_pid = m_right_lead.getPIDController();
    this.m_left_pid = m_left_lead.getPIDController();
    this.m_mid_pid = m_mid_lead.getPIDController();

    this.m_right_pid.setP(PIDCoefficients.kP);
    this.m_left_pid.setP(PIDCoefficients.kP);
    this.m_mid_pid.setP(PIDCoefficients.kP);

    this.m_right_pid.setI(PIDCoefficients.kI);
    this.m_left_pid.setI(PIDCoefficients.kI);
    this.m_mid_pid.setI(PIDCoefficients.kI);

    this.m_right_pid.setD(PIDCoefficients.kD);
    this.m_left_pid.setD(PIDCoefficients.kD);
    this.m_mid_pid.setD(PIDCoefficients.kD);

    this.m_right_pid.setIZone(PIDCoefficients.kIz);
    this.m_left_pid.setIZone(PIDCoefficients.kIz);
    this.m_mid_pid.setIZone(PIDCoefficients.kIz);

    this.m_right_pid.setFF(PIDCoefficients.kFF);
    this.m_left_pid.setFF(PIDCoefficients.kFF);
    this.m_mid_pid.setFF(PIDCoefficients.kFF);

    this.m_right_pid.setOutputRange(PIDCoefficients.kMinOutput, PIDCoefficients.kMaxOutput);
    this.m_left_pid.setOutputRange(PIDCoefficients.kMinOutput, PIDCoefficients.kMaxOutput);
    this.m_mid_pid.setOutputRange(PIDCoefficients.kMinOutput, PIDCoefficients.kMaxOutput);

    /*
     * Put values to Smartdashboard
     */
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", PIDCoefficients.kP);
    SmartDashboard.putNumber("I Gain", PIDCoefficients.kI);
    SmartDashboard.putNumber("D Gain", PIDCoefficients.kD);
    SmartDashboard.putNumber("I Zone", PIDCoefficients.kIz);
    SmartDashboard.putNumber("Feed Forward", PIDCoefficients.kFF);
    SmartDashboard.putNumber("Max Output", PIDCoefficients.kMaxOutput);
    SmartDashboard.putNumber("Min Output", PIDCoefficients.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    /*
     * Encoder
     */

    this.m_left_encoder = m_left_lead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    this.m_right_encoder = m_right_lead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    this.m_mid_encoder = m_mid_lead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // Wheel is 6 inches, gearbox ratio is ~ 11:1 and 1 meter is ~40inches
    // I hate imperial
    this.m_right_encoder.setPositionConversionFactor(Math.PI * 6 / 11 / 39.37);
    this.m_left_encoder.setPositionConversionFactor(Math.PI * 6 / 11 / 39.37);
    this.m_right_encoder.setVelocityConversionFactor(Math.PI * 6 / 11 / 39.37);
    this.m_left_encoder.setVelocityConversionFactor(Math.PI * 6 / 11 / 39.37);
    // mid wheel gearbox ratio is something different
    this.m_mid_encoder.setPosition(Math.PI * 6 / 8.46 / 39.37);
    this.m_mid_encoder.setVelocityConversionFactor(Math.PI * 6 / 8.46 / 39.37);

    /*
     * Encoder and PID
     */

    this.m_right_pid.setFeedbackDevice(m_right_encoder);
    this.m_left_pid.setFeedbackDevice(m_left_encoder);
    this.m_mid_pid.setFeedbackDevice(m_mid_encoder);

    reset_encoders();

    /*
     * Differential Drive
     */
    this.m_drive = new DifferentialDrive(m_left_lead, m_right_lead);
    this.m_drive.setSafetyEnabled(false);

    this.m_gyro.calibrate();

    this.m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_left_encoder.getPosition(),
        m_right_encoder.getPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(m_left_encoder.getVelocity(), m_right_encoder.getVelocity());

  }

  public void reset_odometry(Pose2d pose) {

    reset_encoders();

    m_odometry.resetPosition(m_gyro.getRotation2d(), m_left_encoder.getPosition(), m_right_encoder.getPosition(), pose);

  }

  public void drive_mid_motor(double axis_value, boolean pressed) {
    double output_multiplier = (pressed) ? 1.0 : 0.5;
    this.m_mid_lead.set(axis_value * output_multiplier);
  }

  public void reset_encoders() {
    this.m_right_encoder.setPosition(0);
    this.m_left_encoder.setPosition(0);
    this.m_mid_encoder.setPosition(0);
  }

  /**
   * 
   * Drives the robot using arcade controls.
   *
   * 
   * 
   * @param fwd the commanded forward movement
   * 
   * @param rot the commanded rotation
   * 
   */

  public void arcadeDrive(double fwd, double rot) {

    m_drive.arcadeDrive(fwd, rot);

  }

  /**
   * 
   * Controls the left and right sides of the drive directly with voltages.
   *
   * 
   * 
   * @param leftVolts  the commanded left output
   * 
   * @param rightVolts the commanded right output
   * 
   */

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    m_left_lead.setVoltage(leftVolts);

    m_right_lead.setVoltage(rightVolts);

    m_drive.feed();

  }

  public double getAverageEncoderDistance() {

    return (m_left_encoder.getPosition() + m_right_encoder.getPosition()) / 2.0;

  }

  public RelativeEncoder getLeftEncoder() {

    return m_left_encoder;

  }

  public RelativeEncoder getRightEncoder() {

    return m_right_encoder;

  }

  public void setMaxOutput(double maxOutput) {

    m_drive.setMaxOutput(maxOutput);

  }

  public void zeroHeading() {

    m_gyro.reset();

  }

  public double getHeading() {

    return m_gyro.getRotation2d().getDegrees();

  }

  public double getTurnRate() {

    return -m_gyro.getRate();

  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_left_encoder.getPosition(), m_right_encoder.getPosition());
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != PIDCoefficients.kP)) {
      this.m_right_pid.setP(p);
      this.m_left_pid.setP(p);
      PIDCoefficients.kP = p;
    }
    if ((i != PIDCoefficients.kI)) {
      this.m_right_pid.setI(i);
      this.m_left_pid.setI(i);
      PIDCoefficients.kI = i;
    }
    if ((d != PIDCoefficients.kD)) {
      this.m_right_pid.setD(d);
      this.m_left_pid.setD(d);
      PIDCoefficients.kD = d;
    }
    if ((iz != PIDCoefficients.kIz)) {
      this.m_right_pid.setIZone(iz);
      this.m_left_pid.setIZone(iz);
      PIDCoefficients.kIz = iz;
    }
    if ((ff != PIDCoefficients.kFF)) {
      this.m_right_pid.setFF(ff);
      this.m_left_pid.setFF(ff);
      PIDCoefficients.kFF = ff;
    }
    if ((max != PIDCoefficients.kMaxOutput) || (min != PIDCoefficients.kMinOutput)) {
      this.m_right_pid.setOutputRange(min, max);
      this.m_left_pid.setOutputRange(min, max);
      PIDCoefficients.kMinOutput = min;
      PIDCoefficients.kMaxOutput = max;
    }

    this.m_right_pid.setReference(rotations, CANSparkMax.ControlType.kPosition);
    this.m_left_pid.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }
}
