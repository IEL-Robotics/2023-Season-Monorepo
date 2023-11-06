package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PIDConstants;

public class ChassisSubsystem extends SubsystemBase {
  
  private CANSparkMax m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;
  public RelativeEncoder leftEncoder, rightEncoder;

  public SparkMaxPIDController leftPid, rightPid;

  public PIDController leftPidController = new PIDController(0.2, 0.3, 0.05); 
  public PIDController rightPidController = new PIDController(0.2, 0.3, 0.05);
  public PIDController midPidController = new PIDController(0.2, 0.3, 0.05);

  private AHRS gyro; // as if we have a functional H-Drive :')

  public DifferentialDrive m_drive;

  public ChassisSubsystem() {

    this.m_leftMaster = new CANSparkMax(ChassisConstants.idLeftMaster, MotorType.kBrushless);
    this.m_rightMaster = new CANSparkMax(ChassisConstants.idRightMaster, MotorType.kBrushless);
    this.m_leftSlave = new CANSparkMax(ChassisConstants.idLeftSlave, MotorType.kBrushless);
    this.m_rightSlave = new CANSparkMax(ChassisConstants.idRightSlave, MotorType.kBrushless);

    this.m_leftMaster.setIdleMode(IdleMode.kBrake);
    this.m_rightMaster.setIdleMode(IdleMode.kBrake);
    this.m_leftSlave.setIdleMode(IdleMode.kCoast);
    this.m_rightSlave.setIdleMode(IdleMode.kCoast);

    this.m_leftSlave.follow(m_leftMaster);
    this.m_rightSlave.follow(m_rightMaster);

    this.m_leftMaster.setInverted(false);
    this.m_rightMaster.setInverted(true); // dunno why, be careful tho

    this.leftEncoder = this.m_leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    this.rightEncoder = this.m_rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    this.leftEncoder.setPositionConversionFactor(Math.PI * 6 / 11 / 39.37);
    this.rightEncoder.setPositionConversionFactor(Math.PI * 6 / 11 / 39.37);

    pidInit();

    this.m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
  }

  public void pidInit() {
    
    leftPid = m_leftMaster.getPIDController();
    rightPid = m_rightMaster.getPIDController();

    leftPid.setP(PIDConstants.kP);
    rightPid.setP(PIDConstants.kP);

    leftPid.setI(PIDConstants.kI);
    rightPid.setI(PIDConstants.kI);

    leftPid.setD(PIDConstants.kD);
    rightPid.setD(PIDConstants.kD);

    leftPid.setIZone(PIDConstants.kIz);
    rightPid.setIZone(PIDConstants.kIz);

    leftPid.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    rightPid.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

    // Display
    SmartDashboard.putNumber("P Gain", PIDConstants.kP);
    SmartDashboard.putNumber("I Gain", PIDConstants.kI);
    SmartDashboard.putNumber("D Gain", PIDConstants.kD);
    SmartDashboard.putNumber("I Zone", PIDConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", PIDConstants.kFF);
    SmartDashboard.putNumber("Max Output", PIDConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", PIDConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    leftPid.setFeedbackDevice(leftEncoder);
    rightPid.setFeedbackDevice(rightEncoder);

  }

 
  @Override
  public void periodic() {

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    if (p != PIDConstants.kP) {
        leftPid.setP(p);
        rightPid.setP(p);
        PIDConstants.kP = p;
    }
    if (i != PIDConstants.kI) {
        leftPid.setI(i);
        rightPid.setI(i);
        PIDConstants.kI = i;
    }
    if (d != PIDConstants.kD) {
        leftPid.setD(d);
        rightPid.setD(d);
        PIDConstants.kD = d;
    }
    if (iz != PIDConstants.kIz) {
        leftPid.setIZone(iz);
        rightPid.setIZone(iz);
        PIDConstants.kIz = iz;
    }
    if (max != PIDConstants.kMaxOutput || min != PIDConstants.kMinOutput) {
        leftPid.setOutputRange(min, max);
        rightPid.setOutputRange(min, max);
        PIDConstants.kMaxOutput = max;
        PIDConstants.kMinOutput = min;
    }

    leftPid.setReference(rotations, CANSparkMax.ControlType.kPosition);
    rightPid.setReference(rotations, CANSparkMax.ControlType.kPosition);

    this.m_drive.feed();

    // Display
    SmartDashboard.putNumber("P Gain", PIDConstants.kP);
    SmartDashboard.putNumber("I Gain", PIDConstants.kI);
    SmartDashboard.putNumber("D Gain", PIDConstants.kD);
    SmartDashboard.putNumber("I Zone", PIDConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", PIDConstants.kFF);
    SmartDashboard.putNumber("Max Output", PIDConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", PIDConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

  }

  @Override
  public void simulationPeriodic() {}

}
