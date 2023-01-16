package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.ChassisConstants;


public class Robot extends TimedRobot {

  private CANSparkMax leftMaster = new CANSparkMax(Constants.ChassisConstants.idLeftMaster, MotorType.kBrushless);
  private CANSparkMax leftSlave = new CANSparkMax(Constants.ChassisConstants.idLeftSlave, MotorType.kBrushless);
  private CANSparkMax rightMaster = new CANSparkMax(Constants.ChassisConstants.idRigthMaster, MotorType.kBrushless);
  private CANSparkMax rightSlave = new CANSparkMax(Constants.ChassisConstants.idRightSlave, MotorType.kBrushless);
  private TalonSRX midMaster = new TalonSRX(Constants.ChassisConstants.idMidMaster);
  private TalonSRX midSlave = new TalonSRX(Constants.ChassisConstants.idMidSlave);

  private DifferentialDrive drive;

  private SparkMaxPIDController leftpid;
  private SparkMaxPIDController rightpid;


  private PS4Controller driverController = new PS4Controller(0);

  private int flipVar = -1;


  @Override
  public void robotInit() {
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    midMaster.setInverted(false);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    midSlave.follow(midMaster);

    leftSlave.close();
    rightSlave.close();

    leftpid = leftMaster.getPIDController();
    rightpid = rightMaster.getPIDController();

    leftpid.setSmartMotionMaxVelocity(Constants.ChassisConstants.maxVel, 0);
    leftpid.setSmartMotionMaxAccel(Constants.ChassisConstants.maxAcc, 0);

    rightpid.setSmartMotionMaxVelocity(Constants.ChassisConstants.maxVel, 0);
    rightpid.setSmartMotionMaxAccel(Constants.ChassisConstants.maxAcc, 0);

    drive = new DifferentialDrive(leftMaster, rightMaster);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}
  @Override
  public void teleopInit() {}

  public void flipDirection() {
    if(flipVar == 1 && driverController.getRawButtonReleased(14)){flipVar = -1;}
    else if(flipVar == -1 && driverController.getRawButtonReleased(14)){flipVar = 1;}
  }

  public void driveFunctions() {
    flipDirection();

    double outputConstant = Constants.ChassisConstants.lowerOutput;

    if(driverController.getRawButton(6)){outputConstant = ChassisConstants.higherOutput;}
    drive.arcadeDrive(-driverController.getLeftX() * .8 * outputConstant, flipVar * driverController.getLeftY() * outputConstant);

    midMaster.set(TalonSRXControlMode.PercentOutput, driverController.getRightX() * .5);

  }

  @Override
  public void teleopPeriodic() {
    leftpid.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    rightpid.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    driveFunctions();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
