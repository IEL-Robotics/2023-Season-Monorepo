package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PIDConstants;

public class Chassis {
    private CANSparkMax leftMaster = new CANSparkMax(ChassisConstants.idLeftMaster, MotorType.kBrushless);
    private CANSparkMax leftSlave = new CANSparkMax(ChassisConstants.idLeftSlave, MotorType.kBrushless);
    private CANSparkMax rightMaster = new CANSparkMax(ChassisConstants.idRigthMaster, MotorType.kBrushless);
    private CANSparkMax rightSlave = new CANSparkMax(ChassisConstants.idRightSlave, MotorType.kBrushless);
    private TalonSRX midMaster = new TalonSRX(ChassisConstants.idMidMaster);
    private TalonSRX midSlave = new TalonSRX(ChassisConstants.idMidSlave);

    private DifferentialDrive drive;

    private RelativeEncoder leftEncoder, rightEncoder;

    private SparkMaxPIDController leftpid, rightpid;

    private PS4Controller driverController;
    private Vision vision;

    private int flipVar = -1;

    private double[] coordinates = {0, 0, 0};

    public Chassis(PS4Controller driverController, Vision vision){
        this.driverController = driverController;
        this.vision = vision;
    }

    public void chassisInit(){

        leftMaster.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        leftSlave.setIdleMode(IdleMode.kCoast);
        rightSlave.setIdleMode(IdleMode.kCoast);

        leftMaster.setInverted(true);
        rightMaster.setInverted(true);
        midMaster.setInverted(false);
    
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        midSlave.follow(midMaster);
    
        pidInit();
    
        drive = new DifferentialDrive(leftMaster, rightMaster);
    }

    public void chassisDriving(){

        pidPeriodic();

        if(driverController.getRawButtonReleased(14)){flipVar = -flipVar;}

        drive.setMaxOutput(ChassisConstants.lowerOutput);
        if(driverController.getRawButton(6)){drive.setMaxOutput(ChassisConstants.higherOutput);}

        drive.arcadeDrive(-driverController.getLeftX() * .7, flipVar * driverController.getLeftY());

        midMaster.set(TalonSRXControlMode.PercentOutput, driverController.getRawAxis(2) * .5);

        coordinates = vision.getFieldPosition(); //do whatever you want

    }

    public void pidInit(){

        leftpid = leftMaster.getPIDController();
        rightpid = rightMaster.getPIDController();

        leftpid.setP(PIDConstants.kP);
        rightpid.setP(PIDConstants.kP);

        leftpid.setI(PIDConstants.kI);
        rightpid.setI(PIDConstants.kI);

        leftpid.setD(PIDConstants.kD);
        rightpid.setD(PIDConstants.kD);

        leftpid.setIZone(PIDConstants.kIz);
        rightpid.setIZone(PIDConstants.kIz);

        leftpid.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
        rightpid.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

        //Display
        SmartDashboard.putNumber("P Gain", PIDConstants.kP);
        SmartDashboard.putNumber("I Gain", PIDConstants.kI);
        SmartDashboard.putNumber("D Gain", PIDConstants.kD);
        SmartDashboard.putNumber("I Zone", PIDConstants.kIz);
        SmartDashboard.putNumber("Feed Forward", PIDConstants.kFF);
        SmartDashboard.putNumber("Max Output", PIDConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", PIDConstants.kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);

        //Encoder
        leftEncoder = leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rightEncoder = rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        leftpid.setFeedbackDevice(leftEncoder);
        rightpid.setFeedbackDevice(rightEncoder);

    }

    public void pidPeriodic(){

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        if(p != PIDConstants.kP){leftpid.setP(p); rightpid.setP(p); PIDConstants.kP = p;}
        if(i != PIDConstants.kI){leftpid.setI(i); rightpid.setI(i); PIDConstants.kI = i;}
        if(d != PIDConstants.kD){leftpid.setD(d); rightpid.setD(d); PIDConstants.kD = d;}
        if(iz != PIDConstants.kIz){leftpid.setIZone(iz); rightpid.setIZone(iz); PIDConstants.kIz = iz;}
        if(max != PIDConstants.kMaxOutput || min != PIDConstants.kMinOutput){
            leftpid.setOutputRange(min, max);
            rightpid.setOutputRange(min, max);
            PIDConstants.kMaxOutput = max;
            PIDConstants.kMinOutput = min;
        }

        leftpid.setReference(rotations, CANSparkMax.ControlType.kPosition);
        rightpid.setReference(rotations, CANSparkMax.ControlType.kPosition);

    }

}
