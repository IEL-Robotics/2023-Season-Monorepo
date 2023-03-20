package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisConstants;

public class Arm {
    private CANSparkMax armMotorLeft = new CANSparkMax(ChassisConstants.idLeftTelescope,MotorType.kBrushless);
    private CANSparkMax armMotorRight = new CANSparkMax(ChassisConstants.idRightTelescope,MotorType.kBrushless);

    private RelativeEncoder armLeftEncoder;
    private RelativeEncoder armRightEncoder;

    private PIDController pidController = new PIDController(0.2, 0.3, 0.05);
    // private SparkMaxPIDController rightPid, leftPid;


    private PS4Controller driverController;

    private boolean LoopIsOn = false;
    private double setPoint = 0;

    public Arm(PS4Controller driverController){
        this.driverController = driverController;
    }

    public void armInit(){
        armMotorLeft.setInverted(false);
        armMotorRight.setInverted(false);
        
        armMotorRight.follow(armMotorLeft);
        // leftPid = armMotorLeft.getPIDController();
        // rightPid = armMotorRight.getPIDController();

        pidController.setTolerance(2);

        armLeftEncoder = armMotorLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        armRightEncoder = armMotorRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    public void armPeriodic(){
        if(driverController.getRawButton(7)){runBoth(driverController.getL2Axis());} //L2
        else if(driverController.getRawButton(8)){runBothReverse(driverController.getR2Axis());} //R2
        //else if(driverController.getRawButton(3)){runJustLeft();} //Circle
        //else if(driverController.getRawButton(1)){runJustRight();}//SqUARE
        else{holdBoth();}

        if(driverController.getRawButtonReleased(4)){setThatPosition(500);}

        if(LoopIsOn){
            startTheLoop();
        }

        SmartDashboard.putNumber("Arm Left E Val", armLeftEncoder.getPosition());
        SmartDashboard.putNumber("Arm Right E Val", armRightEncoder.getPosition());
        SmartDashboard.putBoolean("LoopIsOn", LoopIsOn);

    }

    public void runBoth(double speed){
        // armMotorLeft.set(speed);
        // armMotorRight.set(speed);
        armMotorLeft.set(speed);
    }

    public void runBothReverse(double speed){
        // armMotorLeft.set(-speed);
        // armMotorRight.set(-speed);
        armMotorLeft.set(-speed);
    }

    public void holdBoth(){
        armMotorLeft.set(0);
        armMotorRight.set(0);
    }

    public void runJustLeft(){
        System.out.println("Supposedly Left");
        armMotorLeft.set(0.2);
    }

    public void runJustRight(){
        System.out.println("Supposedly Right");
        armMotorRight.set(0.2);
    }

    public void setThatPosition(double posVal){
        setPoint = posVal;
        pidController.setSetpoint(setPoint);
        LoopIsOn = true;
    }

    public void startTheLoop(){
        armMotorRight.set(pidController.calculate(armRightEncoder.getPosition()));
        if(Math.abs(setPoint - armRightEncoder.getPosition())<3){
            LoopIsOn = false;
        }
    }

}