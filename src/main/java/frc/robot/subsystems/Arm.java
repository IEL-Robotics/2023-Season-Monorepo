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
    private CANSparkMax armExtend = new CANSparkMax(ChassisConstants.idLeftTelescope,MotorType.kBrushless);  // arm extend
    private CANSparkMax armRaise = new CANSparkMax(ChassisConstants.idRightTelescope,MotorType.kBrushless); // arm raise lower

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
        armExtend.setInverted(false);
        armRaise.setInverted(false);
        
        //armMotorRight.follow(armMotorLeft);
        // leftPid = armMotorLeft.getPIDController();
        // rightPid = armMotorRight.getPIDController();

        pidController.setTolerance(2);

        armLeftEncoder = armExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        armRightEncoder = armRaise.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    public void armPeriodic(){
        if(driverController.getRawButton(7)){runLeft();} //L2
        else if(driverController.getRawButton(8)){runRight();} //R2
        else if(driverController.getRawButton(5)){runLeftReverse();} //Circle
        else if(driverController.getRawButton(6)){runRightReverse();}//SqUARE
        else{holdBoth();}

        if(driverController.getRawButtonReleased(4)){setThatPosition(500);}

        if(LoopIsOn){
            startTheLoop();
        }

        SmartDashboard.putNumber("Arm Left E Val", armLeftEncoder.getPosition());
        SmartDashboard.putNumber("Arm Right E Val", armRightEncoder.getPosition());
        SmartDashboard.putBoolean("LoopIsOn", LoopIsOn);

    }

    public void runRight(){
        armRaise.set(1);
        armExtend.set(0.3); // TODO: change proportion
        System.out.println("sağ ilerici");
    }

    public void runRightReverse(){
        armRaise.set(-1);
        armExtend.set(-0.3); // TODO: change proportion
        System.out.println("sağ gerici");
    }

    public void runLeft(){
        armExtend.set(0.6);
        System.out.println("sol ilerici");
    }

    public void runLeftReverse(){
        armExtend.set(-0.6);
        System.out.println("sol gerici");
    }

    public void holdBoth(){
        armExtend.set(0);
        armRaise.set(0);
    }

    public void setThatPosition(double posVal){
        setPoint = posVal;
        pidController.setSetpoint(setPoint);
        LoopIsOn = true;
    }

    public void startTheLoop(){
        armRaise.set(pidController.calculate(armRightEncoder.getPosition()));
        if(Math.abs(setPoint - armRightEncoder.getPosition())<3){
            LoopIsOn = false;
        }
    }

}