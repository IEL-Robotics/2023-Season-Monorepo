package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

    private PIDController pidController = new PIDController(0.01, 0.000, 0.02);
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
        
        //armMotorRight.follow(armMotorLeft);
        // leftPid = armMotorLeft.getPIDController();
        // rightPid = armMotorRight.getPIDController();

        pidController.setTolerance(2);

        armLeftEncoder = armMotorLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        armRightEncoder = armMotorRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    private int currentPOVVal = -1;
    public void armPeriodic(){
        if(driverController.getRawButton(7)){runLeft();} //L2
        else if(driverController.getRawButton(8)){runRight();} //R2
        else if(driverController.getRawButton(5)){runLeftReverse();} //L1
        else if(driverController.getRawButton(6)){runRightReverse();}//R1
        else{holdBoth();}

        if(driverController.getRawButtonReleased(3)){setThatPosition(0);} //Start Position
        else if(driverController.getRawButtonReleased(4)){setThatPosition(-20);}
        else if(driverController.getRawButtonReleased(1)){setThatPosition(-50);}
        else if(driverController.getRawButtonReleased(2)){setThatPosition(-30);}
        else if(driverController.getPOV()==0 && currentPOVVal!=0){
            currentPOVVal = 0;
            setThatPosition(-10);}
        else if(driverController.getPOV()==90 && currentPOVVal!=90){
            currentPOVVal = 90;
            setThatPosition(-10);}
        else if(driverController.getPOV()==270 && currentPOVVal!=270){
            currentPOVVal = 270;
            setThatPosition(-10);
        }

        if(LoopIsOn){
            goThatPosition();
        }

        SmartDashboard.putNumber("Arm Left E Val", armLeftEncoder.getPosition());
        SmartDashboard.putNumber("Arm Right E Val", armRightEncoder.getPosition());
        SmartDashboard.putBoolean("LoopIsOn", LoopIsOn);

    }

    public void runRight(){
        armMotorRight.set(0.45);
        System.out.println("sağ ilerici");
    }

    public void runRightReverse(){
        armMotorRight.set(-0.45);
        System.out.println("sağ gerici");
    }

    public void runLeft(){
        armMotorLeft.set(0.35);
        System.out.println("sol ilerici");
    }

    public void runLeftReverse(){
        armMotorLeft.set(-0.35);
        System.out.println("sol gerici");
    }

    public void holdBoth(){
        armMotorLeft.set(0);
        armMotorRight.set(0);
    }

    public void setThatPosition(double posVal){
        setPoint = posVal;
        pidController.setSetpoint(setPoint);
        LoopIsOn = true;
    }

    public void goThatPosition(){ //setlerken ipi sal
        armMotorLeft.set(pidController.calculate(armLeftEncoder.getPosition()));
        // armMotorRight.set(0.1);
        if(Math.abs(setPoint - armLeftEncoder.getPosition())<3){
            LoopIsOn = false;
            armMotorLeft.set(0);
        }
    }
}