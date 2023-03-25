package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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
        
        //armMotorRight.follow(armMotorLeft);
        // leftPid = armMotorLeft.getPIDController();
        // rightPid = armMotorRight.getPIDController();

        pidController.setTolerance(2);

        armLeftEncoder = armMotorLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        armRightEncoder = armMotorRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    public void armPeriodic(){
        if(driverController.getRawButton(7)){runLeft();} //L2
        else if(driverController.getRawButton(8)){runRight();} //R2
        else if(driverController.getRawButton(5)){runLeftReverse();} //L1
        else if(driverController.getRawButton(6)){runRightReverse();}//R1
        else{holdBoth();}

        if(driverController.getRawButtonReleased(3)){setThatPosition(0);}
        else if(driverController.getRawButtonReleased(4)){setThatPosition(-20);}
        else if(driverController.getRawButtonReleased(1)){setThatPosition(-50);}


        if(LoopIsOn){
            goThatPosition();
        }

        SmartDashboard.putNumber("Arm Left E Val", armLeftEncoder.getPosition());
        SmartDashboard.putNumber("Arm Right E Val", armRightEncoder.getPosition());
        SmartDashboard.putBoolean("LoopIsOn", LoopIsOn);

    }

    public void runRight(){
        armMotorRight.set(0.75);
        System.out.println("sağ ilerici");
    }

    public void runRightReverse(){
        armMotorRight.set(-0.75);
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

    public void goThatPosition(){
        armMotorLeft.set(pidController.calculate(armLeftEncoder.getPosition()));
        if(Math.abs(setPoint - armLeftEncoder.getPosition())<3){
            LoopIsOn = false;
        }
    }
}