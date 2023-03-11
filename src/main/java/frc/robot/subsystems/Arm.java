package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PS4Controller;

public class Arm {
    private TalonSRX armMotorLeft = new TalonSRX(1);
    private TalonSRX armMotorRight = new TalonSRX(2);

    private PS4Controller driverController;

    public Arm(PS4Controller driverController){
        this.driverController = driverController;
    }

    public void armInit(){
        armMotorLeft.setInverted(false);
        armMotorRight.setInverted(false);
    }

    public void armPeriodic(){
        if(driverController.getRawButton(7)){runBoth();} //L2
        else if(driverController.getRawButton(8)){runBothReverse();} //R2
        else{holdBoth();}

        // if(driverController.getRawButton(7)){runLeft();}
        // else{holdLeft();}

        // if(driverController.getRawButton(8)){runRight();}
        // else{holdLeft();}

    }

    public void runBoth(){
        armMotorLeft.set(TalonSRXControlMode.PercentOutput, 0.7);
        armMotorRight.set(TalonSRXControlMode.PercentOutput, 0.7);
    }

    public void runBothReverse(){
        armMotorLeft.set(TalonSRXControlMode.PercentOutput, -0.7);
        armMotorRight.set(TalonSRXControlMode.PercentOutput, -0.7);
    }

    public void holdBoth(){
        armMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
        armMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
    }

    // public void runLeft(){
    //     armMotorLeft.set(TalonSRXControlMode.PercentOutput, 0.7);
    // }

    // public void runRight(){
    //     armMotorRight.set(TalonSRXControlMode.PercentOutput, 0.7);
    // }

    // public void holdLeft(){
    //     armMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
    // }

    // public void holdRight(){
    //     armMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
    // }

}