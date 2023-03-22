package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PneumaticsConstants;

public class Pistons {
    // private DoubleSolenoid gripper_piston_1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.gripper1Fwd, PneumaticsConstants.gripper1Bwd);
    // private DoubleSolenoid gripper_piston_2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.gripper2Fwd, PneumaticsConstants.gripper2Bwd);
    
    //private DoubleSolenoid ankle_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.ankleFwd, PneumaticsConstants.ankleBwd);

    private Solenoid gripper_fwd = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    private Solenoid gripper_bwd = new Solenoid(PneumaticsModuleType.CTREPCM, 7);


    private PS4Controller copilotController;

    public Pistons(PS4Controller copilotController){
        this.copilotController = copilotController;
    }

    public void pistonInit(){
        SmartDashboard.putBoolean("GRAB", false);
        SmartDashboard.putBoolean("LEAVE", false);
    }

    // public void grab(){
    //     SmartDashboard.putBoolean("GRAB", true);
    //     System.out.println("Grabbed");
    //     gripper_piston_1.set(Value.kForward);
    //     gripper_piston_2.set(Value.kForward);
    // }

    // public void leave(){
    //     SmartDashboard.putBoolean("LEAVE", true);
    //     System.out.println("Left");
    //     gripper_piston_1.set(Value.kReverse);
    //     gripper_piston_2.set(Value.kReverse);
    // }

    // // public void hold(){
    // //     gripper_piston_1.set(Value.kOff);
    // //     gripper_piston_2.set(Value.kOff);
    // // }

    // public void gripperToggle(){
    //     System.out.println("toggled");
    //     gripper_piston_1.set(Value.kForward);
    //     gripper_piston_2.set(Value.kForward);
    // }

    // public void ankleToggle(){
    //     //ankle_piston.toggle();
    // }

    // // public void lower(){
    // //     ankle_piston.set(Value.kReverse);
    // // }

    // // public void raise(){
    // //     ankle_piston.set(Value.kForward);
    // // }

    public void tempFwd(){
        System.out.println("TEMP FWD");
        gripper_fwd.set(true);
        gripper_bwd.set(false);
    }

    public void tempBwd(){
        System.out.println("TEMP BWD");
        gripper_fwd.set(false);
        gripper_bwd.set(true);
    }

    public void pistonPeriodic(){
        // if(copilotController.getRawButtonPressed(2)){grab();}
        // else if(copilotController.getRawButtonPressed(1)){leave();}

        if(copilotController.getRawButtonPressed(2)){tempFwd();}
        else if(copilotController.getRawButtonPressed(1)){tempBwd();}
    }


}
