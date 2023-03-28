package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pistons {

    // private Solenoid gripper_fwd = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    // private Solenoid gripper_bwd = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    private final DoubleSolenoid ankleDSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private final DoubleSolenoid gripDSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

    private PS4Controller driverController;

    public Pistons(PS4Controller driverController){
        this.driverController = driverController;
    }

    public void pistonInit(){
        SmartDashboard.putBoolean("GRAB", false);
        SmartDashboard.putBoolean("LEAVE", false);
    }

    public void tempFwdAnkle(){
        System.out.println("TEMP FWD Ankle");
        ankleDSolenoid.set(Value.kForward);
    }

    public void tempBwdAnkle(){
        System.out.println("TEMP BWD Ankle");
        ankleDSolenoid.set(Value.kReverse);
    }

    public void toggleAnkle(){
        System.out.println("Toggling Ankle");
        ankleDSolenoid.toggle();
    }

    public void tempFwdGripper(){
        System.out.println("FWD Gripper");
        gripDSolenoid.set(Value.kForward);
    }

    public void tempBwdGripper(){
        System.out.println("BWD Gripper");
        gripDSolenoid.set(Value.kReverse);
    }

    public void toggleGripper(){
        System.out.println("Toggling Gripper");
        gripDSolenoid.toggle();
    }

    public void pistonPeriodic(){

        if(driverController.getRawButtonPressed(6)){tempFwdGripper();}//Kare
        else if(driverController.getRawButtonPressed(8)){tempBwdGripper();}//X
        else if(driverController.getRawButtonPressed(5)){tempFwdAnkle();}//O
        else if(driverController.getRawButtonPressed(7)){tempBwdAnkle();}//Ucgen
    }


}
