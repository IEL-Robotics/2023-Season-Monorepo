package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PistonSubsystem {

    private final DoubleSolenoid ankleDSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    private final DoubleSolenoid gripDSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

    public PistonSubsystem(){
    }

    public void pistonInit(){
        SmartDashboard.putBoolean("GRAB", false);
        SmartDashboard.putBoolean("LEAVE", false);
    }

    public void tempFwdAnkle(){
        System.out.println("TEMP FWD Ankle");
        SmartDashboard.putBoolean("ANKLE ACIK", true);
        ankleDSolenoid.set(Value.kForward);
    }

    public void tempBwdAnkle(){
        System.out.println("TEMP BWD Ankle");
        SmartDashboard.putBoolean("ANKLE ACIK", false);
        ankleDSolenoid.set(Value.kReverse);
    }

    public void toggleAnkle(){
        System.out.println("Toggling Ankle");
        ankleDSolenoid.toggle();
    }

    public void tempFwdGripper(){//KAPA
        System.out.println("FWD Gripper");
        gripDSolenoid.set(Value.kForward);
        SmartDashboard.putBoolean("GRIPPER ACIK", true);
    }

    public void tempBwdGripper(){//AC
        System.out.println("BWD Gripper");
        gripDSolenoid.set(Value.kReverse);
        SmartDashboard.putBoolean("GRIPPER ACIK", false);
    }

    public void toggleGripper(){
        System.out.println("Toggling Gripper");
        gripDSolenoid.toggle();
    }

    public void pistonAnkleSet(int i){//!!!!!!
        if(i==0){tempFwdAnkle();}
        else if(i==1){tempBwdAnkle();}
    }

    //AUTONOMOUS
    public void pistonAuto(){
        tempBwdGripper();
        System.out.println("TEMP FWD AUTONOMOUS/ STEP 3");
    }

    public void pistonAutoAnkle(){
        tempFwdAnkle();
        System.out.println("ANKLE OPEN / STEP 1");
    }


}
