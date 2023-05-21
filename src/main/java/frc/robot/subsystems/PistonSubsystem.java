package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;


public class PistonSubsystem extends SubsystemBase {
  
  private final DoubleSolenoid anklePiston, gripPistons;

  public PistonSubsystem() {
    this.anklePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.idAnkleF, PneumaticsConstants.idAnkleR);
    this.gripPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.idGripF, PneumaticsConstants.idGripR);
  }

  public void ankleOpen() {
    anklePiston.set(Value.kForward);
  }

  public void ankleClose() {
    anklePiston.set(Value.kReverse);
  }

  public void gripperGrab() {
    gripPistons.set(Value.kForward);
  }
  
  public void gripperLeave() {
    gripPistons.set(Value.kReverse);
  }

  public void ankleToggle() {
    anklePiston.toggle();
  }

  public void gripToggle() {
    gripPistons.toggle();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Ankle Condition", this.anklePiston.get() == Value.kForward ? "Open" : "Closed"); // Im not really sure
    SmartDashboard.putString("Gripper Condition", this.gripPistons.get() == Value.kForward ? "Closed" : "Open"); // whether this is the correct labeling :/
  }

  @Override
  public void simulationPeriodic() {}
}
