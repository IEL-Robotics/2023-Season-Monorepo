package frc.robot.subsystems;


public class Autonomus {
    public Chassis a_chassis;
    public Vision a_vision;
    public Arm a_arm;
    public Pistons a_pistons;

    public Autonomus(Chassis a_chassis, Vision a_vision, Arm a_arm, Pistons a_pistons){
        this.a_chassis = a_chassis;
        this.a_arm = a_arm;
        this.a_pistons = a_pistons;
        this.a_vision = a_vision;
    }
}
