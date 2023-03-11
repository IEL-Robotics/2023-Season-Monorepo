package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;

public class Robot extends TimedRobot {

  private PS4Controller driverController = new PS4Controller(0);

  public Chassis m_chassis = new Chassis(driverController);
  public Arm m_arm = new Arm(driverController);

  @Override
  public void robotInit() {
    m_chassis.chassisInit();
    m_arm.armInit();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_chassis.setGyroStartingAngle();
  }

  @Override
  public void teleopPeriodic() {
    m_chassis.chassisDriving();
    m_arm.armPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
