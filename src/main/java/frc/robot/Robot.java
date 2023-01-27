package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {

  private PS4Controller driverController = new PS4Controller(0);
  public Vision vision = new Vision();

  public Chassis m_chassis = new Chassis(driverController, vision);

  @Override
  public void robotInit() {
    vision.targetTags(); //by default, called after passing the vision object into Chassis() -> would still work?
    m_chassis.chassisInit();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    m_chassis.chassisDriving();
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
