package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Pistons;

public class Robot extends TimedRobot {

  private PS4Controller driverController = new PS4Controller(0);

  public Chassis m_chassis = new Chassis(driverController);
  public Arm m_arm = new Arm(driverController);
  public Pistons m_pistons = new Pistons(driverController);

  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  //private final PneumaticsControlModule m_pcm = new PneumaticsControlModule(1);
  private final PneumaticsControlModule m_pcm = new PneumaticsControlModule();

  @Override
  public void robotInit() {
    m_chassis.chassisInit();
    m_arm.armInit();
    m_pistons.pistonInit();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_pcm.enableCompressorDigital();
    m_compressor.enableDigital();
    m_chassis.setGyroStartingAngle();
  }

  @Override
  public void teleopPeriodic() {
    m_chassis.chassisDriving();
    m_arm.armPeriodic();
    m_pistons.pistonPeriodic();
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
