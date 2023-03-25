package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Pistons;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {

  private PS4Controller driverController = new PS4Controller(0);
  private PS4Controller copilotController = new PS4Controller(1);

  public Chassis m_chassis = new Chassis(driverController);
  public Arm m_arm = new Arm(copilotController);
  public Pistons m_pistons = new Pistons(driverController);
  public Vision m_vision = new Vision();

  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final PneumaticsControlModule m_pcm = new PneumaticsControlModule();

  private SequentialCommandGroup m_autocommand;

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
    if(m_autocommand != null) {
      m_autocommand.cancel();
    }
    m_pcm.enableCompressorDigital();
    m_compressor.enableDigital();
    m_chassis.setGyroStartingAngle();
  }

  @Override
  public void teleopPeriodic() {
    m_chassis.chassisDriving();
    m_arm.armPeriodic();
    m_pistons.pistonPeriodic();
    m_vision.getFieldPosition();
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
