package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Pistons;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {

  private PS4Controller driverController = new PS4Controller(0);
  private PS4Controller copilotController = new PS4Controller(1);

  public Chassis m_chassis = new Chassis(driverController);
  public Arm m_arm = new Arm(driverController, copilotController);
  public Pistons m_pistons = new Pistons(driverController);
  public Vision m_vision = new Vision();

  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final PneumaticsControlModule m_pcm = new PneumaticsControlModule();

  private int autonomusSteps = 1;
  private double[] coordinates = {0, 0, 0};

  @Override
  public void robotInit() {
    m_chassis.chassisInit();
    m_arm.armInit();
    m_pistons.pistonInit();
    CameraServer.startAutomaticCapture();

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_arm.armInit();
    m_pistons.pistonInit();
    m_chassis.setGyroStartingAngle();
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Steps", autonomusSteps);
    SmartDashboard.putNumber("AUTONOMOUS GYRO", m_chassis.getAlpha());
    autonomousBasic();
  }

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
    //m_vision.getFieldPosition();
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

  //AUTONOMOUS

  public void autonomousBasic(){
    
  }

  public void autonomousComplex(){
    if(autonomusSteps==0){
      coordinates = m_vision.getFieldPosition();
      if(coordinates[0] != 0){autonomusSteps+=1;}
    }
    else if(autonomusSteps==1){
      m_chassis.setTravelVal(-5);
      autonomusSteps+=1;
    }
    else if(autonomusSteps==2){
      if(m_chassis.travelThisMuch()==true){autonomusSteps+=1;}
    }
    else if(autonomusSteps==3){
      if(true){m_chassis.setRotate90Degrees(1); autonomusSteps+=1;}
      else if(false){m_chassis.setRotate90Degrees(-1); autonomusSteps+=1;} //deadcode, for now    
    }
    else if(autonomusSteps==4){
      if(m_chassis.completeRotation()==false){autonomusSteps+=1;}
    }
    else if(autonomusSteps==5){
      m_chassis.setTravelVal(2);
      autonomusSteps+=1;
    }
    else if(autonomusSteps==6){
      if(m_chassis.travelThisMuch()==true){autonomusSteps+=1;}
    }
    else if(autonomusSteps==7){
      if(m_chassis.travelThisMuch()==true){autonomusSteps+=1;}
    }
    else if(autonomusSteps==8){
      if(true){m_chassis.setRotate90Degrees(-1); autonomusSteps+=1;}
      else if(false){m_chassis.setRotate90Degrees(1); autonomusSteps+=1;} //deadcode, for now    
    }
    else if(autonomusSteps==9){
      if(m_chassis.completeRotation()==false){autonomusSteps+=1;}
    }
    else if(autonomusSteps==10){
      m_chassis.setTravelVal(4);
      autonomusSteps+=1;
    }
    else if(autonomusSteps==11){
      if(m_chassis.travelThisMuch()==true){autonomusSteps+=1;}
    }
    else if(autonomusSteps>11){
      m_chassis.alignThePitch();
    }
  }

}
