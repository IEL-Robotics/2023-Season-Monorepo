package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PIDConstants;

public class Chassis {
    private CANSparkMax leftMaster = new CANSparkMax(ChassisConstants.idLeftMaster, MotorType.kBrushless);
    private CANSparkMax leftSlave = new CANSparkMax(ChassisConstants.idLeftSlave, MotorType.kBrushless);
    private CANSparkMax rightMaster = new CANSparkMax(ChassisConstants.idRigthMaster, MotorType.kBrushless);
    private CANSparkMax rightSlave = new CANSparkMax(ChassisConstants.idRightSlave, MotorType.kBrushless);
    private CANSparkMax midMaster = new CANSparkMax(ChassisConstants.idMidMaster, MotorType.kBrushless);
    private CANSparkMax midSlave = new CANSparkMax(ChassisConstants.idMidSlave, MotorType.kBrushless);
   
    private double CurrentGP = 180;
    private double SupposedGP = 180;
    private double prev_error = 0;

    private DifferentialDrive drive;

    private RelativeEncoder leftEncoder, rightEncoder, midEncoder;

    private SparkMaxPIDController leftpid, rightpid;

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private PS4Controller driverController;

    private float startAlpha;

    private double sideWheelsAngleVariable;
    private double midWheelsAngleVariable;

    public Chassis(PS4Controller driverController){
        this.driverController = driverController;
    }

    public void chassisInit(){

        leftMaster.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        leftSlave.setIdleMode(IdleMode.kCoast);
        rightSlave.setIdleMode(IdleMode.kCoast);

        leftMaster.setInverted(false);
        rightMaster.setInverted(true); //idk why but it works this way, my bad
        midMaster.setInverted(false);
    
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        midSlave.follow(midMaster);

        pidInit();
    
        drive = new DifferentialDrive(leftMaster, rightMaster);
    }

    public void chassisDriving(){
        
        pidPeriodic();

        drive.setMaxOutput(ChassisConstants.lowerOutput);
        if(driverController.getRawButton(6)){drive.setMaxOutput(ChassisConstants.higherOutput);}
        

        SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
        SmartDashboard.putNumber("Pitch", gyro.getPitch()); //Charge Station
        SmartDashboard.putNumber("Start Alpha Pos", startAlpha);
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Relative Pos", gyro.getYaw()-startAlpha);
        SmartDashboard.putNumber("Roll", gyro.getRoll());

        findGyroVariables();
        if(driverController.getTouchpadPressed()){
            startAlpha = gyro.getYaw();
        }

        SmartDashboard.putNumber("SideWAC", sideWheelsAngleVariable);
        SmartDashboard.putNumber("MidWAC", midWheelsAngleVariable);

        SmartDashboard.putNumber("Axis2", driverController.getRawAxis(2));

        // leftMaster.set(sideWheelsAngleConstant + driverController.getRawAxis(2));
        // rightMaster.set(sideWheelsAngleConstant - driverController.getRawAxis(2));        

        //*****/
        // if(Math.abs(sideWheelsAngleVariable) > 0.05){
        //     drive.arcadeDrive(sideWheelsAngleVariable, driverController.getRawAxis(2));
        //     SmartDashboard.putBoolean("Buyuktur?", true);
        //     // leftMaster.set(sideWheelsAngleVariable * .5 + driverController.getRawAxis(2) * .5);
        //     // rightMaster.set(sideWheelsAngleVariable * .5 - driverController.getRawAxis(2) * .5);
        // }
        // else {
        //     if(Math.abs(driverController.getRawAxis(2))>0.05){
        //         leftMaster.set(driverController.getRawAxis(2) * .5);
        //         rightMaster.set(-driverController.getRawAxis(2) * .5);
        //     }
        //     SmartDashboard.putBoolean("Buyuktur?", false);
        // }
        /*****/

        // if(Math.abs(sideWheelsAngleVariable) > 0.05){
        //     drive.tankDrive(sideWheelsAngleVariable + driverController.getRawAxis(2), sideWheelsAngleVariable - driverController.getRawAxis(2));
        //     SmartDashboard.putBoolean("Buyuktur?", true);
        // }
        // else {
        //     drive.tankDrive(0, 0);
        //     SmartDashboard.putBoolean("Buyuktur?", false);
        // }
        double CR = getCorrectionRate() / 90;

        drive.tankDrive(sideWheelsAngleVariable + driverController.getRawAxis(2) + CR, sideWheelsAngleVariable - driverController.getRawAxis(2) - CR);
        //drive.tankDrive(sideWheelsAngleVariable + driverController.getRawAxis(2), sideWheelsAngleVariable - driverController.getRawAxis(2));


        if(Math.abs(midWheelsAngleVariable) > 0.05) {midMaster.set(midWheelsAngleVariable * .5); }
        else{midMaster.set(0);}

        SmartDashboard.putNumber("SGP", SupposedGP);
        SmartDashboard.putNumber("CGP", CurrentGP);

        SmartDashboard.putNumber("Ratium Correctio", CR);
        SmartDashboard.putNumber("Encoder Val Left", leftEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Val Right", rightEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Val Mid", midEncoder.getPosition());
    }

    public double getCorrectionRate(){
        double errorOption1, errorOption2, error,integral = 0, ableitung=0, ausgang;
        CurrentGP = getAlpha() + 180;

        if(Math.abs(driverController.getRawAxis(2)) > 0.05){
            SupposedGP = CurrentGP;
            return 0;
        }
        else{
            errorOption1 = SupposedGP - CurrentGP;
            errorOption2 = 360-(errorOption1);
            if(Math.abs(errorOption1)<Math.abs(errorOption2)){error = errorOption1;}
            else{error = errorOption2;}

            integral += error;
            ableitung -= error;
            ausgang = (0.5* error) + (0.5 * integral) + (0.5 * ableitung);
            prev_error = error;
            return ausgang;
        }
    }

    public double getBeta(){
        double betaAngle = Math.toDegrees(Math.atan2((driverController.getRawAxis(1)),(driverController.getRawAxis(0)))) + 90;
        SmartDashboard.putNumber("Beta as Degree", betaAngle);
        return betaAngle;
    }

    public double getAlpha(){
        double alphaAngle = gyro.getYaw() - startAlpha;
        return alphaAngle;
    }

    public void findGyroVariables(){
        double b = getBeta();
        double a = getAlpha();

        double kSide = Math.cos(Math.toRadians(b - a));
        double kMid = Math.sin(Math.toRadians(b - a));

        double v0 = Math.pow(driverController.getRawAxis(0), 2);
        double v1 = Math.pow(driverController.getRawAxis(1), 2);
        double v2 = Math.sqrt(v0 + v1);

        SmartDashboard.putNumber("B", b);
        SmartDashboard.putNumber("A", a); 
        
        SmartDashboard.putNumber("Cos B-A", kSide);
        SmartDashboard.putNumber("Sin B-A ", kMid);

        sideWheelsAngleVariable = v2 * kSide;
        midWheelsAngleVariable = v2 * kMid;
    }

    public void pidInit(){

        leftpid = leftMaster.getPIDController();
        rightpid = rightMaster.getPIDController();

        leftpid.setP(PIDConstants.kP);
        rightpid.setP(PIDConstants.kP);

        leftpid.setI(PIDConstants.kI);
        rightpid.setI(PIDConstants.kI);

        leftpid.setD(PIDConstants.kD);
        rightpid.setD(PIDConstants.kD);

        leftpid.setIZone(PIDConstants.kIz);
        rightpid.setIZone(PIDConstants.kIz);

        leftpid.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
        rightpid.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

        //Display
        SmartDashboard.putNumber("P Gain", PIDConstants.kP);
        SmartDashboard.putNumber("I Gain", PIDConstants.kI);
        SmartDashboard.putNumber("D Gain", PIDConstants.kD);
        SmartDashboard.putNumber("I Zone", PIDConstants.kIz);
        SmartDashboard.putNumber("Feed Forward", PIDConstants.kFF);
        SmartDashboard.putNumber("Max Output", PIDConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", PIDConstants.kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);

        //Encoder
        leftEncoder = leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rightEncoder = rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        midEncoder = midMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        leftpid.setFeedbackDevice(leftEncoder);
        rightpid.setFeedbackDevice(rightEncoder);

    }

    public void pidPeriodic(){

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        if(p != PIDConstants.kP){leftpid.setP(p); rightpid.setP(p); PIDConstants.kP = p;}
        if(i != PIDConstants.kI){leftpid.setI(i); rightpid.setI(i); PIDConstants.kI = i;}
        if(d != PIDConstants.kD){leftpid.setD(d); rightpid.setD(d); PIDConstants.kD = d;}
        if(iz != PIDConstants.kIz){leftpid.setIZone(iz); rightpid.setIZone(iz); PIDConstants.kIz = iz;}
        if(max != PIDConstants.kMaxOutput || min != PIDConstants.kMinOutput){
            leftpid.setOutputRange(min, max);
            rightpid.setOutputRange(min, max);
            PIDConstants.kMaxOutput = max;
            PIDConstants.kMinOutput = min;
        }
                
        leftpid.setReference(rotations, CANSparkMax.ControlType.kPosition);
        rightpid.setReference(rotations, CANSparkMax.ControlType.kPosition);

    }

    public void setGyroStartingAngle(){
        startAlpha = gyro.getYaw();

        // CurrentGP = getAlpha() + 180;
        // SupposedGP = CurrentGP;

        CurrentGP = 180;
        SupposedGP = 180;
    }

}
