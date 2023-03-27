package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

    private boolean tunaMode = false;

    private double CurrentGP = 180;
    private double SupposedGP = 180;

    private DifferentialDrive drive;

    private RelativeEncoder leftEncoder, rightEncoder, midEncoder;

    private SparkMaxPIDController leftpid, rightpid;

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private PS4Controller driverController;

    private float startAlpha;

    private double sideWheelsAngleVariable;
    private double midWheelsAngleVariable;

    public Chassis(PS4Controller driverController) {
        this.driverController = driverController;
    }

    public void chassisInit() {

        leftMaster.setIdleMode(IdleMode.kBrake);
        rightMaster.setIdleMode(IdleMode.kBrake);
        leftSlave.setIdleMode(IdleMode.kCoast);
        rightSlave.setIdleMode(IdleMode.kCoast);
        midMaster.setIdleMode(IdleMode.kBrake);
        midSlave.setIdleMode(IdleMode.kCoast);

        leftMaster.setInverted(false);
        rightMaster.setInverted(true); // idk why but it works this way, my bad
        midMaster.setInverted(false);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        midSlave.follow(midMaster);

        leftEncoder = leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rightEncoder = rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        midEncoder = midMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        // bu senin bütün şeyi bozabilir
        leftEncoder.setPositionConversionFactor(Math.PI * 6 / 11 / 39.37);
        rightEncoder.setPositionConversionFactor(Math.PI * 6 / 11 / 39.37);
        midEncoder.setPositionConversionFactor(Math.PI * 6 / 8.46 / 39.37);

        pidInit();

        drive = new DifferentialDrive(leftMaster, rightMaster);
    }

    public void chassisDriving() {

        pidPeriodic();

        drive.setMaxOutput(ChassisConstants.lowerOutput);
        if (driverController.getRawButton(6)) {drive.setMaxOutput(ChassisConstants.higherOutput);}

        SmartDashboard.putNumber("Pitch", gyro.getPitch()); // Charge Station

        if (driverController.getRawButtonReleased(14)) {toggleCartesian();}

        double CR=0;
        CR = getFinalCR();

        if (tunaMode == false) {
            findGyroConstants();
            drive.tankDrive(sideWheelsAngleVariable + driverController.getRawAxis(2) + CR,
                    sideWheelsAngleVariable - driverController.getRawAxis(2) - CR);

            if (Math.abs(midWheelsAngleVariable) > 0.05) {
                midMaster.set(midWheelsAngleVariable * .5);
            } else {
                midMaster.set(0);
            }
        } else {
            // + - CR ekle
            drive.tankDrive(-driverController.getRawAxis(1) + driverController.getRawAxis(2),
                    -driverController.getRawAxis(1) - driverController.getRawAxis(2));
            midMaster.set(driverController.getRawAxis(0));
        }

        SmartDashboard.putNumber("SGP", SupposedGP);
        SmartDashboard.putNumber("CGP", CurrentGP);

        SmartDashboard.putNumber("Ratium Correctio", CR);
        SmartDashboard.putNumber("Encoder Val Left", leftEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Val Right", rightEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Val Mid", midEncoder.getPosition());

        SmartDashboard.putBoolean("OC TUNA", tunaMode);
    }

    public double getCorrectionRate() {
        double firstOption, secondOption;
        CurrentGP = getAlpha() + 180;

        if (Math.abs(driverController.getRawAxis(2)) > 0.1) {
            SupposedGP = CurrentGP;
            return 0;
        }
        
        firstOption = SupposedGP - CurrentGP;
        secondOption = 360-Math.abs(firstOption);
        if(Math.abs(firstOption)<Math.abs(secondOption)){return firstOption;}
        else{return secondOption;}
    }

    public double getFinalCR(){
        double CR=0;
        if(getCorrectionRate() >= 10){
             CR = getCorrectionRate() / 80;
        }
        else {
            if(getCorrectionRate() > 3) CR = 0.28;
            if(getCorrectionRate() < -3) CR = -0.28;
        }

        if (Math.abs(driverController.getRawAxis(0)) > 0.2 || Math.abs(driverController.getRawAxis(1)) > 0.2 ) {
            CR = getCorrectionRate() / 80;
        }
        return CR;

    }

    public double getBeta() {
        double betaAngle = Math
                .toDegrees(Math.atan2((driverController.getRawAxis(1)), (driverController.getRawAxis(0)))) + 90;
        SmartDashboard.putNumber("Beta as Degree", betaAngle);
        return betaAngle;
    }

    public double getAlpha() {
        double alphaAngle = gyro.getYaw() - startAlpha;
        return alphaAngle;
    }

    public void findGyroConstants() {
        double b = getBeta();
        double a = getAlpha();

        double kSide = Math.cos(Math.toRadians(b - a));
        double kMid = Math.sin(Math.toRadians(b - a));

        double v0 = Math.pow(driverController.getRawAxis(0), 2);
        double v1 = Math.pow(driverController.getRawAxis(1), 2);
        double v2 = Math.sqrt(v0 + v1);

        sideWheelsAngleVariable = v2 * kSide;
        midWheelsAngleVariable = v2 * kMid;
    }

    public void pidInit() {

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

        // Display
        SmartDashboard.putNumber("P Gain", PIDConstants.kP);
        SmartDashboard.putNumber("I Gain", PIDConstants.kI);
        SmartDashboard.putNumber("D Gain", PIDConstants.kD);
        SmartDashboard.putNumber("I Zone", PIDConstants.kIz);
        SmartDashboard.putNumber("Feed Forward", PIDConstants.kFF);
        SmartDashboard.putNumber("Max Output", PIDConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", PIDConstants.kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);

        leftpid.setFeedbackDevice(leftEncoder);
        rightpid.setFeedbackDevice(rightEncoder);

    }

    public void pidPeriodic() {

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        if (p != PIDConstants.kP) {
            leftpid.setP(p);
            rightpid.setP(p);
            PIDConstants.kP = p;
        }
        if (i != PIDConstants.kI) {
            leftpid.setI(i);
            rightpid.setI(i);
            PIDConstants.kI = i;
        }
        if (d != PIDConstants.kD) {
            leftpid.setD(d);
            rightpid.setD(d);
            PIDConstants.kD = d;
        }
        if (iz != PIDConstants.kIz) {
            leftpid.setIZone(iz);
            rightpid.setIZone(iz);
            PIDConstants.kIz = iz;
        }
        if (max != PIDConstants.kMaxOutput || min != PIDConstants.kMinOutput) {
            leftpid.setOutputRange(min, max);
            rightpid.setOutputRange(min, max);
            PIDConstants.kMaxOutput = max;
            PIDConstants.kMinOutput = min;
        }

        leftpid.setReference(rotations, CANSparkMax.ControlType.kPosition);
        rightpid.setReference(rotations, CANSparkMax.ControlType.kPosition);

    }

    public void setGyroStartingAngle() {
        startAlpha = gyro.getYaw();
        // CurrentGP = getAlpha() + 180;
        // SupposedGP = CurrentGP;
        CurrentGP = 180;
        SupposedGP = 180;
    }

    public void toggleCartesian() {
        tunaMode = !tunaMode;
    }

    // AUTONOMUS
    public void setRotate90Degrees(int factor) {
        double Wert = CurrentGP - (factor * 90);
        if (Wert >= 360) {
            SupposedGP = Wert - 360;
        } else if (Wert < 0) {
            SupposedGP = Wert + 360;
        } else {
            SupposedGP = Wert;
        }
    }

    public boolean completeRotation() {
        double CR = getFinalCR();
        drive.tankDrive(CR, -CR);
        if (Math.abs(SupposedGP - CurrentGP) <= 5) {
            return false;
        } else {
            return true;
        }
    }

    private PIDController leftPidController = new PIDController(0.2, 0.3, 0.05);
    private PIDController rightPidController = new PIDController(0.2, 0.3, 0.05);
    private PIDController midPidController = new PIDController(0.2, 0.3, 0.05);

    private double leftSetPoint, rightSetPoint, midSetPoint;

    public void setTravelVal(double distance) {
        leftSetPoint = leftEncoder.getPosition() + distance;
        rightSetPoint = rightEncoder.getPosition() + distance;
        leftPidController.setSetpoint(leftSetPoint);
        rightPidController.setSetpoint(rightSetPoint);
    }

    public boolean travelThisMuch() {
        leftMaster.set(leftPidController.calculate(leftEncoder.getPosition()));
        rightMaster.set(rightPidController.calculate(rightEncoder.getPosition()));

        if ((Math.abs(leftSetPoint - leftEncoder.getPosition()) < 0.15)
                && (Math.abs(rightSetPoint - rightEncoder.getPosition()) < 0.15)) {
            return true;
        } else {
            return false;
        }
    }

    public void setTravelValMiddle(double distance) {
        midSetPoint = midEncoder.getPosition() + distance;
        midPidController.setSetpoint(midSetPoint);
    }

    public boolean travelThisMuchMiddle() {
        leftMaster.set(midPidController.calculate(midEncoder.getPosition()));


        if ((Math.abs(midSetPoint - midEncoder.getPosition()) < 0.15)) {
            return true;
        } else {
            return false;
        }
    }

    public void alignThePitch() {
        if (gyro.getPitch() > 5) {
            rightMaster.set(-0.4);
            leftMaster.set(-0.4);
        } else if (gyro.getPitch() < -5) {
            rightMaster.set(0.4);
            leftMaster.set(0.4);
        } else {
            rightMaster.set(0);
            leftMaster.set(0);
        }
    }
}
