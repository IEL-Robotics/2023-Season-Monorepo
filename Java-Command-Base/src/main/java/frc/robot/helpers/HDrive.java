// This Drive Thingy is experimental don't copy it @Umut at least for now.
// also I might be too lazy to finish it anyways

package frc.robot.helpers;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

public class HDrive extends DifferentialDrive {

    private boolean m_reported;
    private final CANSparkMax m_frontLeftMotor, m_frontRightMotor, m_rearLeftMotor, m_rearRightMotor;
    private final VictorSPX m_midLeftMotor, m_midRightMotor;

    @SuppressWarnings("MemberName")
    public static class WheelSpeeds {
        public double fronLeft, frontRight, rearLeft, rearRight, midLeft, midRight;

        public WheelSpeeds(
                double frontLeft, double frontRight, double rearLeft, double rearRight, double midLeft,
                double midRight) {
            this.fronLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
            this.midLeft = midLeft;
            this.midRight = midRight;
        }
    }

    public static WheelSpeeds driveCartesianIK(double xSpeed, double ySpeed, double zRotation) {
        return driveCartesianIK(xSpeed, ySpeed, zRotation, new Rotation2d());
    }

    private static WheelSpeeds driveCartesianIK(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {
        xSpeed = MathUtil.clamp(xSpeed, -1., 1);
        ySpeed = MathUtil.clamp(ySpeed, -1., 1);

        var input = new Translation2d(xSpeed, ySpeed).rotateBy(gyroAngle.unaryMinus());

        double[] wheelSpeeds = new double[6];

        wheelSpeeds[MotorType.kFrontLeft.value] = input.getX() + input.getY() + zRotation;
        wheelSpeeds[MotorType.kFrontRight.value] = input.getX() - input.getY() - zRotation;
        wheelSpeeds[MotorType.kRearLeft.value] = input.getX() - input.getY() + zRotation;
        wheelSpeeds[MotorType.kRearRight.value] = input.getX() + input.getY() - zRotation;
        wheelSpeeds[4] = input.getX();
        wheelSpeeds[5] = input.getX();

        normalize(wheelSpeeds);

        return new WheelSpeeds(wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value],
                wheelSpeeds[MotorType.kRearLeft.value],
                wheelSpeeds[MotorType.kRearRight.value],
                wheelSpeeds[4],
                wheelSpeeds[5]);
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        driveCartesian(xSpeed, ySpeed, zRotation, new Rotation2d());
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {
        if (!m_reported) {
            HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_MecanumCartesian);
            m_reported = true;
        }
        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);

        var speeds = driveCartesianIK(xSpeed, ySpeed, zRotation, gyroAngle);

        m_frontLeftMotor.set(speeds.fronLeft * m_maxOutput);
        m_frontRightMotor.set(speeds.frontRight * m_maxOutput);
        m_rearLeftMotor.set(speeds.rearLeft * m_maxOutput);
        m_rearRightMotor.set(speeds.rearRight * m_maxOutput);
        m_midLeftMotor.set(VictorSPXControlMode.PercentOutput, speeds.midLeft * m_maxOutput);
        m_midRightMotor.set(VictorSPXControlMode.PercentOutput, speeds.midRight * m_maxOutput);

        feed();
    }

    public HDrive(
            CANSparkMax frontLeftMotor,
            CANSparkMax frontRightMotor,
            CANSparkMax rearLeftMotor,
            CANSparkMax rearRightMotor,
            VictorSPX midRightMotor,
            VictorSPX midLeftMotor) {
        super(frontRightMotor, frontLeftMotor);
        this.m_frontLeftMotor = frontLeftMotor;
        this.m_frontRightMotor = frontRightMotor;
        this.m_rearLeftMotor = rearLeftMotor;
        this.m_rearRightMotor = rearRightMotor;
        this.m_midLeftMotor = midLeftMotor;
        this.m_midRightMotor = midRightMotor;

        SendableRegistry.addChild(this, m_frontLeftMotor);
        SendableRegistry.addChild(this, m_frontRightMotor);
        SendableRegistry.addChild(this, m_rearLeftMotor);
        SendableRegistry.addChild(this, m_rearRightMotor);
        SendableRegistry.addChild(this, m_midLeftMotor);
        SendableRegistry.addChild(this, m_midRightMotor);

        this.setSafetyEnabled(false);

    }

    @Override
    public void close() {
        SendableRegistry.remove(this);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getDescription());
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty(
                "Front Left Motor speed",
                m_frontLeftMotor::get,
                m_frontLeftMotor::set);
        builder.addDoubleProperty(
                "Front Right Motor speed",
                m_frontRightMotor::get,
                m_frontRightMotor::set);
        builder.addDoubleProperty(
                "Rear Left Motor speed",
                m_rearLeftMotor::get,
                m_rearRightMotor::set);
        builder.addDoubleProperty(
                "Rear Right Motor Speed",
                m_rearRightMotor::get,
                m_rearRightMotor::set);
        builder.addDoubleProperty(
                "Mid Left Motor speed",
                () -> m_midLeftMotor.getMotorOutputPercent(),
                (value) -> m_midLeftMotor.set(VictorSPXControlMode.PercentOutput, value));
        builder.addDoubleProperty(
                "Mid Right Motor speed",
                () -> m_midRightMotor.getMotorOutputPercent(),
                (value) -> m_midRightMotor.set(VictorSPXControlMode.PercentOutput, value));
    }

    @Override
    public void stopMotor() {
        this.m_frontRightMotor.stopMotor();
        this.m_frontLeftMotor.stopMotor();
        this.m_rearRightMotor.stopMotor();
        this.m_rearLeftMotor.stopMotor();
        this.m_midLeftMotor.set(VictorSPXControlMode.PercentOutput, 0);
        this.m_midRightMotor.set(VictorSPXControlMode.PercentOutput, 0);

        feed();
    }

    @Override
    public String getDescription() {
        return "H-Drive";
    }

}
