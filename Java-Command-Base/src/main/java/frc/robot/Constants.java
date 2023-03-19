// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  public static class RobotConstants {
    public static final int kRightFrontMotorPort = 51;
    public static final int kRightBackMotorPort = 50;
    public static final int kLeftFrontMotorPort = 52;
    public static final int kLeftBackMotorPort = 53;
    public static final int kMidFirstMotorPort = 57;
    public static final int kMidSecondMotorPort = 56;
    public static final int kArmRightMotorPort = 54;
    public static final int kArmLeftMotorPort = 55;
  }

  public static class PIDCoefficients {
    public static double kP = 6e-5;
    public static double kI = 1e-6;
    public static double kD = 0;
    public static double kIz = 0;
    public static double kFF = 0.000015;
    public static double kMaxOutput = 1;
    public static double kMinOutput = -1;
  }

  public static class DriveConstants {
    public static final double ksVolts = 0.22;

    public static final double kvVoltSecondsPerMeter = 1.98;

    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!

    public static final double kPDriveVel = 8.5;
    public static final double kTrackwidthMeters = 0.69;

    public static final DifferentialDriveKinematics kDriveKinematics =

        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;

    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;

    public static final double kRamseteZeta = 0.7;

  }
}
