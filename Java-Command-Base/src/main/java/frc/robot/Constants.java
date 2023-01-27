// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kRightFrontMotorPort = 50;
    public static final int kRightBackMotorPort = 53;
    public static final int kLeftFrontMotorPort = 51;
    public static final int kLeftBackMotorPort = 52;
    public static final int kMidFirstMotorPort = 1;
    public static final int kMidSecondMotorPort = 2;
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
}
