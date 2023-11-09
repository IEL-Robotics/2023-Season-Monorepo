package frc.robot;

public final class Constants {
  public final class ChassisConstants {
    public final static int idLeftMaster = 52;
    public final static int idLeftSlave = 54;
    public final static int idRightMaster = 56;
    public final static int idRightSlave = 50;

    public final static double higherOutput = 0.7;
    public final static double lowerOutput = 0.3;
  }

  public static class PIDConstants {
    public static double kP = 6e-5;
    public static double kI = 1e-6;
    public static double kD = 0;
    public static double kIz = 0;
    public static double kFF = 0.000015;
    public static double kMaxOutput = 1;
    public static double kMinOutput = -1;
}

  public static class ArmConstants {
    public final static int idRightTelescope = 2;
    public final static int idLeftTelescope = 55;
}


}
