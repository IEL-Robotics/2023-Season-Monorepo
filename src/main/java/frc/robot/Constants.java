package frc.robot;

public class Constants {

    public static class ChassisConstants {
        public final static int idLeftMaster = 52;
        public final static int idLeftSlave = 53;
        public final static int idRigthMaster = 51;
        public final static int idRightSlave =50;
        public final static int idMidMaster = 5;
        public final static int idMidSlave = 4;

        public final static double lowerOutput = 0.4;
        public final static double higherOutput = 0.8;

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

    public static class ControllerConstants {
        public final static int turboButton = 6;
    }
}
