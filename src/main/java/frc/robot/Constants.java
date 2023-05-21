package frc.robot;

public class Constants {

    public static class ChassisConstants {
        public final static int idLeftMaster = 52;
        public final static int idLeftSlave = 53;
        public final static int idRigthMaster = 51;
        public final static int idRightSlave =50;
        public final static int idMidMaster = 57;
        public final static int idMidSlave = 56;
        

        public final static double lowerOutput = 1;
        public final static double higherOutput = 1;

        public final static double taxiDistance = 4;

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
        public final static int idRightTelescope = 54;
        public final static int idLeftTelescope = 55;

        public final static double errorRange = 40; // highly unstable, must be careful
        public final static double maxSpeedPid = 0.8; // prevent overshoot, might cause undershoot (valla weiß ich nicht)

        public final static double gridPos = 1000;
        public final static double upPos = 0;
        public final static double groundPos = -1000;
    }

    public static class PneumaticsConstants {
        public final static int idGripF = 6;
        public final static int idGripR = 7;

        public final static int idAnkleF = 0;
        public final static int idAnkleR = 1;
    }

    public static class ControllerConstants {
        public final static int idMainController = 0;
        public final static int idCopilotController = 1;

        public final static int turboButton = 6;
        
    }
}
