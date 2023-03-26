package frc.robot.misc;

public final class ControlConstants {
    
    public static final class Driving {

        public static final double TURNING_KP = 1.6; //1.7
        public static final double TURNING_KI = 1.8; //2.0;
        public static final double TURNING_KD = 0.0;
        public static final double TURNING_KS = 0.5;
        public static final double TURNING_KV = 0.3;

        public static final double DRIVE_KP = 0.00005;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KFF = 0.001;
        public static final double DRIVE_MAX_ACCEL = 250.0;
        //public static final double DRIVE_MAX_VEL = 2500.0;

    }

    public static final class Arm {

        public static final double ARM_KP = 0.0000000000002; //0.00000000000005
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0;

        public static final double ARM_KS = 0; // TODO FIXME
        public static final double ARM_KG = 0.0; // how much gravity affects the arm
                                                // we'll leave this as zero since the gearbox
                                                // has such a crazy ratio
        public static final double ARM_KV = 0; // TODO FIXME
        public static final double ARM_KA = 0; // TODO FIXME


    }

    public static final class ChargeStationBalancing {

        public static final double KP = 2.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        // 2.5 degrees is the threshold for the charge station being level
        // We're going to be just a bit more conservative and use 2 degrees
        public static final double DEAD_ZONE = (2.0 / 180.0) * Math.PI;

    }

}
