package frc.robot.misc;

import java.util.*;
import java.util.function.*;

import edu.wpi.first.math.filter.*;

// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.SwerveModule;

public final class Constants {

    // Ports, think DIO and CAN IDs
    // Anything that is a completely arbitrary aspect
    // of how things are laid out
    public static final class Port {

        public static final class Driver {
            
            public static final int CONTROLLER = 0;

            public static final int AX_FWD = 1;
            public static final int AX_STRAFE = 0;
            public static final int AX_ROT_CW = 2;

            public static final int BTN_SUPRESS_FOC = 6;

        }

        public static final class Operator {

            public static final int BUTTONBOARD = 2;
        }

        public static final SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(
                7, // 7, 6, 9, 8, 3, 2, 5, 4
                6,
                new ThriftyEncoder(new AnalogInput(2)).shiftDegs(-67), //30 deg
                "FL"
            ),
            new SwerveModule(
                5,
                4,
                new ThriftyEncoder(new AnalogInput(1)).shiftDegs(30),
                
                "FR"
            ),
            new SwerveModule(
                9,
                8,
                new ThriftyEncoder(new AnalogInput(3)).shiftDegs(-54), //dec
                "BL"
            ),
            new SwerveModule(
                3,
                2,
                new ThriftyEncoder(new AnalogInput(0)).shiftDegs(-90), // -270 good for drive -90 good for turn
                "BR"
            ),
            
        };

        public static final class TowerArmClaw {

            public static int CAN_ID_ARM = 11; // TODO FIXME
            public static int CAN_ID_CLAW = 12; // TODO FIXME

        }

    }

    // Properties, think gear ratios and dimensions
    // Anything that is some measurement specific to
    // what we have created
    public static final class Prop {

        public static final class Drivetrain {

            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(0.381, 0.381),
                new Translation2d(0.381, -0.381),
                new Translation2d(-0.381, 0.381),
                new Translation2d(-0.381, -0.381)

            );

            
            public static final double TURNING_ANG_VEL_MAX = Math.PI * 100;
            public static final double TURNING_ANG_ACC_MAX = Math.PI * 100;


            public static final double WHEEL_RADIUS = 0.0508;
            public static final double WHEEL_GEAR_RATIO = 6.55;

            // Meters per second to rotations per minute
            public static final UnaryOperator<Double> WHEEL_MPS_TO_RPM = mps -> (
                2 * mps * (60 /( 2 * Math.PI * WHEEL_RADIUS)) * WHEEL_GEAR_RATIO
            );

            public static final double CHASSIS_AXIS_SPEED_MAX = 5.0;
            public static final double CHASSIS_AXIS_ACC_MAX = 0.05;
            public static final double CHASSIS_AXIS_DEC_MAX = -1.0;
            public static final double CHASSIS_ANG_VEL_MAX = Math.PI;
            public static final double CHASSIS_ANG_ACC_MAX = 0.1;

            public static final boolean GYRO_REVERSED = false;

        }

        public static final class SnakeEyesCamera {

            public static final Transform3d XFORM_ROBOT_TO_CAMERA = new Transform3d(
                new Translation3d(),
                new Rotation3d(Math.PI / 2, Math.PI / 2, 0.0)
            );

        }

        public static final class Tower {

            public static final DoubleSolenoid SOLENOID = null; // TODO FIXME

            // angles in radians for forward and reverse positions
            // 0 = parallel to the ground
            public static final double ROTATION_RADS_FORWARD = 9.0; // TODO FIXME
            public static final double ROTATION_RADS_REVERSE = 0.0; // TODO FIXME

            // the time for the tower to move back and forth
            public static final double TIME_SECONDS_FOR_ACTION = 1.0; // TODO FIXME

        }
    
    public static final class Arm {

        public static final double ARM_VEL_MAX = 1000;
        public static final double ARM_ACC_MAX = 1500;
        public static final double ARM_DOWN_IN = 0;
        public static final double ARM_FLOOR_CUBE = 70;
        public static final double ARM_FLOOR_CONE = 55;
        public static final double ARM_CUBE_2 = 70;
        public static final double ARM_CONE_2 = 140;
        public static final double ARM_CUBE_3 = 150;
        public static final double ARM_CONE_3 = 165;
        public static final double ARM_STATION = 0;
        
    }
    }

    // Component properties, think ticks per rotation
    // of an encoder or the HC-SR04's conversion factors
    // Any measurement specific to a component we are using
    // Components here don't include stuff like chassis dims
    // That *could* be arbitrary because we could mount stuff
    // differently
    public static final class Comp {

        public static final class ThriftyEnc {

            public static final double STD_READ_VOLTAGE_MAX = 4.8;

        }

        public static final class Joystick {

            public static final double DEADBAND_LINEAR = 0.1; //0.05
            public static final double DEADBAND_ANG = 0.1;

            public static final double RATE_LIMIT = 3.0;

        }

    }

    // Physical constants, think little g
    // Anything that, well, is a constant from physics
    public static final class Phys {}

}
