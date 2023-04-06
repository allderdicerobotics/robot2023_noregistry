package frc.robot.commands;

import java.util.function.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.misc.Vec2;
import frc.robot.misc.Vec3;
import frc.robot.subsystems.DriveSubsystem;

public class CSBalance extends CommandBase {
    private Consumer<ChassisSpeeds> drivefn;
    private Supplier<Quaternion> quaternion_source;

    private static final PIDController BALANCING_PID = new PIDController(
        2.0, 0.0, 0.0
    );
    
    static {
        BALANCING_PID.setSetpoint(0.0);
    }

    // 2.5 degrees is the threshold for the charge station being level
    // We're going to be just a bit more conservative and use 2 degrees
    public static final double DEAD_ZONE = (2.0 / 180.0) * Math.PI;
    
    public static final Vec3 CS_AXIS = new Vec3(0.0, 0.1, 0.0); // RELATIVE TO STARTING POSITION I THINK

    public CSBalance(DriveSubsystem drive, Supplier<Quaternion> quaternion_source) {
        this.drivefn = speeds -> {
            drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
        };
        this.quaternion_source = quaternion_source;
    }

    @Override
    public void execute() {
        Vec3 axis = CS_AXIS;
        double angle = rot_wrt_cs();
        SmartDashboard.putNumber("balanced", Math.abs(angle));
        if (Math.abs(angle) < DEAD_ZONE) {
            this.drivefn.accept(new ChassisSpeeds());
        }
        double out_mag = BALANCING_PID.calculate(angle);
        Vec3 out_dir = Vec3.K_HAT.cross(axis);
        Vec3 motion_3d = out_dir.scale(out_mag);
        Vec2 motion = motion_3d.drop_z();
        this.drivefn.accept(new ChassisSpeeds(motion.x, motion.y, 0.0));
    }

    double rot_wrt_cs() {
            // You know how a vector can be "decomposed" into
            // a component parallel to a particular vector and one perpendicular to it?
            // You can do the same thing for rotations -
            // "decompose" it into a "twist" along a particular axis and a "swing" along a perpendicular axis.
            // We're performing the Quaternion Swing-Twist Decomposition to extract the component of the rotation
            // along the axis that the charge station rotates about.
            // We then convert into a rotation vector, and dotting this with the original axis gives the amount of rotation
            // along the twist axis - exactly what we want!
            // Feel free to poke around in the linear/quaternion algebra defined in Vec3 if you want, it's cool stuff. 
            Quaternion q = quaternion_source.get();
            // SmartDashboard.putNumberArray("q", q[1]);
            Vec3 twist_axis = CS_AXIS;
            Quaternion twist = twist_axis.quaternion_twist_component(q);
            SmartDashboard.putNumber("y ang", Math.asin(q.getY())*2);

            Vec3 twist_rotation_vector = Vec3.rotation_vector_from_quaternion(twist);
            
            return twist_rotation_vector.dot(twist_axis);
    }
}
