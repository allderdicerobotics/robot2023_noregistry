package frc.robot.misc;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.SerialPort;

public final class NavX {
    public static AHRS ahrs = new AHRS(SerialPort.Port.kUSB);
    public static double getRate(){
        return ahrs.getRate();
    }
    public static Rotation2d getAngle(){
        return ahrs.getRotation2d();
    }
    // public get_quaternion(){

    // }
}
