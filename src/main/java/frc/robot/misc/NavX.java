package frc.robot.misc;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.SerialPort;

public final class NavX {
    public static AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

    // Often, the NavX starts thinking it is slightly rotated
    // This offset persists and doesn't really change
    // We can account for it by measuring it right at startup and undoing it every time
    // This is kind of annoying but oh well
    // ONLY QUATERNION STUFF IS CORRECTED
    private Quaternion correction;

    public NavX() {
        // See comment above `correction` field
        Quaternion nil_rotation = new Quaternion(1.0, 0.0, 0.0, 0.0);
        this.correction = get_quaternion_uncorrected()
            .inverse()
            .times(nil_rotation);        
    }

    public static double getRate(){
        return ahrs.getRate();
    }
    public static Rotation2d getAngle(){
        return ahrs.getRotation2d();
    }
    
    // QUATERNION STUFF <below>
    // Quarternions are a four-dimensional number system used to represent rotations in 3D
    // You don't have to worry about how they work (I don't really understand them either)
    // But just know that they're a mind-bogglingly clever way to represent 3D rotations
    public Quaternion get_quaternion_uncorrected() {
        return new Quaternion(
            ahrs.getQuaternionW(),
            ahrs.getQuaternionX(),
            ahrs.getQuaternionY(),
            ahrs.getQuaternionZ()
        );
    }

    public Quaternion get_quaternion() {
        Quaternion uncorrected = get_quaternion_uncorrected();
        return uncorrected.times(correction);
    }

    public void reset() {
        ahrs.reset();
    }
}
