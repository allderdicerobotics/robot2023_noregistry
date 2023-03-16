package frc.robot.misc;

// Represents a vector in 2D
public class Vec2 {
    public final double x;
    public final double y;

    public static final Vec2 I_HAT = new Vec2(1.0, 0.0);
    public static final Vec2 J_HAT = new Vec2(0.0, 1.0);

    public Vec2() {
        this(0.0, 0.0);
    }

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public String toString() {
        return String.format("[%.4f, %.4f]", x, y);
    }

    // Rotate a vector by an angle in radians
    // Note that this only makes sense for 2D, if you want to rotate a 3D vector
    // go learn about quaternions
    public Vec2 rotate(double theta) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        return new Vec2(
            (cos * x) - (sin * y),
            (sin * x) - (cos * y)
        );
    }
}
