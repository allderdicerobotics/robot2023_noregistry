package frc.robot.misc;

import edu.wpi.first.math.geometry.*;

// Represents a vector in 3D
public class Vec3 {
    public final double x;
    public final double y;
    public final double z;

    public static final Vec3 I_HAT = new Vec3(1.0, 0.0, 0.0);
    public static final Vec3 J_HAT = new Vec3(0.0, 1.0, 0.0);
    public static final Vec3 K_HAT = new Vec3(0.0, 0.0, 1.0);

    public Vec3() {
        this(0.0, 0.0, 0.0);
    }

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public String toString() {
        return String.format("[%.4f, %.4f, %.4f]", x, y, z);
    }

    // Calculate the dot product
    public double dot(Vec3 other) {
        return (
            0.0
            + (this.x * other.x)
            + (this.y * other.y)
            + (this.z * other.z)
        );
    }

    // Calculate the cross product
    public Vec3 cross(Vec3 other) {
        Vec3 a = this;
        Vec3 b = other;
        return new Vec3(
            (a.y * b.z) - (a.z * b.y),
            (a.z * b.x) - (a.x * b.z),
            (a.x * b.y) - (a.y * b.x)
        );
    }

    // Scale a vector by a scalar
    public Vec3 scale(double scalar) {
        return new Vec3(
            this.x * scalar,
            this.y * scalar,
            this.z * scalar
        );
    }

    // Vector projection
    // This will trigger a div-by-zero if onto is the zero vector
    // Don't do that!
    public Vec3 proj_onto(Vec3 onto) {
        return onto.scale(dot(onto) / onto.dot(onto));
    }

    // Represent the vector as a pure quaternion (one with no real part)
    public Quaternion as_pure() {
        return new Quaternion(0.0, x, y, z);
    }

    // Drop the real component (scalar part) of a quaternion to obtain a vector
    // In other words, this gets the vector part of a quaternion
    public static Vec3 drop_real(Quaternion q) {
        return new Vec3(
            q.getX(), q.getY(), q.getZ()
        );
    }
    
    // Rotate the vector, using a quaternion to represent the rotation
    public Vec3 rotate_by(Quaternion q) {
        Quaternion q_prime = q
            .times(as_pure())
            .times(q.inverse());
        return drop_real(q_prime);
    }
    
    // Convert a quaternion to a rotation vector
    // The output is a vector that forms the axis of the rotation, with a norm (length) that is
    // just the angle twisted about that axis in radians
    public static Vec3 rotation_vector_from_quaternion(Quaternion q) {
        double theta = Math.acos(q.getW());
        // Avoid div-by-zero, theta ~= 0 => sin(theta) ~= 0
        if (Math.abs(theta) < 0.000001) {
            return new Vec3(); // This isn't a fallback or anything, it's the correct answer
        }
        double fac = theta / Math.sin(theta);
        fac *= 2;
        return Vec3.drop_real(q).scale(fac);
    }

    // Perform the swing-twist decomposition of a quaternion
    // This just returns the twist component, it's easy to get the swing component as q.times(twist.inverse())
    // This is kind of like the projection but for rotatio- wait a minute, it literally *is*
    // the projection but for rotations! You project the vector part of q and keep the scalar part the same,
    // but have to remember to normalize it at the end because rotations only make sense in the context of unit
    // quaternions! Wowzers, math is heck'n cool!
    public Quaternion quaternion_twist_component(Quaternion q) {
        Vec3 projection = drop_real(q).proj_onto(this);
        Quaternion twist = new Quaternion(
            q.getW(), projection.x, projection.y, projection.z
        ).normalize();
        return twist;
    }

    // Drop the z component to convert it to a 2D vector
    // Basically just project it onto the x-y plane
    public Vec2 drop_z() {
        return new Vec2(x, y);
    }
}
