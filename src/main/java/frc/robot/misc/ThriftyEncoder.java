package frc.robot.misc;

import java.util.function.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.AnalogInput;

public class ThriftyEncoder implements Supplier<Rotation2d> {
    private final AnalogInput input;
    private final double read_voltage_max;
    public Rotation2d offset = new Rotation2d(0.0);

    public ThriftyEncoder(AnalogInput input) {
        this(input, Constants.Comp.ThriftyEnc.STD_READ_VOLTAGE_MAX);
    }

    public ThriftyEncoder(AnalogInput input, double read_voltage_max) {
        this.input = input;
        this.read_voltage_max = read_voltage_max;
    }

    // Does not include offset
    public Rotation2d getRawPosition() {
        return new Rotation2d(
            (input.getVoltage() * 2 * Math.PI) / read_voltage_max
        );
    }


    // Does include offset
    public Rotation2d get() {
        return getRawPosition().plus(offset);
    }

    public ThriftyEncoder reset() {
        offset = getRawPosition();
        return this;
    }

    // Update the offset (radians)
    public ThriftyEncoder shiftRads(double radians) {
        offset = offset.plus(new Rotation2d(radians));
        return this;
    }

    // Update the offset (degrees)
    public ThriftyEncoder shiftDegs(double degrees) {
        offset = offset.plus(Rotation2d.fromDegrees(degrees));
        return this;
    }
}
