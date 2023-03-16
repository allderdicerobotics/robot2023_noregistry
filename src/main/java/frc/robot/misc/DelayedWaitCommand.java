package frc.robot.misc;

import java.util.function.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;

public class DelayedWaitCommand extends CommandBase {
    private Supplier<Double> timeSupplier;
    private double endTimestamp = 0.0;

    public DelayedWaitCommand(Supplier<Double> timeSupplier) {
        this.timeSupplier = timeSupplier;
    }

    @Override
    public void initialize() {
        endTimestamp = Timer.getFPGATimestamp() + timeSupplier.get();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTimestamp;
    }
}
