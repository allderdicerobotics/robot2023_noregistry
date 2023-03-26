package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.misc.*;

public class Tower extends SubsystemBase {
    // PneumaticsControlModule pcm = new PneumaticsControlModule();
    
    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 7);

    public void moveForward() {
        m_solenoid.set(true);
    }

    public void moveReverse() {
        m_solenoid.set(false);
    }

    /*public double getRotationOffset() {
        // return angles in radians where 0 is parallel to the ground
        if (solenoid.get()) {
                return 0.0; // TODO FIXME
        } else {
                return 0.0; // TODO FIXME
        }
    }*/

    public double getWaitTime() {
        return 0.5;

    }

    public Command raiseAndWait() {
        return Commands.sequence(
            new InstantCommand(this::moveReverse, this),
            new WaitCommand(1)
        );
    }
}
