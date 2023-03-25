/*package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.misc.ControlConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToScore extends PIDSubsystem {
    public AlignToScore(Pose2d currentPose, Pose2d desiredPose, DriveSubsystem drive){
        super(new PIDController(ControlConstants.Driving.DRIVE_KP,ControlConstants.Driving.DRIVE_KI,ControlConstants.Driving.DRIVE_KD));
        // getController().setTolerance();
        setSetpoint(desiredPose.getX() + desiredPose.getY() + desiredPose.getRotation().getDegrees());
    }
    
}
*/