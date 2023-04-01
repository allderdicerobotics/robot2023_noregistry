package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.misc.Constants;
import frc.robot.subsystems.*;

public class TestAllSubsystems extends SequentialCommandGroup {
    public TestAllSubsystems(DriveSubsystem drive, Claw claw, Arm arm, Tower tower, WheelClaw wheelClaw){
        ArmSetpoints armSetpoints = new ArmSetpoints(arm, tower);
        ArmWheelClawGroup armWheelClawGroup = new ArmWheelClawGroup(arm, tower, wheelClaw);
      
        addCommands(
            new RunCommand(() -> drive.drive(Constants.Prop.Drivetrain.AXIS_SPEED_MAX, Constants.Prop.Drivetrain.AXIS_SPEED_MAX, Constants.Prop.Drivetrain.ANG_VEL_MAX,true), drive).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf),
            new WaitCommand(2),
            new RunCommand(() -> drive.stop(), drive).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf),            
            armSetpoints.coneGround(),
            armWheelClawGroup.autoSecondLevelCube(),
            armSetpoints.stowInside(),
            new PrintCommand(Boolean.toString(tower.m_solenoid.isDisabled())));
    }
}
