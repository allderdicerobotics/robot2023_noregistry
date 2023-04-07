package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.WheelClaw;
import frc.robot.commands.ArmSetpoints;
import frc.robot.misc.ControlConstants;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Arm;

public class ArmWheelClawGroup {

    private Arm arm;

    private Tower tower;
    
    private WheelClaw wheelClaw;

    public ArmWheelClawGroup(Arm arm, Tower tower, WheelClaw wheelClaw) {
      this.arm = arm;
      this.tower = tower;
      this.wheelClaw = wheelClaw;
    }

    public Command autoSecondLevelCube() {
      ArmSetpoints armSetpoints = new ArmSetpoints(arm, tower);
      return Commands.sequence(
        new InstantCommand(() -> arm.setDesiredMotorState(40)),// Constants.Prop.Arm.ARM_CUBE_2)),
        new WaitCommand(1),
        new InstantCommand(() -> wheelClaw.fullOuttake(3)),
        new WaitCommand(0.5),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_DOWN_IN)),
        new WaitCommand(2)
      );
    }
    
}