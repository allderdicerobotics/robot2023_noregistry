package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.WheelClaw;
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
      return Commands.sequence(
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_CUBE_2)),
        new WaitCommand(2),
        new InstantCommand(() -> wheelClaw.fullOuttake()),
        new WaitCommand(4)
      );
    }
    
}