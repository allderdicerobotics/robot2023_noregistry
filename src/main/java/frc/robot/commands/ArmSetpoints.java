package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Tower;
import frc.robot.misc.ControlConstants;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Arm;

public class ArmSetpoints {

    private Arm arm;

    private Tower tower;

    public ArmSetpoints(Arm arm, Tower tower) {
      this.arm = arm;
      this.tower = tower;
    }

    public Command stowInside() {
      return Commands.sequence(
        this.tower.raiseAndWait(),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_DOWN_IN))
      );
    }

    public Command cubeGround() {
      return Commands.sequence(
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_FLOOR_CUBE)),
        new WaitCommand(1.5),
        new InstantCommand(tower::moveForward)
      );
    }

    public Command coneGround() {
      return Commands.sequence(
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_FLOOR_CONE)),
        new WaitCommand(1.5),
        new InstantCommand(tower::moveForward)
      );
    }

    public Command cubeSecondLevel() {
      return Commands.sequence(
        new InstantCommand(tower::moveForward),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_CUBE_2))
      );
    }

    public Command coneSecondLevel() {
      return Commands.sequence(
        new InstantCommand(tower::moveForward),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_CONE_2))
      );
    }

    public Command cubeThirdLevel() {
      return Commands.sequence(
        new InstantCommand(tower::moveForward),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_CUBE_3))
      );
    }
    
    public Command coneThirdLevel() {
      return Commands.sequence(
        new InstantCommand(tower::moveForward),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_CONE_3))
      );
    }

    public Command playerStation() {
      return Commands.sequence(
        new InstantCommand(tower::moveForward),
        new InstantCommand(() -> arm.setDesiredMotorState(Constants.Prop.Arm.ARM_STATION))
      );
    }
    
}