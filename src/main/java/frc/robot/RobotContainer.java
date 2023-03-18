// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArmSetpoints;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.WheelClaw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem drive = new DriveSubsystem();
  private final WheelClaw wheelClaw = new WheelClaw();
  private final Arm arm = new Arm();
  private final Tower tower = new Tower();
  private final PhotonCamera photonCamera = new PhotonCamera("IMX219");
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drive);
  private final ArmSetpoints armSetpoints = new ArmSetpoints(arm, tower);


  PathPlannerTrajectory examplePath = PathPlanner.loadPath("test", new PathConstraints(4, 3));
  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.

  //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  // eventMap.put("intakeDown", new IntakeDown());

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.

  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    poseEstimator::getCurrentPose,
    poseEstimator::setCurrentPose,
    drive.kinematics,
    new PIDConstants(1, 0, 0),
    new PIDConstants(0.1, 0, 0),
    drive::setModuleStates,
    null,
    true,
    drive
  );

  Command fullAuto = autoBuilder.followPath(examplePath);

  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(Constants.Port.Driver.CONTROLLER);
  Joystick m_buttonBoard = new Joystick(Constants.Port.Operator.BUTTONBOARD);
  Trigger xButton = new JoystickButton(m_driverController, 1);
  Trigger aButton = new JoystickButton(m_driverController, 2);
  Trigger bButton = new JoystickButton(m_driverController, 3);
  Trigger yButton = new JoystickButton(m_driverController, 4);

  Trigger leftBumper = new JoystickButton(m_driverController, 5);
  Trigger rightBumper = new JoystickButton(m_driverController, 6);
  
  Trigger cubePickupButton = new JoystickButton(m_buttonBoard, 10);
  Trigger conePickupButton = new JoystickButton(m_buttonBoard, 6);
  Trigger stowInButton = new JoystickButton(m_buttonBoard, 11);
  //Trigger playerStationButton = new JoystickButton(m_buttonBoard, );
  Trigger cubeSecondButton = new JoystickButton(m_buttonBoard, 4);
  Trigger coneSecondButton = new JoystickButton(m_buttonBoard, 7);
  // need more buttons
  Trigger cubeThirdButton = new JoystickButton(m_buttonBoard, 12); 
  Trigger coneThirdButton = new JoystickButton(m_buttonBoard, 9);



//4,6,8,9,10,11
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                drive.drive(
                    -m_driverController.getLeftY(),
                    -m_driverController.getLeftX(),
                    -m_driverController.getRightX(),
                    true),
            drive));
    
    wheelClaw.setDefaultCommand(new RunCommand(() -> wheelClaw.spinInSlowly(), wheelClaw));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .whileTrue(new InstantCommand(m_wheelClaw::spinIn));
    // joysticks
    aButton.whileTrue(new RunCommand(() -> wheelClaw.spinIn(), wheelClaw));
    xButton.whileTrue(new RunCommand(() -> wheelClaw.spinOut(), wheelClaw));

    bButton.onTrue(new InstantCommand(() -> arm.changeDesiredState(10)));
    yButton.onTrue(new InstantCommand(() -> arm.changeDesiredState(-10)));

    leftBumper.onTrue(new InstantCommand(() -> tower.moveForward()));
    rightBumper.onTrue(new InstantCommand(() -> tower.moveReverse()));


    // button board
    stowInButton.onTrue(armSetpoints.stowInside());
    //playerStationButton.onTrue(m_armSetpoints.playerStation());

    cubePickupButton.onTrue(armSetpoints.cubeGround());
    conePickupButton.onTrue(armSetpoints.coneGround());

    cubeSecondButton.onTrue(armSetpoints.cubeSecondLevel());
    coneSecondButton.onTrue(armSetpoints.coneSecondLevel());

    cubeThirdButton.onTrue(armSetpoints.cubeThirdLevel());
    coneThirdButton.onTrue(armSetpoints.coneThirdLevel());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand() {
    return fullAuto;
  }
}
// hey guys how are you- audrey (i snuck into the code)
