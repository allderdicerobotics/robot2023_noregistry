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

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final WheelClaw m_wheelClaw = new WheelClaw();
  private final Arm m_arm = new Arm();
  private final Tower m_tower = new Tower();
  private final PhotonCamera snakeEyes = new PhotonCamera("snakeeyes");
  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(snakeEyes, m_robotDrive);
  private final ArmSetpoints m_armSetpoints = new ArmSetpoints(m_arm, m_tower);

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
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -m_driverController.getLeftY(),
                    -m_driverController.getLeftX(),
                    -m_driverController.getRightX(),
                    false),
            m_robotDrive));
    
    m_wheelClaw.setDefaultCommand(new RunCommand(() -> m_wheelClaw.spinInSlowly(), m_wheelClaw));
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
    aButton.whileTrue(new RunCommand(() -> m_wheelClaw.spinIn(), m_wheelClaw));
    xButton.whileTrue(new RunCommand(() -> m_wheelClaw.spinOut(), m_wheelClaw));

    bButton.onTrue(new InstantCommand(() -> m_arm.changeDesiredState(10)));
    yButton.onTrue(new InstantCommand(() -> m_arm.changeDesiredState(-10)));

    leftBumper.onTrue(new InstantCommand(() -> m_tower.moveForward()));
    rightBumper.onTrue(new InstantCommand(() -> m_tower.moveReverse()));


    // button board
    stowInButton.onTrue(m_armSetpoints.stowInside());
    //playerStationButton.onTrue(m_armSetpoints.playerStation());

    cubePickupButton.onTrue(m_armSetpoints.cubeGround());
    conePickupButton.onTrue(m_armSetpoints.coneGround());

    cubeSecondButton.onTrue(m_armSetpoints.cubeSecondLevel());
    coneSecondButton.onTrue(m_armSetpoints.coneSecondLevel());

    cubeThirdButton.onTrue(m_armSetpoints.cubeThirdLevel());
    coneThirdButton.onTrue(m_armSetpoints.coneThirdLevel());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
   public Command getAutonomousCommand() {
  /*   // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
    return(new InstantCommand());
  }
}
// hey guys how are you- audrey (i snuck into the code)