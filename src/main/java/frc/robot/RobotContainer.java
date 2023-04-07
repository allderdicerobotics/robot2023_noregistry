// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AlignToScoreCommand;
import frc.robot.commands.ArmSetpoints;
import frc.robot.commands.ArmWheelClawGroup;
import frc.robot.commands.CSBalance;
import frc.robot.misc.Constants;
import frc.robot.misc.ControlConstants;
import frc.robot.misc.NavX;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.WheelClaw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


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
  private final ArmWheelClawGroup armWheelClawGroup = new ArmWheelClawGroup(arm, tower, wheelClaw);

  PathPlannerTrajectory examplePath = PathPlanner.loadPath("New New Path Copy Copy", new PathConstraints(1, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();


  public final NavX navx = new NavX();

  // FOR CHARGE BALANCIN IN AUTON
  public final Command csbalance_command = new CSBalance(drive, navx::get_quaternion);


  // PathPlannerTrajectory examplePath = PathPlanner.loadPath("please work oh god please", new PathConstraints(4 ,3));
  // //"please work oh god please", new PathConstraints(4, 3));
  // // This is just an example event map. It would be better to have a constant, global event map
  // // in your code that will be used by all path following commands.
  // eventMap = new HashMap<>();
  
  //eventMap.put("extend arm", armSetpoints.cubeSecondLevel());
  //eventMap.put("score cube lvl 2", new RunCommand(() -> wheelClaw.spinIn(), wheelClaw));
  // eventMap.put("intakeDown", new IntakeDown());

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.

  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    poseEstimator::getCurrentPose,
    poseEstimator::setCurrentPose,
    drive.kinematics,

    new PIDConstants(ControlConstants.Driving.DRIVE_KP, 0, 0),
    new PIDConstants(ControlConstants.Driving.TURNING_KP, ControlConstants.Driving.TURNING_KI, ControlConstants.Driving.TURNING_KD),
    //new PIDConstants(0.75,0.25,0),//ControlConstants.Driving.DRIVE_KP, 0, 0), //0.5
    //new PIDConstants(0.3,0,0), //ControlConstants.Driving.TURNING_KP, ControlConstants.Driving.TURNING_KI, ControlConstants.Driving.TURNING_KD), //0.5
    drive::setModuleStates,
    autoPath(eventMap),
    true,
    drive
  );

  PIDController xController = new PIDController(ControlConstants.Driving.DRIVE_KP, ControlConstants.Driving.DRIVE_KI, ControlConstants.Driving.DRIVE_KD);
  PIDController yController = new PIDController(ControlConstants.Driving.DRIVE_KP, ControlConstants.Driving.DRIVE_KI, ControlConstants.Driving.DRIVE_KD);
  PIDController thetaController = new PIDController(ControlConstants.Driving.TURNING_KP, ControlConstants.Driving.TURNING_KI, ControlConstants.Driving.TURNING_KD);

  Command fullAuto = autoBuilder.fullAuto(examplePath);
  Rotation2d zeroRot = new Rotation2d();
  //Pose2d goalPose = new Pose2d(14.5, 1.45, zeroRot);
  Pose2d goalPose = new Pose2d(0,0,zeroRot);
  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(Constants.Port.Driver.CONTROLLER);
  Joystick m_buttonBoard = new Joystick(Constants.Port.Operator.BUTTONBOARD);
  Trigger xButton = new JoystickButton(m_driverController, 1);
  Trigger aButton = new JoystickButton(m_driverController, 2);
  Trigger bButton = new JoystickButton(m_driverController, 3);
  Trigger yButton = new JoystickButton(m_driverController, 4);
  Trigger scoreButton = new JoystickButton(m_driverController, 14);
  Trigger rightTrigger = new JoystickButton(m_driverController, 8);
  Trigger leftBumper = new JoystickButton(m_driverController, 5);
  Trigger rightBumper = new JoystickButton(m_driverController, 6);
  
  Trigger cubePickupButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.cubePickupButton); //10
  Trigger conePickupButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.conePickupButton); //6

  Trigger stowInButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.stowInButton); //11
  Trigger playerStationButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.playerStation);
  Trigger cubeSecondButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.cubeSecondButton); //8
  Trigger coneSecondButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.coneSecondButton); //7
  // need more buttons
  Trigger cubeThirdButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.cubeThirdButton); //4
  Trigger coneThirdButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.coneThirdButton); //9
  
  Trigger intakeButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.intake);
  Trigger outtakeButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.outtake);
  Trigger microArmUpButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.microArmUp);
  Trigger microArmDownButton = new JoystickButton(m_buttonBoard, Constants.Prop.Buttonboard.microArmDown);



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
    
    wheelClaw.setDefaultCommand(new RunCommand(() -> wheelClaw.spinStop(), wheelClaw));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private HashMap<String,Command> autoPath(HashMap<String,Command> eventMap) {
    eventMap.put("lift arm", armWheelClawGroup.autoSecondLevelCube());
    return eventMap;
  }
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .whileTrue(new InstantCommand(m_wheelClaw::spinIn));
    // joysticks
    aButton.whileTrue(new RunCommand(() -> wheelClaw.spinIn(), wheelClaw));
    xButton.whileTrue(new RunCommand(() -> wheelClaw.spinOut(), wheelClaw));

    bButton.onTrue(new InstantCommand(() -> arm.changeDesiredState(3)));
    yButton.onTrue(new InstantCommand(() -> arm.changeDesiredState(-3)));

    leftBumper.onTrue(new InstantCommand(() -> tower.moveForward()));
    rightBumper.onTrue(new InstantCommand(() -> tower.moveReverse()));


    // leftTrigger.onTrue(new RunCommand(
    //   () ->
    //       drive.driveSlow(
    //           -m_driverController.getLeftY(),
    //           -m_driverController.getLeftX(),
    //           -m_driverController.getRightX(),
    //           true),
    //   drive));


    // button board
    stowInButton.onTrue(armSetpoints.stowInside());
    playerStationButton.onTrue(armSetpoints.playerStation());

    cubePickupButton.onTrue(armSetpoints.cubeGround());
    conePickupButton.onTrue(armSetpoints.coneGround());
    //conePickupButton.whileTrue(new RunCommand(() -> wheelClaw.spinIn(), wheelClaw));

    cubeSecondButton.onTrue(armSetpoints.cubeSecondLevel());
    coneSecondButton.onTrue(armSetpoints.coneSecondLevel());

    cubeThirdButton.onTrue(armSetpoints.cubeThirdLevel());
    coneThirdButton.onTrue(armSetpoints.coneThirdLevel());

    intakeButton.whileTrue(new RunCommand(() -> wheelClaw.spinIn(), wheelClaw));
    outtakeButton.whileTrue(new RunCommand(() -> wheelClaw.spinOut(), wheelClaw));

    microArmUpButton.onTrue(new InstantCommand(() -> arm.changeDesiredState(2)));
    microArmDownButton.onTrue(new InstantCommand(() -> arm.changeDesiredState(-2)));

    var alignCmd = new AlignToScoreCommand(drive,
      poseEstimator::getCurrentPose, 
      goalPose,
      false
      ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    //scoreButton.toggleOnTrue(alignCmd);
    scoreButton.onTrue(csbalance_command);
    // rightTrigger.onTrue(new InstantCommand( () -> drive.stop()));
    


  //   scoreButton.onTrue(new SwerveControllerCommand(poseEstimator.getScorePath(),
  //     poseEstimator::getCurrentPose,
  //     drive.kinematics,
  //     xController, yController , thetaController,
  //     drive::setModuleStates,
  //     drive));
        
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
