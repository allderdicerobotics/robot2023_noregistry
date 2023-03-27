package frc.robot.commands;

// import static frc.robot.Constants.AutoConstants.THETA_kD;
// import static frc.robot.Constants.AutoConstants.THETA_kI;
// import static frc.robot.Constants.AutoConstants.THETA_kP;
// import static frc.robot.Constants.AutoConstants.X_kD;
// import static frc.robot.Constants.AutoConstants.X_kI;
// import static frc.robot.Constants.AutoConstants.X_kP;
// import static frc.robot.Constants.AutoConstants.Y_kD;
// import static frc.robot.Constants.AutoConstants.Y_kI;
// import static frc.robot.Constants.AutoConstants.Y_kP;
// import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
// import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
// import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import frc.robot.misc.Constants;
import frc.robot.misc.ControlConstants;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to drive to a pose.
 */
public class AlignToScoreCommand extends CommandBase {
  
  private static final double TRANSLATION_TOLERANCE = 0.002;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      Constants.Prop.Drivetrain.AXIS_SPEED_MAX/2,
      Constants.Prop.Drivetrain.AXIS_ACC_MAX/2);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      Constants.Prop.Drivetrain.ANG_VEL_MAX/2,
      Constants.Prop.Drivetrain.ANG_ACC_MAX/2);

  private final ProfiledPIDController xController = new ProfiledPIDController(ControlConstants.Align.DRIVE_KP,ControlConstants.Align.DRIVE_KI,ControlConstants.Align.DRIVE_KD,DEFAULT_XY_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(ControlConstants.Align.DRIVE_KP,ControlConstants.Align.DRIVE_KI,ControlConstants.Align.DRIVE_KD,DEFAULT_XY_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(ControlConstants.Align.TURNING_KP,ControlConstants.Align.TURNING_KI,ControlConstants.Align.TURNING_KD,DEFAULT_OMEGA_CONSTRAINTS);
  private final DriveSubsystem drive;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;

  public AlignToScoreCommand(
        DriveSubsystem drive,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        boolean useAllianceColor) {
    this(drive, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, useAllianceColor);
  }

  public AlignToScoreCommand(
        DriveSubsystem drive,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        boolean useAllianceColor) {
    this.drive = drive;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;
    
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(drive);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), 8.0137 - pose.getY()); // 8.0137 is the FIELD_WIDTH_METERS
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX(),goalPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY(),goalPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }
    SmartDashboard.putNumber("x speed (aligncmd)", xSpeed);
    
    SmartDashboard.putNumber("y speed (aligncmd)", ySpeed);
    
    SmartDashboard.putNumber("rot speed (aligncmd)", omegaSpeed);
    drive.drive(
      xSpeed, ySpeed, omegaSpeed, true);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

}
