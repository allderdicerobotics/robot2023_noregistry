package frc.robot.subsystems;
// import javax.xml.xpath.XPath;

// import org.w3c.dom.xpath.XPathNamespace;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.misc.DrivetrainLimiter;
import frc.robot.misc.NavX;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule[] swerveModules = Constants.Port.SWERVE_MODULES;

  private final DrivetrainLimiter xLimiter = new DrivetrainLimiter(Constants.Prop.Drivetrain.AXIS_ACC_MAX, Constants.Prop.Drivetrain.AXIS_DEC_MAX);
  private final DrivetrainLimiter yLimiter = new DrivetrainLimiter(Constants.Prop.Drivetrain.AXIS_ACC_MAX, Constants.Prop.Drivetrain.AXIS_DEC_MAX);
  private final DrivetrainLimiter rotLimiter = new DrivetrainLimiter(Constants.Prop.Drivetrain.ANG_ACC_MAX, Constants.Prop.Drivetrain.ANG_DEC_MAX);


  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.Prop.Drivetrain.MODULE_LOCATIONS[0],
      Constants.Prop.Drivetrain.MODULE_LOCATIONS[1],
      Constants.Prop.Drivetrain.MODULE_LOCATIONS[2],
      Constants.Prop.Drivetrain.MODULE_LOCATIONS[3]
  );

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          kinematics,
          NavX.getAngle(),
          new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  // @Override
  // public void periodic() {
  //   // Update the odometry in the periodic block
  //   m_odometry.update(
  //       NavX.getAngle(),
  //       new SwerveModulePosition[] {
  //         swerveModules[0].getPosition(),
  //         swerveModules[1].getPosition(),
  //         swerveModules[2].getPosition(),
  //         swerveModules[3].getPosition()
  //       });
  // }

  // /**
  //  * Returns the currently-estimated pose of the robot.
  //  *
  //  * @return The pose.
  //  */
  // public Pose2d getPose() {
  //   return m_odometry.getPoseMeters();
  // }

  // /**
  //  * Resets the odometry to the specified pose.
  //  *
  //  * @param pose The pose to which to set the odometry.
  //  */
  // public void resetOdometry(Pose2d pose) {
  //   m_odometry.resetPosition(
  //       NavX.getAngle(),
  //       new SwerveModulePosition[] {
  //         swerveModules[0].getPosition(),
  //         swerveModules[1].getPosition(),
  //         swerveModules[2].getPosition(),
  //         swerveModules[3].getPosition()
  //       },
  //       pose);
  // }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);

  
    SmartDashboard.putNumber("robot angle", NavX.getAngle().getDegrees());
    double[] filteredInputs = joystickFiltering(xSpeed, ySpeed, rot);
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(filteredInputs[0]*Constants.Prop.Drivetrain.AXIS_SPEED_MAX, filteredInputs[1]*Constants.Prop.Drivetrain.AXIS_SPEED_MAX, filteredInputs[2]*Constants.Prop.Drivetrain.ANG_VEL_MAX, NavX.getAngle())//rot, m_gyro.getRotation2d())
                : computeChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Prop.Drivetrain.AXIS_SPEED_MAX);
    //System.out.println("drive :)");
    swerveModules[0].setDesiredState(swerveModuleStates[0]);
    swerveModules[1].setDesiredState(swerveModuleStates[1]);
    swerveModules[2].setDesiredState(swerveModuleStates[2]);
    swerveModules[3].setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Prop.Drivetrain.AXIS_SPEED_MAX);
      swerveModules[0].setDesiredState(desiredStates[0]);
      swerveModules[1].setDesiredState(desiredStates[1]);
      swerveModules[2].setDesiredState(desiredStates[2]);
      swerveModules[3].setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    swerveModules[0].reset();
    swerveModules[1].reset();
    swerveModules[2].reset();
    swerveModules[3].reset();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    NavX.ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return NavX.getAngle().getDegrees();
  }
  public Rotation2d getGyroscopeRotation() {
    return NavX.ahrs.getRotation2d();
  }
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition [] {
      swerveModules[0].getPosition(),
      swerveModules[1].getPosition(),
      swerveModules[2].getPosition(),
      swerveModules[3].getPosition()
    };
}
  public ChassisSpeeds computeChassisSpeeds(double axFwd, double axStrafe, double axRot) {

    double speedFwd = xLimiter.calculate(Math.pow(MathUtil.applyDeadband(axFwd, Constants.Comp.Joystick.DEADBAND_LINEAR),3))
          * Constants.Prop.Drivetrain.AXIS_SPEED_MAX;
    SmartDashboard.putNumber("input", axFwd);
    SmartDashboard.putNumber("new input", speedFwd);
    double speedStrafe = yLimiter.calculate(Math.pow(MathUtil.applyDeadband(axStrafe, Constants.Comp.Joystick.DEADBAND_LINEAR),3))
        * Constants.Prop.Drivetrain.AXIS_SPEED_MAX;
    double speedRot = rotLimiter.calculate(Math.pow(MathUtil.applyDeadband(axRot, Constants.Comp.Joystick.DEADBAND_ANG),3))
        * Constants.Prop.Drivetrain.ANG_VEL_MAX;
    return new ChassisSpeeds(
       speedFwd, speedStrafe, speedRot
    );
}
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return NavX.ahrs.getRate() * (Constants.Prop.Drivetrain.GYRO_REVERSED ? -1.0 : 1.0);
  }
  public double[] joystickFiltering(double x, double y, double rot) {
    SmartDashboard.putNumber("joystick in", x);
    double speedFwd = xLimiter.calculate(Math.pow(MathUtil.applyDeadband(x, Constants.Comp.Joystick.DEADBAND_LINEAR),3));
    double speedStrafe = yLimiter.calculate(Math.pow(MathUtil.applyDeadband(y, Constants.Comp.Joystick.DEADBAND_LINEAR),3));
    double speedRot = rotLimiter.calculate(Math.pow(MathUtil.applyDeadband(rot, Constants.Comp.Joystick.DEADBAND_ANG),3));
    SmartDashboard.putNumber("speedFwd", speedFwd);
    double[] output = {speedFwd, speedStrafe, speedRot};
    return output;
  }

}
