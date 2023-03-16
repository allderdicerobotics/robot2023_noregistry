package frc.robot.subsystems;
import javax.xml.xpath.XPath;

import org.w3c.dom.xpath.XPathNamespace;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.misc.NavX;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule[] swerveModules = Constants.Port.SWERVE_MODULES;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Prop.Drivetrain.CHASSIS_AXIS_ACC_MAX, Constants.Prop.Drivetrain.CHASSIS_AXIS_DEC_MAX, 0);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Prop.Drivetrain.CHASSIS_AXIS_ACC_MAX, Constants.Prop.Drivetrain.CHASSIS_AXIS_DEC_MAX, 0);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Prop.Drivetrain.CHASSIS_ANG_ACC_MAX);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          Constants.Prop.Drivetrain.KINEMATICS,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          swerveModules[0].getPosition(),
          swerveModules[1].getPosition(),
          swerveModules[2].getPosition(),
          swerveModules[3].getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          swerveModules[0].getPosition(),
          swerveModules[1].getPosition(),
          swerveModules[2].getPosition(),
          swerveModules[3].getPosition()
        },
        pose);
  }

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

  

    var swerveModuleStates =
        Constants.Prop.Drivetrain.KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : computeChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Prop.Drivetrain.CHASSIS_AXIS_SPEED_MAX);
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
        desiredStates, Constants.Prop.Drivetrain.CHASSIS_AXIS_SPEED_MAX);
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
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  public ChassisSpeeds computeChassisSpeeds(double axFwd, double axStrafe, double axRot) {

    
    

    double speedFwd = xLimiter.calculate(MathUtil.applyDeadband(axFwd, Constants.Comp.Joystick.DEADBAND_LINEAR))
          * Constants.Prop.Drivetrain.CHASSIS_AXIS_SPEED_MAX;
    double speedStrafe = yLimiter.calculate(MathUtil.applyDeadband(axStrafe, Constants.Comp.Joystick.DEADBAND_LINEAR))
        * Constants.Prop.Drivetrain.CHASSIS_AXIS_SPEED_MAX;
    double speedRot = rotLimiter.calculate(MathUtil.applyDeadband(axRot, Constants.Comp.Joystick.DEADBAND_ANG))
        * Constants.Prop.Drivetrain.CHASSIS_ANG_VEL_MAX;
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
    return m_gyro.getRate() * (Constants.Prop.Drivetrain.GYRO_REVERSED ? -1.0 : 1.0);
  }
}