package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.misc.Constants;
import frc.robot.misc.ControlConstants;
import frc.robot.misc.ThriftyEncoder;
import frc.robot.misc.Constants.Prop;
import frc.robot.misc.Constants.Prop.Drivetrain;
import frc.robot.misc.ControlConstants.Driving;

//import com.gos.lib.properties.PidProperty;
//import com.gos.lib.properties.WpiProfiledPidPropertyBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.*;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final String name;
    // PidProperty PIDproperties;
    public final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;


    private final RelativeEncoder driveEncoder;
    private final ThriftyEncoder turningEncoder;
    private final SparkMaxPIDController drivePIDController;
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
            ControlConstants.Driving.TURNING_KP,
            ControlConstants.Driving.TURNING_KI,
            ControlConstants.Driving.TURNING_KD,
            new TrapezoidProfile.Constraints(
                    Constants.Prop.Drivetrain.TURNING_ANG_VEL_MAX,
                    Constants.Prop.Drivetrain.TURNING_ANG_ACC_MAX));
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
            ControlConstants.Driving.TURNING_KS,
            ControlConstants.Driving.TURNING_KV);

    public SwerveModule(int driveMotorID, int turningMotorID, ThriftyEncoder thriftyEncoder, String name) {
        this.name = name;
        // Setup hardware (motors and encoders)
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();
        
        
        driveMotor.setSmartCurrentLimit(40);
        
        turningMotor.setSmartCurrentLimit(20);

        turningMotor.setInverted(true);
        if (name != "BL" || name != "BR"){
            driveMotor.setInverted(true);
        }
        driveEncoder = driveMotor.getEncoder();
        
        turningEncoder = thriftyEncoder;


        
        // Setup controllers
        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(ControlConstants.Driving.DRIVE_KP, 0);
        drivePIDController.setI(ControlConstants.Driving.DRIVE_KI, 0);
        drivePIDController.setD(ControlConstants.Driving.DRIVE_KD, 0);
        drivePIDController.setOutputRange(-1, 1);
        drivePIDController.setSmartMotionMaxVelocity(10000, 0);
        drivePIDController.setFF(ControlConstants.Driving.DRIVE_KFF, 0);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.burnFlash();
        turningMotor.burnFlash();

    //     PIDproperties = new WpiProfiledPidPropertyBuilder("Swerve Turning", false, turningPIDController).addP(1.1)
    //             .addI(0).addD(0).build();
    }

    // Get the state of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), turningEncoder.get());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        // PIDproperties.updateIfChanged();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, turningEncoder.get());
        // -- SmartDashboard.putNumber("swerve", turningEncoder.getRawPosition().getRadians());
        //(turningEncoder.getRawPosition());
        // Calculate the drive output from the drive PID controller.
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningEncoder.get().getRadians(),
                state.angle.getRadians());
        final double turnFeedforwardOut = turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);
        final double driveMotorRPM = Constants.Prop.Drivetrain.WHEEL_MPS_TO_RPM.apply(state.speedMetersPerSecond);
        // System.out.println(driveEncoder.getPosition());
        // Set all of the motors.
        // setReference is a bit weird because the setpoint and feedforward values are
        // seperated, because feedforward has to run after the PID.
        drivePIDController.setReference(
                driveMotorRPM, // value for setpoint
                ControlType.kVelocity, // units for setpoint (rpm)
                0// units for feedforward
        );

        drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        drivePIDController.setSmartMotionMaxAccel(ControlConstants.Driving.DRIVE_MAX_ACCEL, 0);
        // Here we just add the feedforward component.
        // We couldn't do that with drivePIDController because they had different units
        // (or at least could).
        turningMotor.setVoltage(turnOutput + turnFeedforwardOut);

        SmartDashboard.putNumber("Drive " + name, driveEncoder.getPosition());
        SmartDashboard.putNumber("Turn " + name,turningEncoder.get().getDegrees());
        SmartDashboard.putNumber("State " + name, state.angle.getDegrees());
        SmartDashboard.putNumber("Speed " + name, state.speedMetersPerSecond);

        
    }

    public void reset() {
        driveEncoder.setPosition(0);
        turningEncoder.reset();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Constants.Prop.Drivetrain.WHEEL_ROT_TO_M.apply(driveEncoder.getPosition()), turningEncoder.get()
        );
    }

}
