package frc.robot.subsystems;

import java.util.function.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import frc.robot.misc.*;

public class Arm extends SubsystemBase {
    private DigitalInput hallEffect = new DigitalInput(2);
    private CANSparkMax armMotor = new CANSparkMax(Constants.Port.TowerArmClaw.CAN_ID_ARM, MotorType.kBrushless);
    private RelativeEncoder encoder;

    private double desiredPosition = 0;
    //private CANCoder encoder = new CANCoder(1);
    /*private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
            ControlConstants.Arm.ARM_KP,
            ControlConstants.Arm.ARM_KI,
            ControlConstants.Arm.ARM_KD,
            new TrapezoidProfile.Constraints(
                    Constants.Prop.Arm.ARM_VEL_MAX,
                    Constants.Prop.Arm.ARM_ACC_MAX));
*/
    /*
    private ArmFeedforward armFF = new ArmFeedforward(
        ControlConstants.Arm.ARM_KS,
        ControlConstants.Arm.ARM_KG,
        ControlConstants.Arm.ARM_KV,
        ControlConstants.Arm.ARM_KA
    );
    */

    // See comment when set in constructor.
    private Supplier<Double> rotationOffsetSupplier;

    public Arm() {

        armMotor.setSmartCurrentLimit(40);
        
        /*
            |------|                     |------|
            |      |-------       -------|      |
            |------|                     |------|
            motor1                       motor2
        */
        //motor2.follow(motor1, true);
        // We need to have some way of knowing the absolute angle of the arm to do
        // feedforward properly (said absolute angle is relative to gravity). 
        //this.rotationOffsetSupplier = rotationOffsetSupplier;
        // Setup PIDF on spark MAX - use "smart motion"
        // to have a velocity component
        initMotor();
    }

    public void initMotor() {
        // Default parameters from example code for now.
        // Sets up the "smart motion" PIDF controller on the spark MAX
        // that allows us to call setReference to move to a specific position.
        armMotor.restoreFactoryDefaults();

        SparkMaxPIDController pid = armMotor.getPIDController();
        encoder = armMotor.getEncoder();
        encoder.se
        int smartMotionSlot = 0;
        pid.setP(0.0003);
        pid.setI(0.000001);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF(0.0002);
        pid.setOutputRange(-1, 1);
        pid.setSmartMotionMaxVelocity(Constants.Prop.Arm.ARM_VEL_MAX, smartMotionSlot);
        pid.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        pid.setSmartMotionMaxAccel(Constants.Prop.Arm.ARM_ACC_MAX, smartMotionSlot);
        pid.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
        
    }



    public void setDesiredMotorState(double angle) {
        /*final double turnOutput = turningPIDController.calculate(
                Rotation2d.fromDegrees(encoder.getPosition()).getRadians(),
                angle);
        motor1.setVoltage(turnOutput);
        */
        /*double ffVoltage = armFF.calculate(angle + rotationOffsetSupplier.get(), 0.0);
        motor1
            .getPIDController()
            .setReference(
                angle,
                ControlType.kSmartMotion,
                0, // 
                ffVoltage,
                ArbFFUnits.kVoltage
            ); 
            */
        
        if (hallEffect.get()) {
            encoder.setPosition(0);
            //armMotor.stopMotor();
            //if (angle < encoder.getPosition()) {
            //    return;
        
        }

        SparkMaxPIDController pid = armMotor.getPIDController();
        pid.setReference(angle, CANSparkMax.ControlType.kSmartMotion);
        this.desiredPosition = angle;
    }

    // public double limitAccWhenArmUp() {
    //     if (encoder.getPosition() > 30) {
    //         double maxAcc = 0.0075;
    //         return maxAcc; 
    //     }
    //     return desiredPosition;
    // }

    public void changeDesiredState(double change) {
        setDesiredMotorState(this.desiredPosition + change);
    }

    // public void reset() {
    //     encoder.setPosition(0);
    // }
    
    
    
    public void moveInside() {
        SmartDashboard.putNumber("encoder pos", encoder.getPosition());
        setDesiredMotorState(0); // TODO FIXME
    }
    
    public void moveGroundCubePickup() {
        setDesiredMotorState(0); // TODO FIXME
    }

    public void moveGroundConePickup() {
        setDesiredMotorState(0); // TODO FIXME
    }
    
    public void moveTopCone() {
        setDesiredMotorState(0); // TODO FIXME
    }
    
    public void moveBottomCone() {
        setDesiredMotorState(0); // TODO FIXME
    }
    
    public void moveTopCube() {
        setDesiredMotorState(0); // TODO FIXME
    }
    
    public void moveBottomCube() {
        setDesiredMotorState(0); // TODO FIXME
    }
    
    public void movePortal() {
        setDesiredMotorState(0); // TODO FIXME
    }

    @Override
    public void periodic() {
        if(!hallEffect.get()) {
            //System.out.println("on");
         }
         else {
           //System.out.println("off");
         }
     
     
        SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint", desiredPosition);
    }
}
