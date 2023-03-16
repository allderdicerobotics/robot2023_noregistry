package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.misc.*;

public class Claw extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(Constants.Port.TowerArmClaw.CAN_ID_CLAW, MotorType.kBrushless);
    SparkMaxPIDController pid = motor.getPIDController();
    boolean atLimit = false;
    DigitalInput limit = new DigitalInput(0);
    RelativeEncoder encoder;
    boolean isOpen = true;
    boolean isCalibrated = false;

    public static final Map<ClawPosition, Double> MOTOR_STATES = Map.of(
        ClawPosition.OPEN, 0.0,
        ClawPosition.CLOSED_CUBE, -6.9,
        ClawPosition.CLOSED_CONE, -9.9
    );

    public Claw() {
        encoder = motor.getEncoder();
        initMotor();
    }

    @Override
    public void periodic() {
        atLimit = !limit.get();
        if (atLimit) {
            encoder.setPosition(0);
            if (!isOpen) {
                System.out.println("Claw.periodic - atLimit && !isOpen - forcing zero position");
                forceZeroPosition();
            }
        }
    }

    // Tested these PID F et. al. values with the example code
    // It worked great there
    // The example is at:
    // Downloads\SPARK-MAX-Examples-master\SPARK-MAX-Examples-master\Java\Smart Motion Example

    public void initMotor() {
        /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    SparkMaxPIDController m_pidController = motor.getPIDController();

    // Tested these PID F et. al. values with the example code
    // It worked great there
    // The example is at:
    // Downloads\SPARK-MAX-Examples-master\SPARK-MAX-Examples-master\Java\Smart Motion Example

    // PID coefficients
    double kP = 0.0000003; 
    double kI = 0.00000001;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000156; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    // Smart Motion Coefficients
    int maxVel = 10000; // rpm
    int maxAcc = 12000;
    int minVel = 10;
    
    double allowedErr = 0.25;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    }

    // Represents a possible configuration of the claw
    private enum ClawPosition {
        OPEN,
        CLOSED_CUBE,
        CLOSED_CONE,
    }

    private void goToPosition(ClawPosition pos) {
        setDesiredMotorState(MOTOR_STATES.get(pos));
    }

    public void open() {
        System.out.println("CLAW OPENING");
        if (!isCalibrated) {
            System.out.println("CLAW OPENING as far as it can (!isCalibrated, finding zero)");
            setDesiredMotorState(15.0);
            isCalibrated = true;
        } else if (atLimit) {
            System.out.println("CLAW OPENED (atLimit)");
            return;
        }
        goToPosition(ClawPosition.OPEN);
    }

    public void closeCube() {
        System.out.println("CLAW CLOSING ON CUBE");
        goToPosition(ClawPosition.CLOSED_CUBE);
    }

    public void closeCone() {
        System.out.println("CLAW CLOSING ON CONE");
        goToPosition(ClawPosition.CLOSED_CONE);
    }

    public void setDesiredMotorState(double state){
        if (state == 0.0) {
            isOpen = true;
        }
        pid.setReference(state, CANSparkMax.ControlType.kSmartMotion);
    }

    public void forceZeroPosition() {
        setDesiredMotorState(0.0);
    }
}
