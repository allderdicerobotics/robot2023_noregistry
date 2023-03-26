package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.misc.*;

public class WheelClaw extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(Constants.Port.TowerArmClaw.CAN_ID_CLAW, MotorType.kBrushless);

    //private final RelativeEncoder encoder = motor.getEncoder();
    //private final SparkMaxPIDController pid = motor.getPIDController();

    public WheelClaw() {
        initMotor();
    }

    public void initMotor() {
        // reset motor
        motor.restoreFactoryDefaults();
        // PID coefs
        /*double kP = 0.0005; 
        double kI = 0;
        double kD = 0.005; 
        double kIz = 0; 
        double kFF = 0.3; 
        double kMaxOutput = 0.7; 
        double kMinOutput = -0.7;
    
        // set PID coefficients
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMinOutput, kMaxOutput);
        */
    }

    public void spinIn() {
        // System.out.println("spinIn");
        setVelocity(5);
    }

    public void spinOut() {
        setVelocity(-.5);
    }

    public void spinStop() {
        // System.out.println("spinStop");
        setVelocity(0.0);
    }

    public void spinInSlowly() {
        // System.out.println("spinStop");
        setVelocity(0.1);
    }

    void setVelocity(double setpoint) {
        motor.set(setpoint);
    }


}
