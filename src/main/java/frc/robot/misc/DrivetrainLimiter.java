package frc.robot.misc;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class DrivetrainLimiter {
  private final double m_accelRateLimit;
  private final double m_decelerationLimit;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
   * value.
   *
   * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
   *     second. This is expected to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
   *     second. This is expected to be negative.

   */
  public DrivetrainLimiter(double positiveRateLimit, double negativeRateLimit) {
    m_accelRateLimit = positiveRateLimit;
    m_decelerationLimit = negativeRateLimit;
    m_prevVal = 0;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }
  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    Boolean inputPositive = input >= 0 ? true : false;
    Boolean prevPositive = m_prevVal >= 0 ? true : false;

    if (Math.abs(input) <= 0.05) {
        //System.out.println("zero case");
        if (prevPositive){
            m_prevVal +=
                MathUtil.clamp(
                    input - m_prevVal,
                    -m_decelerationLimit * elapsedTime,
                    m_accelRateLimit * elapsedTime);
        } else {
            m_prevVal +=
                MathUtil.clamp(
                    input - m_prevVal,
                    -m_accelRateLimit * elapsedTime,
                    m_decelerationLimit * elapsedTime);  
        }         
    } else {
        m_prevVal +=
                MathUtil.clamp(
                    input - m_prevVal,
                    -m_accelRateLimit * elapsedTime,
                    m_accelRateLimit * elapsedTime);  
        
    }
    
    m_prevTime = currentTime;

    //double outputValue = MathUtil.clamp(m_prevVal, -1.0, 1.0);

    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }
}

