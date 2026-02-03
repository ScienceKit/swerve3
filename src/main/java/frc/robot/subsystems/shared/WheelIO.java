package frc.robot.subsystems.shared;

import org.littletonrobotics.junction.AutoLog;

public interface WheelIO {
  @AutoLog
  public static class WheelIOInputs {
    public boolean connected = false;
    public double measuredVelocityRPS = 0.0;
    public double appliedCurrent = 0.0;
    public double appliedVoltage = 0.0;
    public double appliedTorque = 0.0;
  }

  /**
   * Call this repeatedly in the loop to keep the autolog happy
   *
   * @param inputs pass an instance of this class here that will be changed.
   */
  public default void updateInputs(WheelIOInputs inputs) {}

  /**
   * Call this once to set the motor power
   *
   * @param percentage the percentage of the motor power.
   */
  public default void setOpenLoop(double percentage) {}

  /**
   * Set the setpoint for the velocity pid controller.
   *
   * @param velocityRPS the setpoint in rps.
   */
  public default void setVelocity(double velocityRPS) {}
}
