package frc.robot.subsystems.shared;

import frc.robot.util.TunableFF.FFConstants;
import frc.robot.util.TunablePID.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public interface WheelIO {
  @AutoLog
  public static class WheelIOInputs {
    public boolean connected = false;
    public double measuredRotations = 0.0;
    public double measuredVelocityRPM = 0.0;
    public double appliedCurrent = 0.0;
    public double appliedVoltage = 0.0;
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
   * @deprecated Use setVolts instead.
   */
  public default void setOpenLoop(double percentage) {}

  /**
   * Set the setpoint for the velocity pid controller.
   *
   * @param velocityRPM the setpoint in rpm.
   */
  public default void setVelocity(double velocityRPM) {}

  /**
   * Run the motor controller with volts volts
   *
   * @param volts the volts in volts for setVolts
   */
  public default void setVolts(double volts) {}

  /**
   * Update the pid settings (if needed)ww
   *
   * @param pid
   */
  public default void updateFeedback(PIDConstants pid, FFConstants ff) {}
}
