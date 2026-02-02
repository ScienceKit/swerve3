package frc.robot.subsystems.shared;

import org.littletonrobotics.junction.AutoLog;

public interface WheelIO {
  @AutoLog
  public static class WheelIOInputs {
    public boolean connected = false;
    public double measuredVelocityRPS = 0.0;
    public double appliedCurrent = 0.0;
    public double appliedVoltage = 0.0;
  }

  public default void updateInputs(WheelIOInputs inputs) {}

  public default void setOpenLoop(double percentage) {}

  public default void setVelocity(double velocityRPS) {}
}
