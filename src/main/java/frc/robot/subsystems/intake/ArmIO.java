package frc.robot.subsystems.intake;

import frc.robot.util.TunableFF.FFConstants;
import frc.robot.util.TunablePID.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean connected = false;
    public double measuredRotations = 0.0;
    public double measuredVelocityRPM = 0.0;
    public double appliedCurrent = 0.0;
    public double appliedVoltage = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  /**
   * @depricated use setVolts
   */
  public default void setOpenLoop(double percentage) {}

  public default void setVolts(double volts) {}

  public default void setAngle(double angle) {}

  public default void updateFeedback(PIDConstants pid, FFConstants ff) {}
}
