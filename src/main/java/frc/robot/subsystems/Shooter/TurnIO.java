package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.util.TunablePID.PIDConstants;
import org.littletonrobotics.junction.AutoLog;

public interface TurnIO {
  @AutoLog
  public static class TurnIOInputs {
    public boolean connected = false;
    public double measuredRotations = 0.0;
    public double measuredVelocityRPM = 0.0;
    public double appliedCurrent = 0.0;
    public double appliedVoltage = 0.0;
  }

  public default void updateInputs(TurnIOInputs inputs) {}

  public default void setAngle(double angleRad) {}

  public default void updateFeedback(PIDConstants pid, ArmFeedforward ff) {}
}