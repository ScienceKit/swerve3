package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
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

  public default void setOpenLoop(double percentage) {}

  public default void setAngle(double angleRad) {}

  public default void updateFeedback(PIDConstants pid, ArmFeedforward ff) {}
}
