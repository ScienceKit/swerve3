package frc.robot.subsystems.intake;

import frc.robot.util.TunableFF;
import frc.robot.util.TunablePID;

public class IntakeConstants {
  public class Arm {
    public static final double reachAngleDeadline = 1;

    public static final double conversionFactor = 0.5;

    public static final double extendedLimit = -1 / 360.0;
    public static final double retractedLimit = 58 / 360.0;

    public static final double extendedPosition = 2 / 360.0;
    public static final double retractedPosition = 30 / 360.0;
    // TODO: add more positions
    // ...

    public static final TunablePID pid =
        new TunablePID(Intake.key() + "/Arm/PID", 2.55, 0, 0); // the magic d
    public static final TunableFF ff =
        new TunableFF(Intake.key() + "/Arm/FF", 0.5, 0.0, 0, 0, 0.75); // kS maybe high???
  }

  public class Wheel {
    public static final double intakePower = -0.85;
  }
}
