package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;

public class LEDConstants {
  public static final int kLEDCount = 300;
  public static final Distance kLEDPitch = Millimeters.of(16.1);
  public static final Distance kStripLength = kLEDPitch.times(kLEDCount);
  public static final int kLEDPWMPort = 9;
  public static LEDPattern pLEDsDisabled = LEDPattern.solid(Color.kBlack);

  /**
   * Creates a nice patern for indicating stuff that orbits around the robot origin (assumes led
   * strip wraps around the robot)
   *
   * @param color1 the first color in the list
   * @param color2 the color opposite to it in the orbit
   * @param speed the speed in {@code radians / second} of the effect. Change this to make cool
   *     stuff happen
   * @return
   */
  public static LEDPattern createDualOrbit(Color color1, Color color2, DoubleSupplier speed) {
    return new LEDPattern() {
      double integratedPosition = 0.0;
      double oldTime = Timer.getFPGATimestamp();

      @Override
      public void applyTo(LEDReader reader, LEDWriter writer) {
        // FRC considers strobing lights above 5 flickers per second. The effect and rule get
        // applied per LED so in total this is max speed.
        double velocity = MathUtil.clamp(speed.getAsDouble(), 0, 5.0 * 2.0 * Math.PI);
        double seconds = Timer.getFPGATimestamp() - oldTime;
        oldTime += seconds;
        integratedPosition += velocity * seconds;

        int bufLen = reader.getLength();
        for (int led = 0; led < bufLen; led++) {
          double progress = (double) led / (double) bufLen;
          double color1magnitude = sin2CurveModulo(integratedPosition, progress * 2.0 * Math.PI);
          double color2magnitude =
              sin2CurveModulo(Math.PI + integratedPosition, progress * 2.0 * Math.PI);
          Color outCol =
              new Color(
                  color1magnitude * color1.red + color2magnitude * color2.red,
                  color1magnitude * color1.green + color2magnitude * color2.green,
                  color1magnitude * color1.blue + color2magnitude * color2.blue);
          writer.setLED(led, outCol);
        }
      }
      ;
    };
  }

  public static Color getTeamColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return Color.kYellow;
    if (alliance.get() == Alliance.Red) return Color.kRed;
    return Color.kBlue;
  }

  /**
   * Internally used to make bell curves for visual effects
   *
   * @param distance distance from the first LED in the list.
   * @return
   */
  static double sin2CurveModulo(double maxPoint, double angle) {
    double clampedAngle = angle % (2.0 * Math.PI);
    return Math.pow(Math.cos((clampedAngle - maxPoint) / 2), 15);
  }
}
