package frc.robot.util;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.measure.Torque;

// Nasko tova e nai tezhkiq boiler plate kojto sum pisal prez zhivota mi
public enum CurrentAndTorque {
  NEO(2.6 / 105.0), // Empirical stall Torque and Current values. 2.6 Nm and 105 A
  NEO550(0.97 / 100.0), // Stall Torque and Current vals. 0.97 Nm and 100A
  ;

  final double val;
  public static final TorqueUnit units = TorqueUnit.combine(Meter, Newton);

  CurrentAndTorque(double val) {
    this.val = val;
  }

  /**
   * Dont want to allow people to touch the value so I added this helper
   *
   * @return the torque constnat for this particular motor (N*m/A)
   */
  public double getConstant() {
    return val;
  }

  /**
   * Gives you the torque constant(N*m/A) with a gear reduction
   *
   * @param motor The motor for which the torque is being calculated
   * @param reduction The reduction 1/6, 1/8, 0.5, etc. Exceed 1 for inverse reduction
   * @return returns a torque conversion constant measured in N*m/A
   */
  public static double getConstantWithReduction(CurrentAndTorque motor, double reduction) {
    return motor.getConstant() * reduction;
  }

  /**
   * returns the torque of a motor based on a particular torqueConstant.
   *
   * @param Amps The Amperes the motor is pushing
   * @param torqueConstant The constant of this motor.
   * @return returns a torque conversion constant measured in N*m/A
   */
  public static Torque getTorque(double Amps, double torqueConstant) {
    return Torque.ofBaseUnits(Amps * torqueConstant, units);
  }

  /**
   * returns the torque of a motor based on a motor and a reduction.
   *
   * @param Amps The Amperes the motor is pushing
   * @param motor The motor for which the torque is being calculated
   * @param reduction The reduction 1/6, 1/8, 0.5, etc. Exceed 1 for inverse reduction(not likely)
   * @return returns a torque conversion constant measured in N*m/A
   */
  public static Torque getTorque(double Amps, CurrentAndTorque motor, double reduction) {
    return Torque.ofBaseUnits(getConstantWithReduction(motor, reduction), units);
  }

  /**
   * returns the torque of a motor based on a motor and no reduction.
   *
   * @param Amps The Amperes the motor is pushing
   * @param motor The motor for which the torque is being calculated
   * @return returns a torque conversion constant measured in N*m/A
   */
  public static Torque getTorque(double Amps, CurrentAndTorque motor) {
    return getTorque(Amps, motor, 1);
  }

  /**
   * Lets you get the amps required to achieve a particular torque.
   *
   * @param targetTorque the targe torque.
   * @param motor the motor for which this calculation is ran
   * @param reduction The reduction 1/6, 1/8, 0.5, etc. Exceed 1 for inverse reduction(not likely)
   * @return returns the target Amperes with which to power the motor.
   */
  public static double getAmps(Torque targetTorque, CurrentAndTorque motor, double reduction) {
    return targetTorque.in(units) / getConstantWithReduction(motor, reduction);
  }

  /**
   * Lets you get the amps required to achieve a particular torque.
   *
   * @param targetTorque the targe torque IN NEWTON METERS
   * @param motor the motor for which this calculation is ran
   * @param reduction The reduction 1/6, 1/8, 0.5, etc. Exceed 1 for inverse reduction(not likely)
   * @return returns the target Amperes with which to power the motor.
   */
  public static double getAmps(double targetTorque, CurrentAndTorque motor, double reduction) {
    return targetTorque / getConstantWithReduction(motor, reduction);
  }
}
