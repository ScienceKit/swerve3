package frc.robot.subsystems.shared;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.TunableFF.FFConstants;
import frc.robot.util.TunablePID.PIDConstants;
import java.util.function.DoubleSupplier;

public class WheelIOSpark implements WheelIO {
  private final SparkMax motor;

  private final Debouncer connectDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public WheelIOSpark(int id) {
    this(id, 0);
  }

  public WheelIOSpark(int id, double rampRateSeconds) {
    motor = new SparkMax(id, MotorType.kBrushless);

    var motorConfig = new SparkMaxConfig();
    motorConfig.closedLoopRampRate(rampRateSeconds);
    motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(45).voltageCompensation(12.0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(WheelIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(motor, motor.getEncoder()::getVelocity, (v) -> inputs.measuredVelocityRPM = v);
    ifOk(motor, motor.getEncoder()::getPosition, (v) -> inputs.measuredRotations = v);
    ifOk(motor, motor::getOutputCurrent, (v) -> inputs.appliedCurrent = v);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    inputs.connected = connectDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void updateFeedback(PIDConstants pid, FFConstants ff) {
    var config = new SparkMaxConfig();
    config
        .closedLoop
        .p(pid.p())
        .i(pid.i())
        .d(pid.d())
        .feedForward
        .kS(ff.kS())
        .kV(ff.kV())
        .kA(ff.kA())
        .kG(ff.kG());

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setOpenLoop(double percentage) {
    motor.set(percentage);
  }

  @Override
  public void setVelocity(double velocityRPM) {
    motor.getClosedLoopController().setSetpoint(velocityRPM, ControlType.kVelocity);
  }

  @Override
  public void setVolts(double volts) {
    motor.setVoltage(volts);
  }
}
