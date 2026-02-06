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
import frc.robot.util.TunablePID;
import java.util.function.DoubleSupplier;

public class WheelIOSpark implements WheelIO {
  private final SparkMax motor;

  private final Debouncer connectDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public WheelIOSpark(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);

    var motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12.0);
    motorConfig.encoder.velocityConversionFactor(1); // 1/60.0
    motorConfig.closedLoop.feedForward.kS(0).kV(0.0021);
    // uncomment these????
    // motorConfig
    //     .signals
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);

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

    ifOk(motor, motor.getEncoder()::getVelocity, (value) -> inputs.measuredVelocityRPM = value);
    ifOk(motor, motor::getOutputCurrent, (v) -> inputs.appliedCurrent = v);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);

    inputs.connected = connectDebounce.calculate(!sparkStickyFault);

    // inputs.appliedTorque =
    //     CurrentAndTorque.getTorque(inputs.appliedCurrent, CurrentAndTorque.NEO, 1.0).magnitude();
  }

  public void updatePID(TunablePID pid) {
    if (!pid.changed()) return;

    var config = new SparkMaxConfig();
    config.closedLoop.p(pid.p()).d(pid.d()).i(pid.i());

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
}
