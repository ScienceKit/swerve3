package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.TunableFF.FFConstants;
import frc.robot.util.TunablePID.PIDConstants;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {
  private final SparkMax motor;

  private final Debouncer connectDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ArmIOSpark(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);

    var motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    motorConfig.inverted(true);
    // motorConfig
    //     .closedLoop
    //     .maxMotion
    //     .cruiseVelocity(10 * (60 * Math.PI / 180))
    //     .maxAcceleration(10 * (60 * Math.PI / 180));
    motorConfig
        .absoluteEncoder
        .zeroCentered(true)
        .positionConversionFactor(ArmConstants.conversionFactor)
        .velocityConversionFactor(ArmConstants.conversionFactor)
        .averageDepth(2);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        motor,
        motor.getAbsoluteEncoder()::getPosition,
        (value) -> inputs.measuredRotations = value);
    ifOk(
        motor,
        motor.getAbsoluteEncoder()::getVelocity,
        (value) -> inputs.measuredVelocityRPM = value);
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
        .kCos(ff.kCos())
        .kCosRatio(1);

    // config
    //     .softLimit
    //     .forwardSoftLimitEnabled(true)
    //     .forwardSoftLimit(58 / 360.0)
    //     .reverseSoftLimitEnabled(true)
    //     .reverseSoftLimit(0);

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
  public void setAngle(double rotation) {
    motor.getClosedLoopController().setSetpoint(rotation, ControlType.kPosition);
  }

  public void setVolts(double volts) {
    motor.setVoltage(volts);
  }
}
