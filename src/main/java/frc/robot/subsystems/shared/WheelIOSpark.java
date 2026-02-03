package frc.robot.subsystems.shared;

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
import frc.robot.util.CurrentAndTorque;
import frc.robot.util.TunablePID;

public class WheelIOSpark implements WheelIO {
  private final SparkMax motor;
  private final TunablePID pid;

  private final Debouncer connectDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public WheelIOSpark(int id, String key) {
    motor = new SparkMax(id, MotorType.kBrushless);
    pid = new TunablePID(key);

    var motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12.0);
    motorConfig.encoder.velocityConversionFactor(1 / 60.0);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(pid.p(), pid.i(), pid.d());

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
    inputs.connected = connectDebounce.calculate(!sparkStickyFault);
    inputs.appliedCurrent = motor.getOutputCurrent();
    inputs.appliedVoltage = motor.getBusVoltage();
    inputs.measuredVelocityRPS = motor.getEncoder().getVelocity();
    inputs.appliedTorque =
        CurrentAndTorque.getTorque(inputs.appliedCurrent, CurrentAndTorque.NEO, 1.0).magnitude();
  }

  @Override
  public void setOpenLoop(double percentage) {
    motor.set(percentage);
  }

  @Override
  public void setVelocity(double velocityRPS) {
    motor.getClosedLoopController().setSetpoint(velocityRPS, ControlType.kVelocity);
  }
}
