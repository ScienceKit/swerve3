package frc.robot.subsystems.shared;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.TunablePID;

import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

public class WheelIOSpark implements WheelIO {
  private final SparkMax motor;
  private final TunablePID pid;

  private final Debouncer connectDebounce =
    new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public WheelIOSpark(int id, String key) {
    motor = new SparkMax(id, MotorType.kBrushless);
    pid = new TunablePID(key);

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .voltageCompensation(12.0);
    motorConfig.encoder
        .velocityConversionFactor(1/60.0);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(pid.p(), pid.i(), pid.d());

    tryUntilOk(motor, 5, () -> 
        motor.configure(motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters));    
  }

  @Override
  public void updateInputs(WheelIOInputs inputs) {
    sparkStickyFault = false;
    inputs.connected = connectDebounce.calculate(!sparkStickyFault);
  }
}
