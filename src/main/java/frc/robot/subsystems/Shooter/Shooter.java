package frc.robot.subsystems.Shooter;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

// TODO: please test 8====D

public class Shooter extends SubsystemBase {

  private final SparkMax motor = new SparkMax(30, MotorType.kBrushless);

  @AutoLogOutput(key = "Shooter/Velocity")
  private double speed = 0.0;

  public Shooter() {
    super();
    SmartDashboard.putNumber("Shooter/Percentage", 0);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12.0);
    config.encoder.positionConversionFactor(Math.PI * 2).velocityConversionFactor(Math.PI * 60);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void periodic() {
    speed = motor.getEncoder().getVelocity();
    motor.set(SmartDashboard.getNumber("Shooter/Percentage", 0));
  }
}
