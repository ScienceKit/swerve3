package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final SparkMax motor;
  private final TunableNumber percentage = new TunableNumber("Climber/percentage", 0.0);
  //   private final SysIdRoutine sysId;

  public Climber(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);

    var motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SmartDashboard.putData(
        "Climber/Openloop",
        runEnd(() -> motor.setVoltage(percentage.get() * 12), () -> motor.setVoltage(0)));
    SmartDashboard.putData("Climber/stop", runOnce(() -> motor.setVoltage(0)));

    motor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Position", motor.getEncoder().getPosition());
    Logger.recordOutput("Climber/Velocit", motor.getEncoder().getVelocity());
    Logger.recordOutput("Climber/Current", motor.getOutputCurrent());
  }
}
