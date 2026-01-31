package frc.robot.subsystems.Shooter;

import static frc.robot.util.SparkUtil.tryUntilOk;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Spindexer extends SubsystemBase {

    private final SparkMax motor = new SparkMax(30, MotorType.kBrushless);

    @AutoLogOutput(key = "Spindexer/Velocity")
    private double speed = 0.0;

    public Spindexer() {
        super();
        SmartDashboard.putNumber("Spindexer/Percentage", 0);

        SparkMaxConfig config = new SparkMaxConfig();
        config.openLoopRampRate(0.4);
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12.0);
        config.encoder.positionConversionFactor(Math.PI * 2).velocityConversionFactor(Math.PI * 60);
        tryUntilOk(
            motor,
            5,
            () -> motor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );
    }

    @Override
    public void periodic() {
        speed = motor.getEncoder().getVelocity();
        motor.set(SmartDashboard.getNumber("Spindexer/Percentage", 0));
    }
}
