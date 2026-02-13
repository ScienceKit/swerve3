package frc.robot.subsystems.Shooter;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.time.format.ResolverStyle;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

public class TurnIOSpark implements TurnIO {
    private final SparkMax motor;

    private final Debouncer connectDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public TurnIOSpark(int id) {
        motor = new SparkMax(id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        
        //----------------------------------------------
        var motorConfig = new SparkMaxConfig();
        motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .voltageCompensation(12.0);
        motorConfig.closedLoop.maxMotion
            .cruiseVelocity(10)
            .maxAcceleration(10);
        //----------------------------------------------
        tryUntilOk(
            motor, 
            5,
            () -> {
                return motor.configure(
                    motorConfig, 
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                );
            }
        );
    }

    @Override
    public void updateInputs(TurnIOInputs inputs)
    {
        sparkStickyFault = false;

        ifOk(
            motor, 
            motor.getEncoder()::getPosition, 
            (value) -> inputs.measuredRotations = value
        );

        ifOk(
            motor, 
            motor.getEncoder()::getVelocity, 
            (value) -> inputs.measuredVelocityRPM = value
        );

        ifOk(
            motor, 
            motor::getOutputCurrent, 
            (value) -> inputs.appliedCurrent = value
        );

        ifOk(
            motor, 
            new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
            (value) -> inputs.appliedVoltage = value[0] * value[1]
        );

        inputs.connected = connectDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setAngle(double angleRad) {
        motor.getClosedLoopController().setSetpoint(angleRad, ControlType.kMAXMotionPositionControl);
    }



}
