package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOInputsAutoLogged;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

// TODO: please test 8====D

public class Shooter extends SubsystemBase {

  private final WheelIO wheel;
  private final TunablePID pid = new TunablePID("Shooter/PID", 0, 0, 0, false);

  private final WheelIOInputsAutoLogged shooterInputs = new WheelIOInputsAutoLogged();

  private final TunableNumber shooterPercentage = new TunableNumber("Shooter/Percentage", 0, true);
  private final TunableNumber shooterVelocity = new TunableNumber("Shooter/Velocity", 0, true);

  public Shooter(WheelIO shooter) {
    super();
    this.wheel = shooter;

    SmartDashboard.putData(
        "Shooter/shootOpenloop", this.run(() -> shooter.setOpenLoop(shooterPercentage.get())));
    SmartDashboard.putData("Shooter/stop", this.runOnce(() -> shooter.setOpenLoop(0)));
    SmartDashboard.putData(
        "Shooter/shootVelocity", this.run(() -> shooter.setVelocity(shooterVelocity.get())));
  }

  @Override
  public void periodic() {
    wheel.updatePID(pid);
    wheel.updateInputs(shooterInputs);

    Logger.processInputs("Shooter/Inputs", shooterInputs);
  }

  public String getKey() {
    return "Shooter";
  }
}
