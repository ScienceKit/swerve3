package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOInputsAutoLogged;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final WheelIO wheel;
  private final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();

  private final Alert alert = new Alert("Disconnected " + getKey() + " motor", AlertType.kError);

  private final TunablePID pid = new TunablePID("Shooter/PID", 0.0003, 0, 0);
  private final TunableNumber percentage = new TunableNumber("Shooter/Percentage", 0, true);
  private final TunableNumber velocity = new TunableNumber("Shooter/VelocityRPM", 0, true);

  public Shooter(WheelIO shooter) {
    super();
    this.wheel = shooter;
    wheel.updatePID(pid);

    SmartDashboard.putData("Shooter/Stop", this.runOnce(() -> shooter.setOpenLoop(0)));
    SmartDashboard.putData(
        "Shooter/Openloop", this.run(() -> shooter.setOpenLoop(percentage.get())));
    SmartDashboard.putData("Shooter/Spin", this.run(() -> shooter.setVelocity(velocity.get())));
  }

  @Override
  public void periodic() {
    if (pid.changed()) wheel.updatePID(pid);

    wheel.updateInputs(inputs);
    Logger.processInputs("Shooter/Inputs", inputs);

    alert.set(!inputs.connected);
  }

  public String getKey() {
    return "Shooter";
  }
}
