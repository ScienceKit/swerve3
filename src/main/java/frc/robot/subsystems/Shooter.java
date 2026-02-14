package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOInputsAutoLogged;
import frc.robot.util.TunableFF;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final WheelIO wheel;
  private final WheelIO feeder;

  private final WheelIOInputsAutoLogged wheelInputs = new WheelIOInputsAutoLogged();
  private final WheelIOInputsAutoLogged feederInputs = new WheelIOInputsAutoLogged();

  private final Alert alertWheel = new Alert("Disconnected main shooter motor", AlertType.kError);
  private final Alert alertFeeder = new Alert("Disconnectd feeder shooter motor", AlertType.kError);

  private final TunablePID wheelPID = new TunablePID("Shooter/Wheel/PID", 0.0003, 0, 0);
  private final TunablePID feederPID = new TunablePID("Shooter/Feeder/PID", 0.0001, 0, 0);

  private final TunableFF wheelFF = new TunableFF("Shooter/Wheel/FF", 0, 0.0022, 0, 0);
  private final TunableFF feederFF = new TunableFF("Shooter/Feeder/FF", 0.4, 0.0022, 0, 0);

  private final TunableNumber wheelPercentage =
      new TunableNumber("Shooter/Wheel/Percentage", 0, true);
  private final TunableNumber feederPercentage =
      new TunableNumber("Shooter/Feeder/Percentage", 0, true);

  private final TunableNumber wheelVelocity =
      new TunableNumber("Shooter/Wheel/VelocityRPM", 0, true);
  private final TunableNumber feederVelocity =
      new TunableNumber("Shooter/Feeder/VelocityRPM", 0, true);

  public Shooter(WheelIO wheel, WheelIO feeder) {
    super();

    this.wheel = wheel;
    this.feeder = feeder;

    wheel.updateFeedback(wheelPID.pid(), wheelFF.ff());
    feeder.updateFeedback(feederPID.pid(), feederFF.ff());

    SmartDashboard.putData("Shooter/Wheel/Stop", this.runOnce(() -> wheel.setOpenLoop(0)));
    SmartDashboard.putData(
        "Shooter/Wheel/Openloop", this.run(() -> wheel.setOpenLoop(wheelPercentage.get())));
    SmartDashboard.putData(
        "Shooter/Wheel/Spin", this.run(() -> wheel.setVelocity(wheelVelocity.get())));

    SmartDashboard.putData("Shooter/Feeder/Stop", this.runOnce(() -> feeder.setOpenLoop(0)));
    SmartDashboard.putData(
        "Shooter/Feeder/Openloop", this.run(() -> feeder.setOpenLoop(feederPercentage.get())));
    SmartDashboard.putData(
        "Shooter/Feeder/Spin", this.runOnce(() -> feeder.setVelocity(feederVelocity.get())));

    SmartDashboard.putData(
        "Shooter/Run",
        this.run(
            () -> {
              wheel.setVelocity(wheelVelocity.get());
              feeder.setVelocity(feederVelocity.get());
            }));

    SmartDashboard.putData(
        "Shooter/Stop",
        this.run(
            () -> {
              wheel.setOpenLoop(0);
              feeder.setOpenLoop(0);
            }));
  }

  @Override
  public void periodic() {
    if (wheelPID.changed() || wheelFF.changed()) wheel.updateFeedback(wheelPID.pid(), wheelFF.ff());
    if (feederPID.changed() || feederFF.changed())
      feeder.updateFeedback(feederPID.pid(), feederFF.ff());

    wheel.updateInputs(wheelInputs);
    feeder.updateInputs(feederInputs);

    Logger.processInputs("Shooter/Wheel", wheelInputs);
    Logger.processInputs("Shooter/Feeder", feederInputs);

    alertWheel.set(!wheelInputs.connected);
    alertFeeder.set(!feederInputs.connected);
  }

  public String getKey() {
    return "Shooter";
  }
}
