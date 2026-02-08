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
  private final WheelIO feeder;

  private final WheelIOInputsAutoLogged wheelInputs = new WheelIOInputsAutoLogged();
  private final WheelIOInputsAutoLogged feederInputs = new WheelIOInputsAutoLogged();

  private final Alert alertWheel = new Alert("Disconnected main shooter motor", AlertType.kError);
  private final Alert alertFeeder = new Alert("Disconnectd feeder shooter motor", AlertType.kError);

  private final TunablePID wheelPID = new TunablePID("Shooter/Wheel/PID", 0.0003, 0, 0);
  private final TunablePID feederPID = new TunablePID("Shooter/Feeder/PID");

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

    // TODO: these are just copy pasted, don't want that
    wheel.updateFeedback(wheelPID.pid(), 0, 0.0021);
    feeder.updateFeedback(feederPID.pid(), 0, 0);

    SmartDashboard.putData("Shooter/Wheel/Stop", this.runOnce(() -> wheel.setOpenLoop(0)));
    SmartDashboard.putData(
        "Shooter/Wheel/Openloop", this.run(() -> wheel.setOpenLoop(wheelPercentage.get())));
    SmartDashboard.putData(
        "Shooter/Wheel/Spin", this.run(() -> wheel.setVelocity(wheelVelocity.get())));

    SmartDashboard.putData("Shooter/Feeder/Stop", this.runOnce(() -> feeder.setOpenLoop(0)));
    SmartDashboard.putData(
        "Shooter/Feeder/Openloop", this.run(() -> feeder.setOpenLoop(feederPercentage.get())));
    SmartDashboard.putData(
        "Shooter/Feeder/Spin", this.run(() -> feeder.setVelocity(feederVelocity.get())));
  }

  @Override
  public void periodic() {
    if (wheelPID.changed())
      wheel.updateFeedback(wheelPID.pid(), 0, 0.0021);
    if (feederPID.changed())
      feeder.updateFeedback(feederPID.pid(), 0, 0);

    wheel.updateInputs(wheelInputs);
    feeder.updateInputs(feederInputs);

    Logger.processInputs("Shooter/Wheel/Inputs", wheelInputs);
    Logger.processInputs("Shooter/Feeder/Inputs", feederInputs);

    alertWheel.set(!wheelInputs.connected);
    alertFeeder.set(!feederInputs.connected);
  }

  public String getKey() {
    return "Shooter";
  }
}
