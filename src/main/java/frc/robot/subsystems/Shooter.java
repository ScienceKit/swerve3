package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOInputsAutoLogged;
import frc.robot.util.TunableFF;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  private final SysIdRoutine wheelSysId;
  private final SysIdRoutine feederSysId;
  private final LoggedDashboardChooser<Command> sysIDChooser = new LoggedDashboardChooser<>("Shooter/SysIdRoutine");

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

    wheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/Wheel/SysID", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> wheel.setVolts(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
    feederSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/Feeder/SysID", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> feeder.setVolts(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

    sysIDChooser.addDefaultOption("_", Commands.none());
    sysIDChooser.addOption("Wheel qs f", sysIdQuasistatic(Direction.kForward));
    sysIDChooser.addOption("Wheel qs r", sysIdQuasistatic(Direction.kReverse));
    sysIDChooser.addOption("Wheel d f", sysIdDynamic(Direction.kForward));
    sysIDChooser.addOption("Wheel d r", sysIdDynamic(Direction.kReverse));
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

    SmartDashboard.putData("Shooter/SYSID", sysIDChooser.get());
  }

  public String getKey() {
    return "Shooter";
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    Command wheelCommand =
        run(() -> wheel.setOpenLoop(0.0))
            .withTimeout(1.0)
            .andThen(wheelSysId.quasistatic(direction));
    Command feederCommand =
        run(() -> feeder.setOpenLoop(0.0))
            .withTimeout(1.0)
            .andThen(feederSysId.quasistatic(direction));

    return wheelCommand.andThen(feederCommand);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    Command wheelCommand =
        run(() -> wheel.setOpenLoop(0.0)).withTimeout(1.0).andThen(wheelSysId.dynamic(direction));
    Command feederCommand =
        run(() -> wheel.setOpenLoop(0.0)).withTimeout(1.0).andThen(feederSysId.dynamic(direction));
    return wheelCommand.andThen(feederCommand);
  }
}
