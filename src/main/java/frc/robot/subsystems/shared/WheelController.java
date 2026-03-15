package frc.robot.subsystems.shared;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.TunableFF;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class WheelController {
  private final String key;
  private final WheelIO io;
  private final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();
  private final Alert alert;

  private final TunablePID pid;
  private final TunableFF ff;

  private final TunableNumber percentage;
  private final TunableNumber velocity;

  private final SysIdRoutine sysId;
  private final SubsystemBase parent;

  public WheelController(
      String key, WheelIO io, SubsystemBase parent, TunablePID pid, TunableFF ff) {
    this.key = key;
    this.io = io;
    this.parent = parent;
    this.pid = pid;
    this.ff = ff;

    this.alert = new Alert("Disconnected " + key + " motor", AlertType.kError);

    // Tunable numbers for dashboard control
    this.percentage = new TunableNumber(key + "/Percentage", 0, true);
    this.velocity = new TunableNumber(key + "/VelocityRPM", 0, true);

    // Apply initial feedback settings
    io.updateFeedback(pid.pid(), ff.ff());

    // Register dashboard commands for common operations
    SmartDashboard.putData(key + "/Stop", stop());
    SmartDashboard.putData(key + "/Openloop", openloop());
    SmartDashboard.putData(key + "/Spin", spin());

    // SysId routine for this wheel
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput(key + "/SysID", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setVolts(voltage.in(Volts)), null, parent));
  }

  public void periodic() {
    if (pid.changed() || ff.changed()) io.updateFeedback(pid.pid(), ff.ff());

    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    alert.set(!inputs.connected);
  }

  // TODO: commads should be parent
  public Command stop() {
    return Commands.runOnce(() -> io.setVolts(0));
  }

  public Command openloop() {
    return Commands.run(() -> io.setOpenLoop(percentage.get()));
  }

  public Command spin() {
    return Commands.run(() -> io.setVelocity(velocity.get()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return parent
        .runOnce(() -> io.setVolts(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return parent
        .runOnce(() -> io.setVolts(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  public WheelIO getIO() {
    return io;
  }
}
