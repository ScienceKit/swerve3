package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelController;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final ArmIO arm;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  private final TunableNumber armVolts = new TunableNumber("Intake/Arm/Volts");
  private final TunableNumber armPosition = new TunableNumber("Intake/Arm/Degrees");

  private final WheelController wheelController;

  public Intake(ArmIO armIo, WheelIO wheelIO) {
    super();
    this.arm = armIo;
    arm.updateFeedback(IntakeConstants.Arm.pid.pid(), IntakeConstants.Arm.ff.ff());

    this.wheelController = new WheelController("Intake/Wheel", wheelIO);

    SmartDashboard.putData("Intake/Arm/Stop", this.runOnce(() -> arm.setVolts(0)));
    SmartDashboard.putData("Intake/Arm/Openloop", this.run(() -> arm.setVolts(armVolts.get())));
    SmartDashboard.putData(
        "Intake/Arm/Rotate",
        this.run(() -> arm.setAngle(armPosition.get() / 360.0)).finallyDo(() -> arm.setVolts(0)));
    SmartDashboard.putData("Intake/Panic", panic());

    // TODO: lol
    setDefaultCommand(run(() -> arm.setAngle(IntakeConstants.Arm.retractedPosition)));
  }

  @Override
  public void periodic() {
    wheelController.periodic();

    if (IntakeConstants.Arm.pid.changed() || IntakeConstants.Arm.ff.changed())
      arm.updateFeedback(IntakeConstants.Arm.pid.pid(), IntakeConstants.Arm.ff.ff());
    arm.updateInputs(armInputs);
    Logger.processInputs("Intake/Arm", armInputs);

    SmartDashboard.putNumber("Intake/MeasuredDegrees", armInputs.measuredRotations * 360);
  }

  public static String key() {
    return "Intake";
  }

  // TODO: Write commands
  // ...

  public Command intake() {
    return this.startEnd(
        () -> {
          arm.setAngle(IntakeConstants.Arm.extendedPosition);
          wheelController.getIO().setVolts(IntakeConstants.Wheel.intakePower * 12);
        },
        () -> {
          arm.setAngle(IntakeConstants.Arm.retractedPosition);
          wheelController.getIO().setVolts(0);
        });
  }

  public Command panic() {
    return this.runOnce(
            () -> {
              arm.setAngle(10 / 360.0);
              wheelController.getIO().setVolts(-0.2 * 12);
            })
        .andThen(Commands.waitSeconds(0.3))
        .andThen(() -> arm.setAngle(40 / 360.0))
        .andThen(Commands.waitSeconds(0.3))
        .repeatedly()
        .finallyDo(
            () -> {
              arm.setAngle(IntakeConstants.Arm.retractedPosition);
              wheelController.getIO().setVolts(0);
            });
  }
}
