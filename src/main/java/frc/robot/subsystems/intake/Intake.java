package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelController;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.util.TunableFF;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final ArmIO arm;
  private final WheelController wheel;

  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  private final TunablePID armPID = new TunablePID("Intake/Arm/PID", 2.05, 0, 10); // the magic d
  private final TunableFF armFF = new TunableFF("Intake/Arm/FF", 0.4, 0.0, 0,  0, 1.2);

  private final TunableNumber armVolts = new TunableNumber("Intake/Arm/Volts");
  private final TunableNumber armPosition = new TunableNumber("Intake/Arm/PositionDegrees");

  public Intake(ArmIO armIo, WheelIO wheelIO) {
    super();
    this.arm = armIo;

    // useless
    TunablePID wheelPID = new TunablePID("Intake/Wheel/PID", 0, 0, 0);
    TunableFF wheelFF = new TunableFF("Intake/Wheel/FF", 0, 0, 0, 0, 0);

    this.wheel = new WheelController("Intake/Wheel", wheelIO, this, wheelPID, wheelFF);

    arm.updateFeedback(armPID.pid(), armFF.ff());

    SmartDashboard.putData("Intake/Arm/Stop", this.runOnce(() -> arm.setVolts(0)));
    SmartDashboard.putData("Intake/Arm/Openloop", this.run(() -> arm.setVolts(armVolts.get())));
    SmartDashboard.putData(
        "Intake/Arm/Rotate", this.run(() -> arm.setAngle(armPosition.get() / 360.0)));
  }

  @Override
  public void periodic() {
    wheel.periodic();

    SmartDashboard.putNumber("Intake/MeasuredRotation", armInputs.measuredRotations * 360);

    if (armPID.changed() || armFF.changed()) arm.updateFeedback(armPID.pid(), armFF.ff());
    arm.updateInputs(armInputs);
    Logger.processInputs("Intake/Arm", armInputs);
  }

  public String getKey() {
    return "Intake";
  }
}
