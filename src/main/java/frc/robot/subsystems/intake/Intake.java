package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOInputsAutoLogged;
import frc.robot.util.TunableFF;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final ArmIO arm;
  private final WheelIO wheel;

  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private final WheelIOInputsAutoLogged wheelInputs = new WheelIOInputsAutoLogged();

  private final TunablePID armPID = new TunablePID("Intake/Arm/PID");
  private final TunablePID wheelPID = new TunablePID("Intake/Wheel/PID");

  private final ArmFeedforward armFFConstants = new ArmFeedforward(0, 0, 0, 0);
  private final TunableFF wheelFF = new TunableFF("Intake/Wheel/FF", 0, 0, 0, 0);

  private final TunableNumber position = new TunableNumber("Intake/Position");
  private final TunableNumber velocity = new TunableNumber("Intake/Velocity");

  public Intake(ArmIO arm, WheelIO wheel) {
    super();
    this.arm = arm;
    this.wheel = wheel;

    arm.updateFeedback(armPID.pid(), armFFConstants);
    wheel.updateFeedback(wheelPID.pid(), wheelFF.ff());

    SmartDashboard.putData(
        "Intake/Stop",
        this.runOnce(
            () -> {
              arm.setOpenLoop(0);
              wheel.setVolts(0);
            }));
    SmartDashboard.putData("Intake/Rotate", this.run(() -> arm.setAngle(position.get())));
    SmartDashboard.putData("Intake/Spin", this.run(() -> wheel.setVelocity(velocity.get())));
  }

  @Override
  public void periodic() {
    if (armPID.changed()) arm.updateFeedback(armPID.pid(), armFFConstants);
    if (wheelPID.changed() || wheelFF.changed()) wheel.updateFeedback(wheelPID.pid(), wheelFF.ff());

    arm.updateInputs(armInputs);
    wheel.updateInputs(wheelInputs);

    Logger.processInputs("Intake/Arm", armInputs);
    Logger.processInputs("Intake/Wheel", wheelInputs);
  }

  public String getKey() {
    return "Intake";
  }
}
