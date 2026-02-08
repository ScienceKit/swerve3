package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOInputsAutoLogged;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final ArmIO arm;
  private final WheelIO wheel;

  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private final WheelIOInputsAutoLogged wheelInputs = new WheelIOInputsAutoLogged();

  private final ArmFeedforward ff = new ArmFeedforward(0, 0, 0, 0);

  private final TunablePID armPID = new TunablePID("Intake/Arm/PID");
  private final TunablePID wheelPID = new TunablePID("Arm/Wheel/PID");

  private final TunableNumber position = new TunableNumber("Intake/Position");
  private final TunableNumber velocity = new TunableNumber("Intake/Velocity");

  public Intake(ArmIO arm, WheelIO wheel) {
    super();
    this.arm = arm;
    this.wheel = wheel;

    // TODO: these are just copy patesd, so fix
    wheel.updateFeedback(wheelPID.pid(), 0, 0);
    arm.updateFeedback(armPID.pid(), ff);

    SmartDashboard.putData(
        "Intake/Stop",
        this.runOnce(
            () -> {
              arm.setOpenLoop(0);
              wheel.setOpenLoop(0);
            }));
    SmartDashboard.putData("Shooter/Rotate", this.run(() -> arm.setAngle(position.get())));
    SmartDashboard.putData("Shooter/Spin", this.run(() -> wheel.setVelocity(velocity.get())));
  }

  @Override
  public void periodic() {
    if (armPID.changed()) arm.updateFeedback(armPID.pid(), ff);
    if (wheelPID.changed()) wheel.updateFeedback(wheelPID.pid(), 0, 0);

    arm.updateInputs(armInputs);
    wheel.updateInputs(wheelInputs);

    Logger.processInputs("Intake/Arm/Inputs", armInputs);
    Logger.processInputs("Intake/Wheel/Input", wheelInputs);
  }

  public String getKey() {
    return "Intake";
  }
}
