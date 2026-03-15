package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.ArmIO;
import frc.robot.subsystems.intake.ArmIOInputsAutoLogged;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final ArmIO arm;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private final TunablePID armPID = new TunablePID("Turret/Arm/PID");
  private final ArmFeedforward armFFConstants = new ArmFeedforward(0, 0, 0, 0);
  private final TunableNumber position = new TunableNumber("Turret/Position");

  public Turret(ArmIO arm) {
    super();
    this.arm = arm;

    arm.updateFeedback(armPID.pid(), armFFConstants);
    SmartDashboard.putData(
        "Intake/Stop",
        this.runOnce(
            () -> {
              arm.setOpenLoop(0);
            }));
    SmartDashboard.putData("Turret/Rotate", this.run(() -> arm.setAngle(position.get())));
  }

  @Override
  public void periodic() {
    if (armPID.changed()) arm.updateFeedback(armPID.pid(), armFFConstants);

    arm.updateInputs(armInputs);

    Logger.processInputs("Turret/Arm", armInputs);
  }

  public String getKey() {
    return "Turret";
  }

  public void setArmAngle(double angle) {
    arm.setAngle(angle);
  }
}
