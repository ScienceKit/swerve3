package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIO.WheelIOInputs;
import frc.robot.subsystems.shared.WheelIOSpark;
import frc.robot.util.TunableNumber;

// TODO: please test 8====D

public class Shooter extends SubsystemBase {

  public WheelIO shooter;
  public WheelIO kicker;

  WheelIOInputs shooterInputs = new WheelIOInputs();
  WheelIOInputs kickerInputs = new WheelIOInputs();

  TunableNumber shooterSpeed = new TunableNumber("Shooter/ShooterPercentage", 0, true);
  TunableNumber kickerSpeed = new TunableNumber("Shooter/KickerPercentage", 0, true);

  public Shooter(int id) {
    super();
    shooter = new WheelIOSpark(id, "Shooter/ShooterWheel");
    kicker = new WheelIOSpark(id, "Shooter/KickerWheel");
  }

  @Override
  public void periodic() {
    shooter.updateInputs(shooterInputs);

    if (shooterSpeed.changed()) {
      shooterSpeed.update();
      shooter.setOpenLoop(shooterSpeed.get());
    }

    kicker.updateInputs(kickerInputs);

    if (kickerSpeed.changed()) {
      kickerSpeed.update();
      kicker.setOpenLoop(kickerSpeed.get());
    }
  }
}
