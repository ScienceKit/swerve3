package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIO.WheelIOInputs;
import frc.robot.subsystems.shared.WheelIOSpark;
import frc.robot.util.TunableNumber;

public class Spindexer extends SubsystemBase {

  public WheelIO motor;
  WheelIOInputs inputs = new WheelIOInputs();
  TunableNumber speed = new TunableNumber("Spindexer/Percentage", 0, true);

  public Spindexer(int id) {
    super();
    motor = new WheelIOSpark(id, "Spindexer/Wheel");
  }

  @Override
  public void periodic() {

    if (speed.changed()) {
      speed.update();
      motor.updateInputs(inputs);
      motor.setOpenLoop(speed.get());
    }
  }
}
