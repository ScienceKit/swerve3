package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shared.WheelController;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.util.Util;

public class Indexer extends SubsystemBase {
  private final WheelController controller;

  private int counter = 0;

  public Indexer(WheelIO io) {
    this.controller = new WheelController("Indexer", io);

    SmartDashboard.putData("Indexer/Sinusoidal", sinusoidal());
  }

  @Override
  public void periodic() {
    controller.periodic();
    counter++;
  }

  public Command sinusoidal() {
    return runEnd(
        () -> controller.getIO().setVolts(Util.remap(Math.sin(counter / 3), -1, 1, 1, 6)),
        () -> controller.getIO().setVolts(0));
  }
}
