package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shared.WheelController;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.util.TunableFF;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Indexer extends SubsystemBase {
  public WheelController controller;
  private final LoggedDashboardChooser<Command> sysIDChooser =
      new LoggedDashboardChooser<>("Indexer/SysIdRoutine");

  public Indexer(WheelIO io) {
    TunablePID indexerPID = new TunablePID("Indexer/PID", 0, 0, 0);
    TunableFF indexerFF = new TunableFF("Indexer/FF", 0, 0, 0, 0, 0);

    this.controller = new WheelController("Indexer", io, this, indexerPID, indexerFF);

    sysIDChooser.addDefaultOption("_", Commands.none());
    sysIDChooser.addOption("qs fwd", controller.sysIdQuasistatic(Direction.kForward));
    sysIDChooser.addOption("qs rev", controller.sysIdQuasistatic(Direction.kReverse));
    sysIDChooser.addOption("d fwd", controller.sysIdDynamic(Direction.kForward));
    sysIDChooser.addOption("d rev", controller.sysIdDynamic(Direction.kReverse));
  }

  @Override
  public void periodic() {
    controller.periodic();

    SmartDashboard.putData("Indexer/RunSysID", sysIDChooser.get());
  }
}
