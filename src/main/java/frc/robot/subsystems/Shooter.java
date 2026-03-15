package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shared.WheelController;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.util.TunableFF;
import frc.robot.util.TunablePID;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Shooter extends SubsystemBase {

  private final WheelController wheelController;
  private final WheelController feederController;

  private final LoggedDashboardChooser<Command> sysIDChooser =
      new LoggedDashboardChooser<>("Shooter/SysIdRoutine");

  public Shooter(WheelIO wheel, WheelIO feeder) {
    super();

    TunablePID wheelPID = new TunablePID("Shooter/Wheel/PID", 0.000003, 0, 0);
    TunablePID feederPID = new TunablePID("Shooter/Feeder/PID", 0.000008, 0, 0);

    TunableFF wheelFF = new TunableFF("Shooter/Wheel/FF", 0.13, 0.002117, 0, 0);
    TunableFF feederFF = new TunableFF("Shooter/Feeder/FF", 0.38, 0.0022662, 0, 0);

    this.wheelController = new WheelController("Shooter/Wheel", wheel, this, wheelPID, wheelFF);
    this.feederController =
        new WheelController("Shooter/Feeder", feeder, this, feederPID, feederFF);

    // Combined run/stop dashboard controls
    SmartDashboard.putData(
        "Shooter/Run",
        wheelController.spin().alongWith(feederController.spin()));

    SmartDashboard.putData(
        "Shooter/Stop",
        wheelController.stop().alongWith(feederController.stop()));

    sysIDChooser.addDefaultOption("_", Commands.none());
    sysIDChooser.addOption("Wheel qs f", sysIdQuasistatic(Direction.kForward));
    sysIDChooser.addOption("Wheel qs r", sysIdQuasistatic(Direction.kReverse));
    sysIDChooser.addOption("Wheel d f", sysIdDynamic(Direction.kForward));
    sysIDChooser.addOption("Wheel d r", sysIdDynamic(Direction.kReverse));
  }

  @Override
  public void periodic() {
    wheelController.periodic();
    feederController.periodic();

    SmartDashboard.putData("Shooter/RunSysID", sysIDChooser.get());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    Command wheelCommand = wheelController.sysIdQuasistatic(direction);
    Command feederCommand = feederController.sysIdQuasistatic(direction);
    return wheelCommand.andThen(feederCommand);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    Command wheelCommand = wheelController.sysIdDynamic(direction);
    Command feederCommand = feederController.sysIdDynamic(direction);
    return wheelCommand.andThen(feederCommand);
  }
}
