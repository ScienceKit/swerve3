package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Shooter extends SubsystemBase {

  private final WheelController wheelController;
  private final WheelController feederController;

  private final Supplier<Pose2d> poseSupplier;

  private final InterpolatingDoubleTreeMap distToRPMMap =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.6, 2900.0),
          Map.entry(2.75, 3000.0),
          Map.entry(2.9, 3080.0),
          Map.entry(3.08, 3130.0),
          Map.entry(3.2, 3170.0),
          Map.entry(3.35, 3250.0),
          Map.entry(3.5, 3330.0),
          Map.entry(3.65, 3360.0),
          Map.entry(3.8, 3390.0),
          Map.entry(3.95, 3430.0),
          Map.entry(4.1, 3460.0),
          Map.entry(4.25, 3490.0),
          Map.entry(4.4, 3520.0),
          Map.entry(4.55, 3555.0),
          Map.entry(4.7, 3590.0),
          Map.entry(4.85, 3700.0),
          Map.entry(5.0, 3790.0),
          Map.entry(5.165, 3870.0),
          Map.entry(5.29, 3900.0),
          Map.entry(5.458, 3960.0),
          Map.entry(5.6, 4000.0),
          Map.entry(5.75, 4070.0));

  private final Translation2d robotToShooter = new Translation2d(-0.143, -0.209);

  private final LoggedDashboardChooser<Command> sysIDChooser =
      new LoggedDashboardChooser<>("Shooter/SysIdRoutine");

  public Shooter(WheelIO wheel, WheelIO feeder) {
    this(wheel, feeder, () -> new Pose2d());
  }

  public Shooter(WheelIO wheel, WheelIO feeder, Supplier<Pose2d> poseSupplier) {
    super();

    TunablePID wheelPID = new TunablePID("Shooter/Wheel/PID", 0.00015, 0, 0);
    TunablePID feederPID = new TunablePID("Shooter/Feeder/PID", 0.00007499, 0, 0);

    TunableFF wheelFF = new TunableFF("Shooter/Wheel/FF", 0.10376, 0.0021162, 0.00052972, 0, 0);
    TunableFF feederFF = new TunableFF("Shooter/Feeder/FF", 0.45974, 0.0021951, 0.00020702, 0, 0);

    this.wheelController = new WheelController("Shooter/Wheel", wheel, this, wheelPID, wheelFF);
    this.feederController =
        new WheelController("Shooter/Feeder", feeder, this, feederPID, feederFF);
    this.poseSupplier = poseSupplier;

    SmartDashboard.putData(
        "Shooter/Stop", idle().raceWith(wheelController.stop().andThen(feederController.stop())));
    SmartDashboard.putData(
        "Shooter/Spin", idle().raceWith(wheelController.spin().raceWith(feederController.spin())));

    sysIDChooser.addDefaultOption("_", Commands.none());
    sysIDChooser.addOption("qs fwd", sysIdQuasistatic(Direction.kForward));
    sysIDChooser.addOption("qs rev", sysIdQuasistatic(Direction.kReverse));
    sysIDChooser.addOption("d fwd", sysIdDynamic(Direction.kForward));
    sysIDChooser.addOption("d rev", sysIdDynamic(Direction.kReverse));
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

  public Command autoShoot(Translation2d target) {
    return runEnd(
        () -> {
          wheelController
              .getIO()
              .setVelocity(distToRPMMap.get(getFieldTranslation().getDistance(target)));
          feederController.getIO().setVelocity(2500);
        },
        () -> {
          wheelController.getIO().setVolts(0);
          feederController.getIO().setVolts(0);
        });
  }

  public Translation2d getFieldTranslation() {
    Pose2d robot = poseSupplier.get();
    return robot.plus(new Transform2d(robotToShooter, robot.getRotation())).getTranslation();
  }

  public Rotation2d yawToTarget(Translation2d target) {
    return target.minus(getFieldTranslation()).getAngle().plus(new Rotation2d(-Math.PI / 2));
  }
}
