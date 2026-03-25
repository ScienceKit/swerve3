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
  private final Supplier<Translation2d> velocitySupplier;

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

  // Distance based shooting while moving
  private final InterpolatingDoubleTreeMap distToTime =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.6, 0.7154990438),
          Map.entry(2.75, 0.7567778347),
          Map.entry(2.9, 0.7980566257),
          Map.entry(3.08, 0.8475911749),
          Map.entry(3.2, 0.8806142077),
          Map.entry(3.35, 0.9218929987),
          Map.entry(3.5, 0.9631717897),
          Map.entry(3.65, 1.004450581),
          Map.entry(3.8, 1.045729372),
          Map.entry(3.95, 1.087008163),
          Map.entry(4.1, 1.128286954),
          Map.entry(4.25, 1.169565745),
          Map.entry(4.4, 1.210844536),
          Map.entry(4.55, 1.252123327),
          Map.entry(4.7, 1.293402118),
          Map.entry(4.85, 1.334680909),
          Map.entry(5.0, 1.3759597),
          Map.entry(5.165, 1.42136637),
          Map.entry(5.29, 1.43), // 1.3759597), sus
          Map.entry(5.458, 1.455765362),
          Map.entry(5.6, 1.501997608),
          Map.entry(5.75, 1.541074863));

  // Velocity based shooting while moving
  // private final InterpolatingDoubleTreeMap distToSpeed = new IntepolatingDoubleTreeMap();
  // private final InterpolatingDoubleTreeMap speedToRPM = new InterpolatingDoubleTreeMap();

  private final Translation2d robotToShooter = new Translation2d(-0.143, -0.209);

  private final LoggedDashboardChooser<Command> sysIDChooser =
      new LoggedDashboardChooser<>("Shooter/SysIdRoutine");

  public Shooter(WheelIO wheel, WheelIO feeder) {
    this(wheel, feeder, () -> new Pose2d(), () -> new Translation2d());
  }

  public Shooter(WheelIO wheel, WheelIO feeder, Supplier<Pose2d> poseSupplier) {
    this(wheel, feeder, poseSupplier, () -> new Translation2d());
  }

  public Shooter(
      WheelIO wheel,
      WheelIO feeder,
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> velocitySupplier) {
    super();

    TunablePID wheelPID = new TunablePID("Shooter/Wheel/PID", 0.00015, 0, 0);
    TunablePID feederPID = new TunablePID("Shooter/Feeder/PID", 0.00007499, 0, 0);

    TunableFF wheelFF = new TunableFF("Shooter/Wheel/FF", 0.10376, 0.0021162, 0.00052972, 0, 0);
    TunableFF feederFF = new TunableFF("Shooter/Feeder/FF", 0.45974, 0.0021951, 0.00020702, 0, 0);

    this.wheelController = new WheelController("Shooter/Wheel", wheel, this, wheelPID, wheelFF);
    this.feederController =
        new WheelController("Shooter/Feeder", feeder, this, feederPID, feederFF);
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;

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
    return wheelController
        .sysIdQuasistatic(direction)
        .andThen(feederController.sysIdQuasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return wheelController
        .sysIdDynamic(direction)
        .andThen(feederController.sysIdDynamic(direction));
  }

  // Auto shoot no moving
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

  // Distance based autoShootMoving
  // TODO: more advanced algoithm would take into account the angular velocity of the robot
  public Translation2d compensatedFieldTranslation(Translation2d target) {
    return getFieldTranslation()
        .plus(
            velocitySupplier
                .get()
                // .plus(...) from the angular velocity
                .times(distToTime.get(getFieldTranslation().getDistance(target))));
  }

  public Rotation2d compensatedYawToTarget(Translation2d target) {
    return target
        .minus(compensatedFieldTranslation(target))
        .getAngle()
        .plus(new Rotation2d(-Math.PI / 2));
  }

  public Command autoShootMoving(Translation2d target) {
    return runEnd(
        () -> {
          wheelController
              .getIO()
              .setVelocity(
                  distToRPMMap.get(compensatedFieldTranslation(target).getDistance(target)));
          feederController.getIO().setVelocity(2500);
        },
        () -> {
          wheelController.getIO().setVolts(0);
          feederController.getIO().setVolts(0);
        });
  }

  // Velocity based auto shoot
  // public Translation2d compensatedVelocity(Translation2d target) {
  //   return (new Translation2d(
  //           distToSpeed.get(getFieldTranslation().getDistance((target))), yawToTarget(target)))
  //       .minus(velocitySupplier.get());
  // }

  // public Rotation2d compensatedYawToTarget(Translation2d target) {
  //   return target.minus(compensatedVelocity(target)).getAngle().plus(new Rotation2d(-Math.PI /
  // 2));
  // }

  // public Command autoShoot(Translation2d target) {
  //   return runEnd(
  //       () -> {
  //         wheelController
  //             .getIO()
  //             .setVelocity(speedToRPM.get(compensatedVelocity(target).getNorm()));
  //       },
  //       () -> {});
  // }
}
