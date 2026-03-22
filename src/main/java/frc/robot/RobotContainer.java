// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.ArmIO;
import frc.robot.subsystems.intake.ArmIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shared.WheelIO;
import frc.robot.subsystems.shared.WheelIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Vision vision;

  // Controller
  private final CommandGenericHID controller = new CommandGenericHID(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        shooter = new Shooter(new WheelIOSpark(51), new WheelIOSpark(52, 1.2), drive::getPose);
        // shooter = new Shooter(new WheelIO() {}, new WheelIO() {});

        // indexer = new Indexer(new WheelIOSpark(61));
        indexer = new Indexer(new WheelIO() {});

        intake = new Intake(new ArmIOSpark(41), new WheelIOSpark(42));
        // intake = new Intake(new ArmIO() {}, new WheelIO() {});

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        // new VisionIOPhotonVision(
        // VisionConstants.camera1Name, VisionConstants.robotToCamera1);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        shooter = new Shooter(new WheelIO() {}, new WheelIO() {}, drive::getPose);
        indexer = new Indexer(new WheelIO() {});
        intake = new Intake(new ArmIO() {}, new WheelIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shooter = new Shooter(new WheelIO() {}, new WheelIO() {}, drive::getPose);
        indexer = new Indexer(new WheelIO() {});
        intake = new Intake(new ArmIO() {}, new WheelIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.throttleDrive(
            drive,
            () -> controller.getRawAxis(3),
            () -> controller.getRawAxis(2),
            () -> -controller.getRawAxis(0),
            () -> (controller.getRawAxis(1) + 1) / 2));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            new Runnable() {
              @Override
              public void run() {
                SmartDashboard.putNumber(
                    "blah/Hub distance",
                    shooter.getFieldTranslation().getDistance(FieldConstants.Red.hub));
              }
            });

    controller
        .axisGreaterThan(5, 0.9)
        .whileTrue(
            DriveCommands.throttleDriveAtAngle(
                    drive,
                    () -> controller.getRawAxis(3),
                    () -> controller.getRawAxis(2),
                    () -> shooter.yawToTarget(FieldConstants.Red.hub),
                    () -> (controller.getRawAxis(1) + 1) / 2)
                .alongWith(shooter.autoShoot(FieldConstants.Red.hub)));

    controller
        .axisGreaterThan(4, 0.9)
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                DriveConstants.gyroAngleAdjustment))) // i hope
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    try {
      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile("Example Path"), new PathConstraints(1.5, 1.5, 0.5, 0.5));
    } catch (Exception e) {
      System.out.println("Tashaci");
      return Commands.none();
    }
  }
}
