// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableFF.FFConstants;
import frc.robot.util.TunablePID;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8; // TODO: WRONG
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = 0.6096;
  public static final double wheelBase = 0.5969;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final Rotation2d gyroAngleAdjustment = new Rotation2d(Math.PI);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

  public static final int frontLeftDriveCanId = 18;
  public static final int backLeftDriveCanId = 16;
  public static final int frontRightDriveCanId = 12;
  public static final int backRightDriveCanId = 14;

  public static final int frontLeftTurnCanId = 17;
  public static final int backLeftTurnCanId = 15;
  public static final int frontRightTurnCanId = 11;
  public static final int backRightTurnCanId = 13;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40; // TODO: IDK?
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (13.0 * 15.0); // MAXSwerve with 14 pinion teeth
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM  ->Wheel Rad/Sec

  // Drive PID configuration
  public static final TunablePID drivePID = new TunablePID("Drive/Driving", 0.0, 0.0, 0.0);
  public static final FFConstants driveFFConstants =
      new FFConstants(0.00, 0.1, 0, 0, 0); // only kS and kV used, kA for MOI
  public static final FFConstants driveAngularFFConstants =
      new FFConstants(0, 0, 0, 0, 0); // kA for MOI

  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  // public static final double turnKp = 2.0;
  // public static final double turnKd = 0.0
  public static final TunablePID turnPID = new TunablePID("Drive/Turning", 2.0, 0.0, 0.0);

  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final TunablePID autoDrivePID = new TunablePID("Drive/Auto/Driving", 5.0, 0.0, 0.0);
  public static final TunablePID autoTurnPID = new TunablePID("Drive/Auto/Turning", 5.0, 0.0, 0.0);

  public static final double robotMassKg = 30; // TODO: WRONG
  public static final double robotMOI =
      4; // robotMassKg * trackWidth/2 * driveAngularFFConstants.kA() / driveFFConstants.kA();
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
