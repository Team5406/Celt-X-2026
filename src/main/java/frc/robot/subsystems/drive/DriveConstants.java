// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.5; // reducing this breaks on the fly pathing!!
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(21.5);
    public static final double wheelBase = Units.inchesToMeters(21.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final double DRIVE_TURN_MODIFIER = 0.5;
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int pigeonCanId = 30;

    public static final int frontLeftDriveCanId = 5;

    public static final int backLeftDriveCanId = 7;
    public static final int frontRightDriveCanId = 3;
    public static final int backRightDriveCanId = 1;

    public static final int frontLeftTurnCanId = 4;
    public static final int backLeftTurnCanId = 8;
    public static final int frontRightTurnCanId = 2;
    public static final int backRightTurnCanId = 6;

    public static final int frontLeftCancoderId = 40;
    public static final int backLeftCancoderId = 43;
    public static final int frontRightCancoderId = 41;
    public static final int backRightCancoderId = 42;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 70;
    public static final int teleopDriveCurrentLimit = 70;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.928);
    public static final double driveMotorReduction = 6.75; // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    /***********
     * Wheel Radius Characterization Results **********
     * Wheel Delta: 90.652 radians
     * Gyro Delta: 12.643 radians
     * Wheel Radius: 0.049 meters, 1.923 inches
     */

    /***********
     * Drive FF Characterization Results **********
     * kS: 0.17472
     * kV: 0.10852
     */

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->

    // Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.005;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.17472;
    public static final double driveKv = 0.10852;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 60;
    public static final double turnMotorReduction = 150.0 / 7.0;
    public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 6.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
    public static final double AUTO_SLOW_ENOUGH = 1.0;
    public static final double AUTO_CLOSE_ENOUGH = 1.0;

    // PathPlanner configuration
    public static final double robotMassKg = 67;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig = new RobotConfig(
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
    public static final double MAX_TELEOP_ACCELERATION = 9.5;
    public static final double MAX_TELEOP_SPEED = 4.0;
}
