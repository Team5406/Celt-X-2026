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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double DEADBAND = 0.05;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    public static double maxSpeed = DriveConstants.MAX_TELEOP_SPEED;
    public static double teleopAcceleration = DriveConstants.MAX_TELEOP_ACCELERATION;
    public static double maxAngularVelocity = DriveConstants.ANGLE_MAX_VELOCITY;
    public static double angularLimiter = DriveConstants.ANGLE_NO_VELOCITY_LIMIT;

    private static Translation2d lastLinearVelocity = new Translation2d();
    private static double lastTime = 0;

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    if (drive.getShooting() && drive.currentZone == Drive.Zone.CELTX_ALLIANCE) {
                        maxSpeed = DriveConstants.DRIVE_SHOOT_TELEOP_SPEED;
                        angularLimiter = DriveConstants.ANGLE_SHOOT_VELOCITY_LIMIT;
                        teleopAcceleration =
                                DriveConstants.MAX_TELEOP_ACCELERATION * (maxSpeed / DriveConstants.MAX_TELEOP_SPEED);
                    } else {
                        maxSpeed = DriveConstants.MAX_TELEOP_SPEED;
                        angularLimiter = DriveConstants.ANGLE_NO_VELOCITY_LIMIT;
                        teleopAcceleration = DriveConstants.MAX_TELEOP_ACCELERATION;
                    }

                    Logger.recordOutput("Acceleration/LinearVelocity X:", linearVelocity.getX());
                    Logger.recordOutput("Acceleration/LinearVelocity Y:", linearVelocity.getY());

                    double dTime = Timer.getFPGATimestamp() - lastTime;
                    double dVx = (linearVelocity.getX() - lastLinearVelocity.getX());
                    double dVy = (linearVelocity.getY() - lastLinearVelocity.getY());
                    double aX = dVx / dTime;
                    double aY = dVy / dTime;

                    Logger.recordOutput("Acceleration/dTime:", dTime);
                    Logger.recordOutput("Acceleration/dVx:", dVx);
                    Logger.recordOutput("Acceleration/dVy:", dVy);
                    Logger.recordOutput("Acceleration/dAx:", aX);
                    Logger.recordOutput("Acceleration/dAy:", aY);

                    double largestA = Math.hypot(Math.abs(aX), Math.abs(aY));

                    Logger.recordOutput("Acceleration/teleopAcceleration", teleopAcceleration);
                    Logger.recordOutput("Acceleration/Max Speed", maxSpeed);
                    Logger.recordOutput("Acceleration/isShooting", drive.shooting);

                    if (largestA
                            > (teleopAcceleration
                                    / DriveConstants.MAX_TELEOP_SPEED)) { // drive.getMaxLinearSpeedMetersPerSec
                        aX = aX * (teleopAcceleration / DriveConstants.MAX_TELEOP_SPEED) / largestA;
                        aY = aY * (teleopAcceleration / DriveConstants.MAX_TELEOP_SPEED) / largestA;
                    }

                    Translation2d adjustedLinearVelocity = new Translation2d(
                            lastLinearVelocity.getX() + (aX * dTime), lastLinearVelocity.getY() + (aY * dTime));

                    double largestV = adjustedLinearVelocity.getNorm();

                    double constrainedVelocity =
                            maxSpeed * Math.sqrt(teleopAcceleration / DriveConstants.MAX_TELEOP_ACCELERATION);
                    Logger.recordOutput("Acceleration/constrainedVelocity", constrainedVelocity);

                    double vX = adjustedLinearVelocity.getX();
                    double vY = adjustedLinearVelocity.getY();

                    Logger.recordOutput("Acceleration/largestV:", largestV);
                    Logger.recordOutput("Acceleration/new vX:", vX);
                    Logger.recordOutput("Acceleration/new vY:", vY);

                    if (largestV > (constrainedVelocity / maxSpeed)) { // drive.getMaxLinearSpeedMetersPerSec
                        vX = adjustedLinearVelocity.getX() * (constrainedVelocity / maxSpeed) / largestV;
                        vY = adjustedLinearVelocity.getY() * (constrainedVelocity / maxSpeed) / largestV;
                    }

                    Translation2d finalVelocity = new Translation2d(vX, vY);

                    Logger.recordOutput("Acceleration/adjustedvX:", vX);
                    Logger.recordOutput("Acceleration/adjustedvY:", vY);

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            finalVelocity.getX() * maxSpeed,
                            finalVelocity.getY() * maxSpeed,
                            omega * drive.getMaxAngularSpeedRadPerSec() * angularLimiter);
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
                    lastLinearVelocity = finalVelocity;
                    lastTime = Timer.getFPGATimestamp();
                },
                drive);
    }

    public static void setLastVelocity(Translation2d lastVelocity) {
        lastLinearVelocity = lastVelocity;
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            BooleanSupplier shooting) {

        if (shooting.getAsBoolean() == true) {
            maxAngularVelocity = DriveConstants.ANGLE_SHOOT_VELOCITY;
        } else {
            maxAngularVelocity = DriveConstants.ANGLE_MAX_VELOCITY;
        }

        Logger.recordOutput("Acceleration/maxAngularVelocity", maxAngularVelocity);

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(maxAngularVelocity, DriveConstants.ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * DriveConstants.MAX_TELEOP_SPEED,
                                    linearVelocity.getY() * DriveConstants.MAX_TELEOP_SPEED,
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius =
                                            (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
