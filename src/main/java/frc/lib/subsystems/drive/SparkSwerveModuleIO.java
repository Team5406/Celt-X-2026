/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.subsystems.drive;

import static frc.lib.hardware.rev.SparkUtil.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.FeedForwardGains;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Swerve Module IO implementation for REV Spark motor controllers. Feedback sensors are assumed to
 * be connected to the Spark motor controllers. Absolute encoder offsets should be configured in the
 * REV Hardware Client.
 */
public class SparkSwerveModuleIO implements SwerveModuleIO {

  // Hardware objects
  private final SparkBase driveMotor;
  private final SparkBase turnMotor;
  private final CANcoder turnCancoder;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final FeedForwardGains driveFFGains;
  private final FeedForwardGains turnFFGains;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  /** Encoder synchronization queued. */
  private boolean synchronizeEncoderQueued = false;
  /** Encoder, Absolute encoder synchronization enabled. */
  private boolean synchronizeEncoderEnabled = false;
  /** Encoder synchronization deadband in degrees. */
  private double synchronizeEncoderDeadband = 0;//TODO find a good number

  
  private double resetIteration = 0;
  private static final int ENCODER_RESET_ITERATIONS = 500;
  private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);


  /**
   * Creates a new {@code SparkSwerveModule}.
   *
   * @param driveMotor The Spark motor controller for the drive motor.
   * @param turnMotor The Spark motor controller for the turn motor.
   * @param driveConfig The configuration for the drive motor controller.
   * @param turnConfig The configuration for the turn motor controller.
   * @param driveFFGains The feedforward gains for the drive motor controller.
   * @param turnFFGains The feedforward gains for the turn motor controller.
   */
  public SparkSwerveModuleIO(
      SparkBase driveMotor,
      SparkBase turnMotor,
      CANcoder turnCancoder,
      SparkBaseConfig driveConfig,
      SparkBaseConfig turnConfig,
      FeedForwardGains driveFFGains,
      FeedForwardGains turnFFGains) {

    this.turnCancoder = turnCancoder;
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;

    this.driveFFGains = driveFFGains;
    this.turnFFGains = turnFFGains;

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    turnCancoder.getAbsolutePosition().setUpdateFrequency(100);
    turnCancoder.optimizeBusUtilization();

    CANcoderConfiguration config = new CANcoderConfiguration();

    CANcoderConfigurator canCoderConfigurator = turnCancoder.getConfigurator();
    CANcoderConfiguration oldConfig = new CANcoderConfiguration();
    canCoderConfigurator.refresh(oldConfig);
    config.MagnetSensor.MagnetOffset = oldConfig.MagnetSensor.MagnetOffset;
    // config.MagnetSensor.MagnetOffset = canCoderOffsetDegrees;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; //AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnCancoder.getConfigurator().apply(config);

    // Configure the drive and turn motors
    tryUntilOk(
        driveMotor,
        5,
        () ->
            driveMotor.configure(
                driveConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

    tryUntilOk(
        turnMotor,
        5,
        () ->
            turnMotor.configure(
                turnConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnMotor, turnEncoder::getPosition);
  }

  /** {@inheritDoc} */
  public void updateInputs(SwerveModuleInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePosition = value);
    ifOk(driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocity = value);
    ifOk(
        driveMotor,
        new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
        (values) -> inputs.driveAppliedVoltage = values[0] * values[1]);
    ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrent = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnMotor,
        turnEncoder::getPosition,
        (value) -> inputs.turnRelativeRotation = new Rotation2d(value));
    ifOk(turnMotor, turnEncoder::getVelocity, (value) -> inputs.turnAngularVelocity = value);
    ifOk(
        turnMotor,
        new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
        (values) -> inputs.turnAppliedVoltage = values[0] * values[1]);
    ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrent = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
    inputs.turnAbsoluteRotation = getAbsoluteAngle();
    inputs.CANcoderAbsolutePosition = new Rotation2d(turnCancoder.getAbsolutePosition().getValue().in(Units.Radians));

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }


  public Rotation2d getAbsoluteAngle()
    {
        double angle = turnEncoder.getPosition();

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
             if (turnEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;

                    // The cancoder outputs 0 to 1
                    double absAngle = turnCancoder.getAbsolutePosition().getValue().in(Units.Radians);
                    /*absAngle %= 2.0 * Math.PI;
                    if (absAngle < 0.0) {
                        absAngle += 2.0 * Math.PI;
                    }*/
                    turnEncoder.setPosition(absAngle);
                    angle = absAngle;
                }
            } else {
                resetIteration = 0;
            }
        return new Rotation2d(angle);
    }


  /** {@inheritDoc} */
  @Override
  public void setDriveState(double velocity, double acceleration) {
    double ff =
        driveFFGains.kS() * Math.signum(velocity)
            + driveFFGains.kV() * velocity
            + driveFFGains.kA() * acceleration;
    driveController.setSetpoint(
        velocity,
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ff,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  /** {@inheritDoc} */
  @Override
  public void setTurnState(double rotation, double angularVelocity) {
    double direction = Math.abs(angularVelocity) > 1e-5 ? Math.signum(angularVelocity) : 0;
    double ff = turnFFGains.kS() * direction + turnFFGains.kV() * angularVelocity;


    double adjustedReferenceAngleRadians = rotation;

    double currentAngleRadians = getAbsoluteAngle().getRadians();

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
        currentAngleRadiansMod += 2.0 * Math.PI;
      }

    // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
    adjustedReferenceAngleRadians = rotation + currentAngleRadians - currentAngleRadiansMod;
    if (rotation - currentAngleRadiansMod > Math.PI) {
        adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (rotation - currentAngleRadiansMod < -Math.PI) {
        adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    turnController.setSetpoint(
      adjustedReferenceAngleRadians, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);

  }

}
