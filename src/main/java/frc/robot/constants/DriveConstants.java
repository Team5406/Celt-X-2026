/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;
import frc.lib.hardware.constants.GearRatios;
import frc.lib.hardware.constants.SwerveInversions;

public class DriveConstants {
  public static final double MAX_VELOCITY = 3.5;
  public static final double MAX_ACCELERATION = 3.0;
  public static final double ODOMETRY_FREQUENCY = 100.0;

  public static final double TRACK_WIDTH = Units.Meter.convertFrom(17.5, Units.Inch);
  public static final double WHEEL_BASE = Units.Meter.convertFrom(21.5, Units.Inch);
  public static final double DRIVEBASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

  // CAN IDs
  public static final int CAN_ID_IMU = 35;

  public static final int CAN_ID_FRONT_LEFT_DRIVE =5;
  public static final int CAN_ID_FRONT_RIGHT_DRIVE = 1;
  public static final int CAN_ID_BACK_LEFT_DRIVE = 7;
  public static final int CAN_ID_BACK_RIGHT_DRIVE=3;

  public static final int CAN_ID_FRONT_LEFT_TURN = 6;
  public static final int CAN_ID_FRONT_RIGHT_TURN =2;
  public static final int CAN_ID_BACK_LEFT_TURN = 8;
  public static final int CAN_ID_BACK_RIGHT_TURN = 4;

  public static final int CAN_ID_FRONT_LEFT_CANCODER = 31;
  public static final int CAN_ID_FRONT_RIGHT_CANCODER =32;
  public static final int CAN_ID_BACK_LEFT_CANCODER = 34;
  public static final int CAN_ID_BACK_RIGHT_CANCODER = 33;



  // Motor constants
  public static final boolean FRONT_LEFT_INVERTED = true;
  public static final boolean FRONT_RIGHT_INVERTED = true;
  public static final boolean BACK_LEFT_INVERTED = true;
  public static final boolean BACK_RIGHT_INVERTED = true;
  public static final double WHEEL_DIAMETER = Units.Meter.convertFrom(3.9, Units.Inch);
  public static final int DRIVE_MOTOR_CURRENT_LIMIT = 25;
  public static final double DRIVE_MOTOR_REDUCTION =
      GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED;
  public static final double DRIVE_GEAR_RATIO = 6.75;

  public static final int TURN_MOTOR_CURRENT_LIMIT = 25;
  public static final double TURN_MOTOR_REDUCTION = 2.89 * 3.61 / 14 * 62;
  public static final double TURN_GEAR_RATIO = 89.6/7.0; //150.0/7.0;

  // Encoder constants
  public static final double DRIVE_ENCODER_POSITION_FACTOR =
      Math.PI * WHEEL_DIAMETER / DRIVE_GEAR_RATIO;
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
      (Math.PI * WHEEL_DIAMETER / DRIVE_GEAR_RATIO) / 60;

  // Turn encoder configuration
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI / TURN_GEAR_RATIO; //TURN_GEAR_RATIO; // Rotations -> Radians
  public static final double TURN_ENCODER_VELOCITY_FACTOR = TURN_ENCODER_POSITION_FACTOR / 60.0; //(TURN_GEAR_RATIO) / 60.0; // RPM -> Rad/Sec

  // Closed-loop constants
  public static final PIDGains DRIVE_PID = new PIDGains(0.017, 0, 0);
  public static final FeedForwardGains DRIVE_FF = new FeedForwardGains(0.12249, 2.73, 0);
  public static final PIDGains TURN_PID = new PIDGains(1.67, 0, 0);
  public static final FeedForwardGains TURN_FF = new FeedForwardGains(0.12249, 1.73, 0);//TODO fix this

  // Configs
  public static SparkMaxConfig DRIVE_CONFIG_BASE = new SparkMaxConfig();
  public static SparkMaxConfig DRIVE_CONFIG_FRONT_LEFT = new SparkMaxConfig();
  public static SparkMaxConfig DRIVE_CONFIG_FRONT_RIGHT = new SparkMaxConfig();
  public static SparkMaxConfig DRIVE_CONFIG_BACK_LEFT = new SparkMaxConfig();
  public static SparkMaxConfig DRIVE_CONFIG_BACK_RIGHT = new SparkMaxConfig();

  static {
    DRIVE_CONFIG_BASE
        .encoder
        .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    DRIVE_CONFIG_BASE
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DRIVE_PID.kP(), DRIVE_PID.kI(), DRIVE_PID.kD(), ClosedLoopSlot.kSlot0);//TODO change ff to calculated feed forwards
    DRIVE_CONFIG_BASE
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        //.voltageCompensation(12.0);
    DRIVE_CONFIG_BASE
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // Apply the separate motor inversion settings
    DRIVE_CONFIG_FRONT_LEFT.apply(DRIVE_CONFIG_BASE);
    DRIVE_CONFIG_FRONT_LEFT.inverted(FRONT_LEFT_INVERTED);
    DRIVE_CONFIG_FRONT_RIGHT.apply(DRIVE_CONFIG_BASE);
    DRIVE_CONFIG_FRONT_RIGHT.inverted(FRONT_RIGHT_INVERTED);
    DRIVE_CONFIG_BACK_LEFT.apply(DRIVE_CONFIG_BASE);
    DRIVE_CONFIG_BACK_LEFT.inverted(BACK_LEFT_INVERTED);
    DRIVE_CONFIG_BACK_RIGHT.apply(DRIVE_CONFIG_BASE);
    DRIVE_CONFIG_BACK_RIGHT.inverted(BACK_RIGHT_INVERTED);
  }

  public static final SparkMaxConfig TURN_CONFIG = new SparkMaxConfig();

  static {
    TURN_CONFIG
        .encoder
        .positionConversionFactor(TURN_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(TURN_ENCODER_VELOCITY_FACTOR);
    TURN_CONFIG
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        //.positionWrappingInputRange(0, TURN_ENCODER_POSITION_FACTOR)
        .positionWrappingInputRange(-Math.PI, +Math.PI)
        .pid(TURN_PID.kP(), TURN_PID.kI(), TURN_PID.kD(), ClosedLoopSlot.kSlot0);
    TURN_CONFIG
        .inverted(false)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(TURN_MOTOR_CURRENT_LIMIT);
       // .voltageCompensation(12.0);
    TURN_CONFIG
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
  }
}
