/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */

package frc.robot.subsystems.intake.pivot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.control.PIDGains;

public class IntakePivotConstants {
    public static int PIVOT_MOTOR_ID = 11; // Left
    public static boolean PIVOT_MOTOR_FOLLOWER_INVERSION = true;
    public static final double PIVOT_ACTIVE_POSITION = 15;
    public static final double PIVOT_STOW_POSITION = 135;
    public static final double PIVOT_ZEROED_UP_POSITION = 140;
    public static final double PIVOT_ZEROED_DOWN_POSITION = 5;

    public static final double PIVOT_SHOOTING_POSITION = 70; // degrees
    public static final double PIVOT_SHOOTING_POSITION_SECOND = 15; // degrees
    public static final double PIVOT_BOUNCE_TIMEOUT = 0.3; // seconds
    public static final double TANGLED_HEIGHT = 90; // degrees

    // public static final double PIVOT_G = 0.01;
    public static final double PIVOT_GEAR_RATIO = 5 * 4 * 3 * 16 / 12; // 2:1 gear ratio
    public static final double PIVOT_CONVERSION_FACTOR = 360 / PIVOT_GEAR_RATIO; // In degrees
    public static final double PIVOT_MAX_VELOCITY = 360;
    public static final double PIVOT_MAX_ACCELERATION = 1500;
    public static final double PIVOT_CURRENT_SPIKE = 30;
    public static final double PIVOT_POSITION_TOLERANCE = 180.0;
    public static final double PIVOT_POSITION_OFFSET = 0.0;
    public static final double PIVOT_ZEROING_VOLTAGE = 4;
    public static final ClosedLoopSlot PIVOT_PID_POSITION_SLOT = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot PIVOT_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;

    public static final double PIVOT_POSITION_KP = 0.019;
    public static final double PIVOT_POSITION_KI = 0.0;
    public static final double PIVOT_POSITION_KD = 0.0; // 3.0259

    public static final double PIVOT_VELOCITY_KP = 0.0;
    public static final double PIVOT_VELOCITY_KI = 0.0;
    public static final double PIVOT_VELOCITY_KD = 0.0;

    public static final double PIVOT_MIN_OUTPUT = -1;

    public static final double PIVOT_MAX_OUTPUT = 1;
    private static final double ODOMETRY_FREQUENCY = 50;
    public static final PIDGains PIVOT_PROFILE_PID = new PIDGains(.01, 0.0, 0.0);

    public static final double STALL_TIME = 0.1;
    public static final double DEBOUNCE_TIME = 0.2;

    public static int PIVOT_CURRENT_LIMIT_STALL = 40; // amps
    public static int PIVOT_CURRENT_LIMIT_FREE = 40; // amps

    public static double PIVOT_FORWARD_SOFT_LIMIT = 0; // degrees
    public static double PIVOT_REVERSE_SOFT_LIMIT = 0; // degrees
    public static final ArmFeedforward feedForward = new ArmFeedforward(0.82027, 0.42835, 0.016127);

    public static SparkFlexConfig pivotConfigBase = new SparkFlexConfig();
    public static SparkFlexConfig pivotConfigLeader = new SparkFlexConfig();

    // thresholds
    public static final double PIVOT_ACTIVE_THRESHOLD = 20;
    public static final double PIVOT_AGITATING_POSITION = 15;
    public static final double PIVOT_AGITATING_POSITION_SECOND = 40;
    public static final double AGITATING_POSITION_FINAL_SECOND = 85;
    public static final double AGITATING_POSITION_FINAL = 95;
    public static final double AGITATING_POSITION_DUMP = 100;
    public static final double PIVOT_BOUNCE_TEST_TIMEOUT = 0.35;
    public static final double AGITATION_INCREMENT = 10;

    public enum IntakeState {
        UNKNOWN,
        STOWED,
        SHOOTING,
        ACTIVE,
        ZERO
    }

    static {
        // TODO: add velocity PID and slots
        pivotConfigBase
                .encoder
                .positionConversionFactor(PIVOT_CONVERSION_FACTOR)
                .velocityConversionFactor(PIVOT_CONVERSION_FACTOR / 60)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .quadratureAverageDepth(5)
                .quadratureMeasurementPeriod(1);

        pivotConfigBase
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIVOT_POSITION_KP, PIVOT_POSITION_KI, PIVOT_POSITION_KD)
                .outputRange(PIVOT_MIN_OUTPUT, PIVOT_MAX_OUTPUT)
                .iZone(4)
                .iMaxAccum(100)
                .maxMotion
                .cruiseVelocity(PIVOT_MAX_VELOCITY, PIVOT_PID_POSITION_SLOT)
                .maxAcceleration(PIVOT_MAX_ACCELERATION, PIVOT_PID_POSITION_SLOT)
                .allowedProfileError(10, PIVOT_PID_POSITION_SLOT);
        pivotConfigBase
                .closedLoop
                .feedForward
                .kS(feedForward.getKs())
                .kV(feedForward.getKv(), PIVOT_PID_VELOCITY_SLOT)
                .kCos(feedForward.getKg())
                .kCosRatio(1.0 / 360.0);

        pivotConfigBase
                .idleMode(SparkFlexConfig.IdleMode.kCoast)
                .smartCurrentLimit(PIVOT_CURRENT_LIMIT_STALL, PIVOT_CURRENT_LIMIT_FREE);

        pivotConfigBase
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(5)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        pivotConfigBase
                .softLimit
                .forwardSoftLimit(PIVOT_FORWARD_SOFT_LIMIT)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(PIVOT_REVERSE_SOFT_LIMIT)
                .reverseSoftLimitEnabled(false);

        pivotConfigLeader.apply(pivotConfigBase);
        pivotConfigLeader.inverted(true);
    }
}
