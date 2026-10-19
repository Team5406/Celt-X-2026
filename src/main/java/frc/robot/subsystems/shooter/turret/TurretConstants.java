/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.turret;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.control.PIDGains;

public class TurretConstants {
    public static boolean TURRET_INVERTED = true;
    public static boolean TURRET_CANCODER_INVERTED = false;
    public static final double ALLOWABLE_FORWARD_LIMIT = 400.0;
    public static final double ALLOWABLE_REVERSE_LIMIT = -40.0;
    public static final double ENTER = 10; // Degrees
    public static final double EXIT = 20; // Degrees
    public static double PADDING = ENTER; // Degress
    public static final int TURRET_MOTOR_ID = 15;
    public static final int TURRET_CANCODER_ID = 50;
    public static final int TURRET_CURRENT_LIMIT = 60;
    public static final double TURRET_DEADBAND = 0.5;
    public static final double TURRET_GEAR_RATIO = 280.0 / 30.0; // 280:30 Gear Ratio
    public static final double TURRET_CANCODER_GEAR_RATIO = 280.0 / 15.0; // 280:15 Gear Ratio
    public static final double TURRET_ENCODER_FACTOR = 360 / TURRET_CANCODER_GEAR_RATIO;
    public static final double TURRET_CONVERSION_FACTOR = 7.73;
    public static final double ZEROING_VOLTAGE = 0.0;
    public static final double TURRET_MAX_VELOCITY = 680;
    public static final double TURRET_MAX_ACCELERATION = 3600;
    public static final double TURRET_ZEROED_POSITION = 0.0;
    private static final double CANCODER_DEGREES_CONVERSION = 0.4917;
    public static final double TURRET_CANCODER_CONVERSION_FACTOR = 360.0 / CANCODER_DEGREES_CONVERSION;
    private static final int TURRET_CURRENT_LIMIT_STALL = 40;
    private static final int TURRET_CURRENT_LIMIT_FREE = 60;
    private static final double ODOMETRY_FREQUENCY = 50.0; // In hertz
    public static final double STALL_TIME = 0.5;
    public static final double TURRET_FORWARD_SOFT_LIMIT = 410;
    public static final double TURRET_REVERSE_SOFT_LIMIT = -50;
    public static final double TURRET_CURRENT_SPIKE = 0.0;
    public static final double TURRET_POSITION_TOLERANCE = 1;
    public static final double DEBOUNCE_TIME = 0.1;
    public static final SimpleMotorFeedforward turretFeedForward =
            new SimpleMotorFeedforward(0.21859, 0.013446, 0.0011084);
    // PID
    public static final ClosedLoopSlot TURRET_PID_POSITION_SLOT = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot TURRET_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;
    public static final PIDGains TURRET_POSITION_PID = new PIDGains(0.05, 0.0, 0.005);
    public static final PIDGains TURRET_VELOCITY_PID = new PIDGains(0.00000127, 0, 0);
    public static final PIDGains TURRET_VELOCITY_PROFILE_PID = new PIDGains(0, 0, 0);

    // Config
    public static SparkFlexConfig turretConfig = new SparkFlexConfig();

    static {
        turretConfig
                .encoder
                .positionConversionFactor(TURRET_CONVERSION_FACTOR)
                .velocityConversionFactor(TURRET_CONVERSION_FACTOR / 60)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .quadratureAverageDepth(10);

        turretConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        TURRET_POSITION_PID.kP(),
                        TURRET_POSITION_PID.kI(),
                        TURRET_POSITION_PID.kD(),
                        TURRET_PID_POSITION_SLOT);
        turretConfig.closedLoop.pid(
                TURRET_VELOCITY_PID.kP(), TURRET_VELOCITY_PID.kI(), TURRET_VELOCITY_PID.kD(), TURRET_PID_VELOCITY_SLOT);
        turretConfig
                .closedLoop
                .maxMotion
                .cruiseVelocity(TURRET_MAX_VELOCITY, TURRET_PID_POSITION_SLOT)
                .maxAcceleration(TURRET_MAX_ACCELERATION, TURRET_PID_POSITION_SLOT)
                .allowedProfileError(10, TURRET_PID_POSITION_SLOT);
        turretConfig
                .closedLoop
                .feedForward
                .kS(turretFeedForward.getKs(), TURRET_PID_POSITION_SLOT) // using position for maxMotion
                .kV(turretFeedForward.getKv(), TURRET_PID_POSITION_SLOT);
        turretConfig
                .idleMode(SparkFlexConfig.IdleMode.kCoast)
                .smartCurrentLimit(TURRET_CURRENT_LIMIT_STALL, TURRET_CURRENT_LIMIT_FREE)
                .inverted(TURRET_INVERTED);

        turretConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(5)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        turretConfig
                .softLimit
                .forwardSoftLimit(TURRET_FORWARD_SOFT_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(TURRET_REVERSE_SOFT_LIMIT)
                .reverseSoftLimitEnabled(true);
    }
}
