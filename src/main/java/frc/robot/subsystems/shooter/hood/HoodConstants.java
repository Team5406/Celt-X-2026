package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.lib.control.PIDGains;

// ALL NEEDS CHANGE
public class HoodConstants {
    // Motor ID
    public static int HOOD_MOTOR_ID = 16;
    public static boolean HOOD_INVERTED = true;
    public static boolean HOOD_ABS_ENCODER_INVERTED = true;
    public static Time SYSID_TIMEOUT = Units.Seconds.of(1.5);

    // Positional values
    public static final double HOOD_FIXED_SHOT_ANGLE = 8.0;
    public static final double HOOD_ABS_ENCODER_GEAR_RATIO = 304.0 / 44; // Confirmed
    public static final double HOOD_GEAR_RATIO = (60.0 / 12.0) * HOOD_ABS_ENCODER_GEAR_RATIO; // empirical
    public static final double HOOD_STOW_POSITION = 8.5; // in degrees
    public static final double HOOD_ABS_ENCODER_OFFSET = 3.721; // in degrees
    public static final double HOOD_ZERO_OFFSET = 7; // in degrees
    public static final double HOOD_TRENCH_ANGLE_THRESHOLD = 12;
    public static final double HOOD_DEBOUNCE_TIME = 0.1; // In seconds
    public static final double HOOD_POSITION_TOLERANCE = 1; // In degrees needs to be adjusted
    public static final double HOOD_ABS_ENCODER_CONVERSION_FACTOR = 360.0 / HOOD_ABS_ENCODER_GEAR_RATIO; // In
    // degrees
    // per
    // rotation
    // of the
    // encoder
    public static final double HOOD_CONVERSION_FACTOR = 360.0 / HOOD_GEAR_RATIO; // In degrees per rotation of the
    // motor
    public static final double HOOD_DEADBAND = 0.1; // Reading between -1 and 1

    // Speed and voltage
    public static final double HOOD_ZEROING_VOLTAGE = 0.05;
    public static final double MAX_VELOCITY = 0.0; // m/s
    public static final double MAX_ACCELERATION = 0.0; // m/s^2

    // Limits
    public static final double HOOD_MIN_OUTPUT = -1;
    public static final double HOOD_MAX_OUTPUT = 1;
    private static final double ODOMETRY_FREQUENCY = 50; // In Hertz
    public static final double HOOD_CURRENT_SPIKE = 0.35;
    public static final double ARB_FEED_FORWARD = 0.35;
    public static final int MOTOR_COUNT = 1;

    public static int HOOD_CURRENT_LIMIT_STALL = 20;
    public static int HOOD_CURRENT_LIMIT_FREE = 25;
    public static double HOOD_FORWARD_SOFT_LIMIT = 49;
    public static double HOOD_REVERSE_SOFT_LIMIT = 8;
    public static double HOOD_MOTION_RANGE = HOOD_FORWARD_SOFT_LIMIT - HOOD_REVERSE_SOFT_LIMIT; // The range of
    // motion the hood
    // can move in
    public static final double STALL_DURATION = 0.5; // The wait time to stop zeroing In seconds

    // ALL NEEDS ADJUSTMENTS PID gains
    public static final PIDGains HOOD_POS_PID = new PIDGains(0.038, 0.0, 0.044099);
    // Works
    public static final ClosedLoopSlot HOOD_PID_POSITION_SLOT = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot HOOD_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;

    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.10619, 0.025418, 0.00050111);
    public static SparkMaxConfig hoodConfig = new SparkMaxConfig();
    public static AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();

    static {
        hoodConfig
                .encoder
                .positionConversionFactor(HOOD_CONVERSION_FACTOR)
                .velocityConversionFactor(HOOD_CONVERSION_FACTOR / 60)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        hoodConfig
                .absoluteEncoder
                .positionConversionFactor(HOOD_ABS_ENCODER_CONVERSION_FACTOR)
                .velocityConversionFactor(HOOD_ABS_ENCODER_CONVERSION_FACTOR / 60)
                .inverted(HOOD_ABS_ENCODER_INVERTED);

        hoodConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(HOOD_POS_PID.kP(), HOOD_POS_PID.kI(), HOOD_POS_PID.kP(), HOOD_PID_POSITION_SLOT)
                .outputRange(HOOD_MIN_OUTPUT, HOOD_MAX_OUTPUT)
                .iZone(4)
                .iMaxAccum(100);
        hoodConfig
                .closedLoop
                .outputRange(HOOD_MIN_OUTPUT, HOOD_MAX_OUTPUT)
                .iZone(4)
                .iMaxAccum(100);
        hoodConfig
                .idleMode(SparkMaxConfig.IdleMode.kBrake)
                .smartCurrentLimit(HOOD_CURRENT_LIMIT_STALL, HOOD_CURRENT_LIMIT_FREE)
                .inverted(HOOD_INVERTED);

        hoodConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(5)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        hoodConfig
                .softLimit
                .forwardSoftLimit(HOOD_FORWARD_SOFT_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(HOOD_REVERSE_SOFT_LIMIT)
                .reverseSoftLimitEnabled(true);
    }
}
