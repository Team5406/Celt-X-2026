package frc.robot.subsystems.spindexer.serializer;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;

public class SerializerConstants {
    public static int SERIALIZER_MOTOR_ID = 13;
    public static final int SERIALIZER_MOTOR_COUNT = 1; // NEEDS CHANGE
    public static final double WHEEL_DIAMETER = 0.049;
    public static final double SERIALIZER_GEAR_RATIO = 5.0;
    public static final double SERIALIZER_POSITION_OFFSET = 0.4;
    public static final double SERIALIZER_CONVERSION_FACTOR = 1.0 / SERIALIZER_GEAR_RATIO; // NEEDS
    // CHANGE
    public static final int SERIALIZER_CURRENT_LIMIT = 60; // NEEDS CHANGE
    public static final double SERIALIZER_ACTIVE_SPEED = 0.0;
    public static final double SERIALIZER_VELOCITY = 200.0;
    public static final double SERIALIZER_ZERO_SPEED = 0.0;
    public static final double SERIALIZER_ARB_FF = 0.0;
    public static final FeedForwardGains SERIALIZER_FF = new FeedForwardGains(0.095039, 0.008781, 0.00064189);
    public static final PIDGains SERIALIZER_VELOCITY_PID = new PIDGains(3.7154E-07, 0, 0);
    public static final ClosedLoopSlot SERIALIZER_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;

    public static SparkFlexConfig serializerConfig = new SparkFlexConfig();

    static {
        serializerConfig
                .encoder
                .positionConversionFactor(SERIALIZER_CONVERSION_FACTOR)
                .velocityConversionFactor(SERIALIZER_CONVERSION_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        serializerConfig
                .closedLoop
                .pid(
                        SERIALIZER_VELOCITY_PID.kP(),
                        SERIALIZER_VELOCITY_PID.kI(),
                        SERIALIZER_VELOCITY_PID.kD(),
                        SERIALIZER_PID_VELOCITY_SLOT)
                .feedForward
                .kS(SERIALIZER_FF.kS())
                .kV(SERIALIZER_FF.kV(), SERIALIZER_PID_VELOCITY_SLOT);
        serializerConfig.idleMode(SparkFlexConfig.IdleMode.kCoast).smartCurrentLimit(SERIALIZER_CURRENT_LIMIT);
        serializerConfig.inverted(true);
    }
}
