package frc.robot.subsystems.Indexer.rollerFloor;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;

public class RollerFloorConstants {
    public static int FLOOR_MOTOR_ID = 13;
    public static final int FLOOR_MOTOR_COUNT = 1; // NEEDS CHANGE
    public static final double WHEEL_DIAMETER = 0.049;
    public static final double FLOOR_GEAR_RATIO = 3.0;
    public static final double FLOOR_POSITION_OFFSET = 0.4;
    public static final double FLOOR_CONVERSION_FACTOR = 1.0 / FLOOR_GEAR_RATIO; // NEEDS
    public static final double FLOOR_SHOOTING_DETECT_VELOCITY = 200;
    // CHANGE
    public static final int FLOOR_CURRENT_LIMIT = 60; // NEEDS CHANGE
    public static final double FLOOR_ACTIVE_SPEED = 0.0;
    public static final double FLOOR_VELOCITY = 1200.0;
    public static final double FLOOR_INTAKING_VELOCITY = 800.0;
    public static final double FLOOR_ZERO_SPEED = 0.0;
    public static final double FLOOR_ARB_FF = 0.0;
    public static final FeedForwardGains FLOOR_FF = new FeedForwardGains(0.095039, 0.008781, 0.00064189);
    public static final PIDGains FLOOR_VELOCITY_PID = new PIDGains(3.7154E-07, 0, 0);
    public static final ClosedLoopSlot FLOOR_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;

    public static final double UNJAM_VELOCITY = -FLOOR_VELOCITY;
    public static final double UNJAM_THRESHOLD = 0.0; // decimal percent
    public static final double UNJAM_DURATION = 0.1;
    public static final double UNJAM_DEBOUNCE_TIME = 1.0; // Delay

    public static SparkFlexConfig floorConfig = new SparkFlexConfig();

    static {
        floorConfig
                .encoder
                .positionConversionFactor(FLOOR_CONVERSION_FACTOR)
                .velocityConversionFactor(FLOOR_CONVERSION_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .quadratureAverageDepth(5)
                .quadratureMeasurementPeriod(1);

        floorConfig
                .closedLoop
                .pid(FLOOR_VELOCITY_PID.kP(), FLOOR_VELOCITY_PID.kI(), FLOOR_VELOCITY_PID.kD(), FLOOR_PID_VELOCITY_SLOT)
                .feedForward
                .kS(FLOOR_FF.kS())
                .kV(FLOOR_FF.kV(), FLOOR_PID_VELOCITY_SLOT);
        floorConfig.idleMode(SparkFlexConfig.IdleMode.kCoast).smartCurrentLimit(FLOOR_CURRENT_LIMIT);
        floorConfig.inverted(false);
    }
}
