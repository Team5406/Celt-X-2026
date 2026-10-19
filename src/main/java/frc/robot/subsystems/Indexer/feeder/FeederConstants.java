/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.Indexer.feeder;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.control.PIDGains;

public class FeederConstants {
    public static final double FEEDER_TARGET_SPEED = 0;
    public static int FEEDER_MOTOR_ID = 14;
    public static final double FEED_GEAR_RATIO = 3.25 / 1;
    public static final double FEEDER_CONVERSION_FACTOR = 1.0 / FEED_GEAR_RATIO;
    public static final int FEEDER_CURRENT_LIMIT = 60; // NEEDS CHANGE
    public static final double FEEDER_MAX_VELOCITY = 4500.0;
    public static final double FEEDER_THRESHOLD = 0.0;
    public static double FEEDER_FEEDING_VELOCITY = 3000;
    public static double FEEDER_INTAKING_VELOCITY = -500;

    // Feed Forward
    public static final SimpleMotorFeedforward feederFeedForward =
            new SimpleMotorFeedforward(0.14028, 0.0026544, 0.00012404);
    public static final ClosedLoopSlot FEEDER_PID_POSITION_SLOT = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot FEEDER_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;
    public static final PIDGains FEEDER_POSITION_PID = new PIDGains(0.00012052, 0.0, 0.0);
    public static final PIDGains FEEDER_VELOCITY_PID = new PIDGains(2.1598E-06, 0.0, 0.0);
    public static SparkFlexConfig feederConfig = new SparkFlexConfig();

    static {
        feederConfig
                .encoder
                .positionConversionFactor(FEEDER_CONVERSION_FACTOR)
                .velocityConversionFactor(FEEDER_CONVERSION_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .quadratureAverageDepth(5)
                .quadratureMeasurementPeriod(1);

        feederConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        FEEDER_POSITION_PID.kP(),
                        FEEDER_POSITION_PID.kI(),
                        FEEDER_POSITION_PID.kD(),
                        FEEDER_PID_POSITION_SLOT);
        feederConfig
                .closedLoop
                .pid(
                        FEEDER_VELOCITY_PID.kP(),
                        FEEDER_VELOCITY_PID.kI(),
                        FEEDER_VELOCITY_PID.kD(),
                        FEEDER_PID_VELOCITY_SLOT)
                .feedForward
                .kS(feederFeedForward.getKs())
                .kV(feederFeedForward.getKv(), FEEDER_PID_VELOCITY_SLOT);

        feederConfig.idleMode(SparkFlexConfig.IdleMode.kCoast).smartCurrentLimit(FEEDER_CURRENT_LIMIT);
        feederConfig.inverted(false);
    }
}
