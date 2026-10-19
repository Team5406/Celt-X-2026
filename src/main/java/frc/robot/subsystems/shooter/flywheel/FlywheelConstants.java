/*
 * Copyright (c) 2026 FRC 5406 - Team CeltX.
 * https://github.com/FRC-5406-Team-CeltX.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;

public class FlywheelConstants {

    public static int FLYWHEEL_MOTOR_ID = 17;
    public static int FLYWHEEL_MOTOR_SECOND_ID = 19;
    public static int FLYWHEEL_FOLLOWER_MOTOR_ID = 18;
    public static int FLYWHEEL_FOLLOWER_MOTOR_SECOND_ID = 20;
    public static double FLYWHEEL_GEAR_RATIO = 27.0 / 34.0; // 27:34 Gear Ratio
    public static final double FLYWHEEL_CONVERSION_FACTOR = 1.0 / FLYWHEEL_GEAR_RATIO;
    public static final double FLYWHEEL_IDLE_PERCENT = 0.1;
    public static final boolean FLYWHEEL_FOLLOWER_INVERSION = true;
    public static final int FLYWHEEL_CURRENT_LIMIT = 100;
    public static final FeedForwardGains FLYWHEEL_FF = new FeedForwardGains(0.0494, 0.00163, 0.00011);
    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.0494, 0.00163, 0.00011);
    public static final PIDGains FLYWHEEL_VELOCITY_PID = new PIDGains(0.0004, 0, 0);
    public static final PIDGains FLYWHEEL_VELOCITY_PROFILE_PID = new PIDGains(0.0004, 0, 0);
    public static final ClosedLoopSlot FLYWHEEL_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;
    public static final double READY_TO_SHOOT_THRESHOLD = 1000;
    public static double FLYWHEEL_FIXED_SHOT_VELOCITY = 2000;
    public static final double MAX_FLYWHEEL_VELOCITY = 8000.0;
    public static final double MAX_FLYWHEEL_ACCELERATION = 3000.0;
    public static final double FLYWHEEL_VELOCITY_TOLERANCE = 100.0;

    // Configs
    public static SparkFlexConfig flywheelConfigBase = new SparkFlexConfig();
    public static SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static SparkFlexConfig flywheelConfigFollower = new SparkFlexConfig();

    static {
        flywheelConfigBase
                .encoder
                .positionConversionFactor(FLYWHEEL_CONVERSION_FACTOR)
                .velocityConversionFactor(FLYWHEEL_CONVERSION_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(1);

        // Flywheel PID configration
        flywheelConfigBase
                .closedLoop
                .pid(
                        FLYWHEEL_VELOCITY_PID.kP(),
                        FLYWHEEL_VELOCITY_PID.kI(),
                        FLYWHEEL_VELOCITY_PID.kD(),
                        FLYWHEEL_PID_VELOCITY_SLOT)
                .feedForward
                .kS(FLYWHEEL_FF.kS())
                .kV(FLYWHEEL_FF.kV(), FLYWHEEL_PID_VELOCITY_SLOT);

        flywheelConfigBase.idleMode(SparkFlexConfig.IdleMode.kCoast).smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);

        flywheelConfig.apply(flywheelConfigBase);
        flywheelConfig.inverted(true);
        flywheelConfigFollower.apply(flywheelConfigBase);
        flywheelConfigFollower.inverted(false);
    }
}
