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
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;

public class FlywheelConstants {

    public static int FLYWHEEL_MOTOR_ID = 17;
    public static int FLYWHEEL_FOLLOWER_MOTOR_ID = 18;
    public static double FLYWHEEL_GEAR_RATIO = 22.0 / 24.0; // 22:24 Gear Ratio Upscale
    public static final double FLYWHEEL_CONVERSION_FACTOR = 1.0 / FLYWHEEL_GEAR_RATIO; // NEEDS CHANGE
    public static final boolean FLYWHEEL_FOLLOWER_INVERSION = true; // NEEDS CHANGE
    public static final int FLYWHEEL_CURRENT_LIMIT = 60; // NEEDS CHANGE
    public static final FeedForwardGains FLYWHEEL_FF = new FeedForwardGains(0.045745, 0.001686, 0.000091); //
    public static final PIDGains FLYWHEEL_VELOCITY_PID = new PIDGains(0.00005, 0, 0);
    public static final ClosedLoopSlot FLYWHEEL_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;
    public static double FLYWHEEL_FIXED_SHOT_VELOCITY = 3500;

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
                .uvwAverageDepth(2);

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
        flywheelConfigFollower.apply(flywheelConfigBase);
        flywheelConfigFollower.follow(FLYWHEEL_MOTOR_ID, FLYWHEEL_FOLLOWER_INVERSION);
    }
}
