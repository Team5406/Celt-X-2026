/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.rollers;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;

public class RollerConstants {
    public static final double INTAKE_TARGET_SPEED = 1000; // RPM
    public static final double OUTTAKE_TARGET_SPEED = -700; // RPM

    public static int ROLLER_MOTOR_ID = 9; // Left
    public static int ROLLER_MOTOR_FOLLOWER_ID = 10; // Right
    public static boolean ROLLER_MOTOR_FOLLOWER_INVERSION = true;

    public static final double ROLLER_GEAR_RATIO = 1.0; // NEEDS CHANGE
    public static final double ROLLER_CONVERSION_FACTOR = 1.0 / ROLLER_GEAR_RATIO; // NEEDS CHANGE
    public static final int ROLLER_CURRENT_LIMIT = 60; // NEEDS CHANGE
    public static final FeedForwardGains ROLLER_FF = new FeedForwardGains(0, 0.038, 0);
    public static final PIDGains ROLLER_POSITION_PID = new PIDGains(0, 0, 0);
    public static final PIDGains ROLLER_VELOCITY_PID = new PIDGains(0, 0, 0);
    public static final ClosedLoopSlot ROLLER_PID_VELOCITY_SLOT = ClosedLoopSlot.kSlot1;
    public static final double ROLLER_ACTIVE_VELOCITY = 2000.0;

    public static SparkFlexConfig rollerConfigLeader = new SparkFlexConfig();
    public static SparkFlexConfig rollerConfigBase = new SparkFlexConfig();
    public static SparkFlexConfig rollerConfigFollower = new SparkFlexConfig();

    static {
        rollerConfigBase
                .encoder
                .positionConversionFactor(ROLLER_CONVERSION_FACTOR)
                .velocityConversionFactor(ROLLER_CONVERSION_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        rollerConfigBase.idleMode(SparkFlexConfig.IdleMode.kCoast).smartCurrentLimit(ROLLER_CURRENT_LIMIT);
        rollerConfigBase
                .closedLoop
                .pid(
                        ROLLER_VELOCITY_PID.kP(),
                        ROLLER_VELOCITY_PID.kI(),
                        ROLLER_VELOCITY_PID.kD(),
                        ROLLER_PID_VELOCITY_SLOT)
                .feedForward
                .kS(ROLLER_FF.kS())
                .kV(ROLLER_FF.kV(), ROLLER_PID_VELOCITY_SLOT);
        rollerConfigLeader.apply(rollerConfigBase);
        rollerConfigFollower.apply(rollerConfigBase);
        rollerConfigFollower.follow(ROLLER_MOTOR_ID, ROLLER_MOTOR_FOLLOWER_INVERSION);
    }
}
