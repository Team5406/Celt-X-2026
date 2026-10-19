/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodSubsystemIO {

    @AutoLog
    class HoodSubsystemInputs {
        public double hoodMotorPosition;
        public double hoodMotorAppliedVoltage;
        public double hoodMotorCurrentDraw;
        public double hoodMotorTemperature;
        public double hoodMotorVelocity;
        public double hoodMotorAbsolutePosition;
        public double hoodSparkAbsolutePosition;
        public boolean atTarget;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(HoodSubsystemInputs inputs) {}
    ;

    /**
     * Sync relative encoder with CANcoder
     *
     */
    default void syncEncoder() {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in rpm of the output
     * @param arbFF arbitrary feed forward value
     */
    default void setVelocity(double velocity, double arbFF) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in rpm of the output
     */
    default void setVelocity(double velocity) {}
    ;

    /**
     * Sets the percent of max speed (-1 to 1)
     *
     * @param percent of max speed (-1 to 1)
     */
    default void setPercent(double percent) {}
    ;

    /**
     * Sets voltage of the motor
     *
     * @param volt voltage to set the motor to
     */
    default void setVoltage(double volt) {}
    ;

    /*
     * Sets the angle of the motor
     *
     * @param setpoint rotation in degrees to set the motor to
     */
    default void setPosition(double setpoint) {}
    ;

    /*
     * Sets the motor encoder to a specific rotation value
     *
     * @param angle value to set the motor encoder to
     */
    default void setEncoder(double angle) {}
    ;

    /*
     * Adds the softlimit value to motor configuraiton
     *
     * @param direction checks if the desired direction is forwards or backwards
     * (forwards is true)
     *
     * @param limit The desired softlimit to set the motors
     */
    default void setSoftLimit(boolean direction, double limit) {}
    ;

    /*
     * enables the softlimit on the motor
     *
     * @param direction checks if the desired direction is forwards or backwards
     * (forwards is true)
     *
     * @param enabled Check if the limit is enabled or disabled
     */
    default void enableSoftLimit(boolean direction, boolean enabled) {}
    ;
}
