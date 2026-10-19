/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretSubsystemIO {

    @AutoLog
    class TurretSubsystemInputs {
        public double turretMotorPosition;
        public double turretMotorVelocity;
        public double turretMotorAppliedVoltage;
        public double turretMotorCurrentDraw;
        public double turretMotorTemperature;
        public double turretCANCoderAbsolutePosition;
        public double zeroRotation;
        public double maxMotionPositionSetpoint;
        public double maxMotionVelocitySetpoint;
        public double controllerSetpoint;
        public boolean atTarget;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(TurretSubsystemInputs inputs) {}
    ;

    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     * @param arbFF control value in voltage to compensate for external forces
     */
    default void setVelocity(double velocity, double arbFF) {}
    ;

    /**
     * Sync the relative encoder to the CANCoder
     */
    default void syncEncoder() {}
    ;

    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     */
    default void setVelocity(double velocity) {}
    ;

    /** Cause the turret motor to zero */
    default void zero() {}
    ;

    /**
     * Sets the voltage of the motor
     *
     * @param volt
     */
    default void setVoltage(double volt) {}
    ;

    /**
     * Set the position of the encoder
     *
     * @param position in degrees
     */
    default void setEncoder(double position) {}
    ;

    /**
     * Set the position that the motor will spin to
     *
     * @param position in degrees
     */
    default void setPosition(double position) {}
    ;

    /**
     * Set the position that the motor will spin to following a motion profile
     *
     * @param position in degrees
     */
    default void setProfiledPosition(double position) {}
    ;

    /**
     * Set the motor to a percentage of its maximum applied output
     *
     * @param percent number between -1 and 1
     */
    default void setPercent(double percent) {}
    ;

    /**
     * Set limits that prevent the motor from moving beyond a certain position
     *
     * @param forward the direction that is to be accessed
     * @param value in degrees
     */
    default void setLimit(boolean forward, double value) {}
    ;

    /**
     * Enable the motor soft limits (Here, true = forward, false = reverse)
     *
     * @param forward the direction that is to be accessed
     * @param enabled either enabled or disabled
     */
    default void enableLimit(boolean forward, boolean enabled) {}
    ;
}
