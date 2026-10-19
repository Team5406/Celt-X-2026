/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.pivot;

import frc.robot.subsystems.intake.pivot.IntakePivotConstants.IntakeState;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotSubsystemIO {

    @AutoLog
    class IntakePivotSubsystemInputs {
        public IntakeState state;
        public boolean atTarget;
        public double intakePivotMotorPosition;
        public double intakePivotMotorVelocity;
        public double intakePivotMotorAppliedVoltage;
        public double intakePivotMotorCurrentDraw;
        public double intakePivotMotorTemperature;
        // public double intakePivotMotorFollowPosition;
        // public double intakePivotMotorFollowVelocity;
        public double intakePivotMotorFollowAppliedVoltage;
        public double intakePivotMotorFollowCurrentDraw;
        public double intakePivotMotorFollowTemperature;
        public double intakePivotMotorAbsoluteEncoder;
        public double maxMotionPositionSetpoint;
        public double controllerSetpoint;
        public double maxMotionVelocitySetpoint;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(IntakePivotSubsystemInputs inputs) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in degrees/s of the output
     * @param arbFF in volts
     */
    default void setVelocity(double velocity, double arbFF) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in degrees/s of the output
     */
    default void setVelocity(double velocity) {}
    ;

    /**
     * Sets the percent of max speed
     *
     * @param percent in percent from -1 to 1 as a decimal.
     */
    default void setPercent(double percent) {}
    ;

    /**
     * Sets voltage of the motor
     *
     * @param volt is the voltage in volts
     */
    default void setVoltage(double volt) {}
    ;

    /**
     * Commands the mechanism to move to the given position.
     *
     * @param setpoint Target position in degrees of the mechanism.
     */
    default void setPosition(double setpoint) {}
    ;

    /**
     * Sets the motor encoder position.
     *
     * @param position Encoder position in degrees.
     */
    default void setEncoder(double position) {}
    ;

    /**
     * Set limits that prevent the motor from moving beyond a certain position
     *
     * @param forward the direction that is to be accessed
     * @param value in degrees
     */
    default void setSoftLimit(boolean forward, double value) {}
    ;

    /**
     * Enable the motor soft limits
     *
     * @param forward true if changing forward, false for reverse
     * @param enabled either enabled or disabled
     */
    default void enableSoftLimit(boolean forward, boolean enabled) {}

    /**
     * Set the position that the motor will spin to following a motion profile
     *
     * @param position in degrees
     */
    default void setProfiledPosition(double position) {}
    ;
    ;
}
