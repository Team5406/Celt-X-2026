/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.spindexer.serializer;

import org.littletonrobotics.junction.AutoLog;

public interface SerializerSubsystemIO {

    @AutoLog
    class SerializerSubsystemInputs {
        public double serializerMotorPosition;
        public double serializerMotorVelocity;
        public double serializerMotorAppliedVoltage;
        public double serializerMotorCurrentDraw;
        public double serializerMotorTemperature;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(SerializerSubsystemInputs inputs) {}
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

    /*
     * Sets the position
     *
     * @param setpoint rotation value to set the motor to
     */
    default void setPosition(double setpoint) {}
    ;

    /**
     * Sets the percent of max speed
     *
     * @param percent of the max speed (between -1 and 1)
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
     * Set the motor encoder position value to a specific value
     *
     * @param rotation value to set the motor encoder to
     */
    default void setEncoder(double position) {}
    ;
}
