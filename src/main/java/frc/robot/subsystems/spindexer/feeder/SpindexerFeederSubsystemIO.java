/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.spindexer.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerFeederSubsystemIO {

    @AutoLog
    class SpindexerFeederSubsystemInputs {
        public double spindexerFeederMotorPosition;
        public double spindexerFeederMotorVelocity;
        public double spindexerFeederMotorAppliedVoltage;
        public double spindexerFeederMotorCurrentDraw;
        public double spindexerFeederMotorTemperature;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(SpindexerFeederSubsystemInputs inputs) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in rpm of the output
     */
    default void setVelocity(double velocity) {}
    ;

    /**
     * Set the motor to a percentage of its maximum velocity
     *
     * @param percent number between -1 and 1
     */
    default void setPercent(double percent) {}
    ;

    /**
     * Set the voltage of the motor
     *
     * @param volt
     */
    default void setVoltage(double volt) {}
    ;
}
