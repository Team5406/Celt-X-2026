/*
 * Copyright (c) 2026 FRC 5406 - Team CeltX.
 * https://github.com/FRC-5406-Team-CeltX.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelSubsystemIO {

    @AutoLog
    class FlywheelSubsystemInputs {
        public double flywheelMotorPosition;
        public double flywheelMotorVelocity;
        public double flywheelMotorAppliedVoltage;
        public double flywheelMotorCurrentDraw;
        public double flywheelMotorTemperature;
        public double flywheelSecondMotorPosition;
        public double flywheelSecondMotorVelocity;
        public double flywheelSecondMotorAppliedVoltage;
        public double flywheelSecondMotorCurrentDraw;
        public double flywheelSecondMotorTemperature;
        public double flywheelMotorFollowerPosition;
        public double flywheelMotorFollowerVelocity;
        public double flywheelMotorFollowerAppliedVoltage;
        public double flywheelMotorFollowerCurrentDraw;
        public double flywheelMotorFollowerTemperature;
        public double flywheelSecondMotorFollowerPosition;
        public double flywheelSecondMotorFollowerVelocity;
        public double flywheelSecondMotorFollowerAppliedVoltage;
        public double flywheelSecondMotorFollowerCurrentDraw;
        public double flywheelSecondMotorFollowerTemperature;
        public boolean atTarget;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(FlywheelSubsystemInputs inputs) {}
    ;

    /**
     * Sets duty cycle of the motor
     *
     * @param percent as a decimal (from -1 to +1)
     */
    default void setPercent(double percent) {}
    ;
    /**
     * Sets voltage of the motor
     *
     * @param volts in volts of the output
     */
    default void setVoltage(double volts) {}

    /**
     * Sets velocity of the motor
     *
     * @param velocity in rpm of the output
     */
    default void setVelocity(double velocity) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in rpm of the output
     */
    default void setVelocity(double velocity, double arbFF) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param velocity in rpm of the output
     */
    default void setAccel(double acceleration) {}
    ;
}
