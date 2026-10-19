/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerSubsystemIO {

    @AutoLog
    class IntakeRollerSubsystemInputs {
        public double intakeRollerMotorPosition;
        public double intakeRollerMotorVelocity;
        public double intakeRollerMotorAppliedVoltage;
        public double intakeRollerMotorCurrentDraw;
        public double intakeRollerMotorTemperature;

        public double intakeRollerMotorFollowerPosition;
        public double intakeRollerMotorFollowerVelocity;
        public double intakeRollerMotorFollowerAppliedVoltage;
        public double intakeRollerMotorFollowerCurrentDraw;
        public double intakeRollerMotorFollowerTemperature;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(IntakeRollerSubsystemInputs inputs) {}
    ;

    /**
     * Sets velocity of the motor
     *
     * @param speed in rpm of the output
     */
    default void setVelocity(double velocity) {}
    ;

    // Sets the motor output as a percentage of full power.
    default void setPercent(double percent) {}
    ;

    default void setCurrentLimit() {}
    ;
}
