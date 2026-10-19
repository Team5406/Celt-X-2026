/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final IntakeRollerSubsystemIO io;
    private final IntakeRollerSubsystemInputsAutoLogged inputs = new IntakeRollerSubsystemInputsAutoLogged();

    public IntakeRollerSubsystem(IntakeRollerSubsystemIO io) {
        this.io = io;
        SmartDashboard.putNumber("Shooter Target RPM", RollerConstants.INTAKE_TARGET_SPEED);
    }

    public Command stop() {
        return Commands.runOnce(
                () -> {
                    // Stop the intake rollers
                    io.setPercent(0);
                },
                this);
    }

    // Gets velocity of the motor in rpm.
    public double getVelocity() {
        return inputs.intakeRollerMotorVelocity;
    }

    // Get the current draw of the motor in amps.
    public double getCurrent() {
        return inputs.intakeRollerMotorCurrentDraw;
    }

    public void setRoller(double velocity, IntakePivotSubsystem pivot, BooleanSupplier override) {
        if (pivot.getPosition() >= IntakePivotConstants.TANGLED_HEIGHT && !override.getAsBoolean()) {
            io.setVelocity(0);

        } else {
            // Set velocity of the motor
            io.setVelocity(velocity);
        }
    }

    // Set the speed of the intake rollers as a percentage of max speed.
    public Command setPercent(double percent) {

        return Commands.run(
                () -> {
                    io.setPercent(percent);
                },
                this);
    }

    // Set the speed of the intake rollers to the intake speed.
    public Command intake() {

        return Commands.run(
                () -> {
                    io.setVelocity(RollerConstants.INTAKE_TARGET_SPEED);
                },
                this);
    }

    public Command outtake() {

        return Commands.run(
                () -> {
                    io.setVelocity(RollerConstants.OUTTAKE_TARGET_SPEED);
                },
                this);
    }

    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     */
    public Command setVelocity(double velocity, IntakePivotSubsystem pivot, BooleanSupplier override) {
        return Commands.run(
                () -> {
                    setRoller(velocity, pivot, override);
                },
                this);
    }
    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     */
    public Command setVelocity(double velocity, IntakePivotSubsystem pivot) {
        return Commands.run(
                () -> {
                    setRoller(velocity, pivot, () -> false);
                },
                this);
    }

    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     */
    public Command setVelocityAuto(double velocity) {
        return Commands.run(
                () -> {
                    io.setVelocity(3000);
                },
                this);
    }

    /**
     * Sets the velocity of the motorx
     *
     * @param velocity in degrees per second
     */
    public Command setVelocityOnce(double velocity, IntakePivotSubsystem pivot) {
        return Commands.runOnce(
                () -> {
                    setRoller(velocity, pivot, () -> false);
                },
                this);
    }

    /**
     * Sets the velocity of the motorx
     *
     * @param velocity in degrees per second
     */
    public Command setVelocityOnceOver(double velocity, IntakePivotSubsystem pivot) {
        return Commands.runOnce(
                () -> {
                    setRoller(velocity, pivot, () -> true);
                },
                this);
    }

    /**
     * Sets the velocity of the motorx
     *
     * @param velocity in degrees per second
     */
    public Command setVelocityOver(double velocity, IntakePivotSubsystem pivot) {
        return Commands.run(
                () -> {
                    setRoller(velocity, pivot, () -> true);
                },
                this);
    }

    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     */
    public Command setVelocityOnce(double velocity, IntakePivotSubsystem pivot, BooleanSupplier override) {
        return Commands.runOnce(
                () -> {
                    setRoller(velocity, pivot, override);
                },
                this);
    }

    public void setCurrentLimit() {
        io.setCurrentLimit();
    }

    /** Runs once per schedule run */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SmartDashboard.putNumber(getName(), getVelocity());
        Logger.processInputs("RollerSubsystem", inputs);
        SmartDashboard.putNumber("Roller Speed", getVelocity());
        SmartDashboard.putNumber("Roller Voltage", inputs.intakeRollerMotorAppliedVoltage);
        SmartDashboard.putNumber("Roller Current Draw", inputs.intakeRollerMotorCurrentDraw);
    }
}
