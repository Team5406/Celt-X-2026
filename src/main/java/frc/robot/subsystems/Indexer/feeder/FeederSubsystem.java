/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.Indexer.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
    private final FeederSubsystemIO io;
    private final FeederSubsystemInputsAutoLogged inputs = new FeederSubsystemInputsAutoLogged();

    public FeederSubsystem(FeederSubsystemIO io) {
        this.io = io;
        SmartDashboard.putNumber("Feeder Target RPM", FeederConstants.FEEDER_TARGET_SPEED);
    }

    /** Stop the motor by setting the output to 0 */
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    io.setPercent(0);
                },
                this);
    }

    /** Return the velocity of the motor in rpm */
    public double getVelocity() {
        return inputs.feederMotorVelocity;
    }

    /** Directly set the velocity of the motor. For use in commands */
    public void setVelocityDirect(double velocity) {
        io.setVelocity(velocity);
    }

    /** Return the position of the motor in rotations */
    public double getPosition() {
        return inputs.feederMotorPosition;
    }

    /** Return the current of the motor in amperes */
    public double getCurrent() {
        return inputs.feederMotorCurrentDraw;
    }

    /**
     * Set the percent of the maximum velocity of the motor
     *
     * @param percent of the maximum velocity
     */
    public Command setPercent(double percent) {
        return Commands.run(
                () -> {
                    io.setPercent(percent);
                },
                this);
    }

    public Command setFeederControl(double speed) {
        return Commands.run(() -> FeederConstants.FEEDER_FEEDING_VELOCITY = speed, this);
    }

    /**
     * Set the velocity of the motor
     *
     * @param velocity in rpm
     */
    public Command setVelocity(double velocity) {
        return Commands.run(
                () -> {
                    io.setVelocity(velocity);
                },
                this);
    }

    public Command setTargetVelocity() {
        return new FunctionalCommand(
                () -> {
                    setVelocity(FeederConstants.FEEDER_TARGET_SPEED);
                },
                () -> {},
                interrupted -> {},
                () -> (Math.abs(getVelocity() - FeederConstants.FEEDER_TARGET_SPEED)
                        < FeederConstants.FEEDER_THRESHOLD),
                this);
    }

    /** Runs once per schedule run */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SmartDashboard.putNumber(getName(), getVelocity());
        Logger.processInputs("FeederSubsystem", inputs);
        SmartDashboard.putNumber("Feeder Speed", getVelocity());
        SmartDashboard.putNumber("Feeder Position", getPosition());
        SmartDashboard.putNumber("Feeder Voltage", inputs.feederMotorAppliedVoltage);
        SmartDashboard.putNumber("Feeder Current Draw", inputs.feederMotorCurrentDraw);
    }
}
