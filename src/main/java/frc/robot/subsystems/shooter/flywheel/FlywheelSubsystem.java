/*
 * Copyright (c) 2026 FRC 5406 - Team CeltX.
 * https://github.com/FRC-5406-Team-CeltX.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
    private final FlywheelSubsystemIO io;
    private final FlywheelSubsystemInputsAutoLogged inputs = new FlywheelSubsystemInputsAutoLogged();
    private final SysIdRoutine sysId;

    public FlywheelSubsystem(FlywheelSubsystemIO io) {
        this.io = io;
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(5),
                        null,
                        (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
        SmartDashboard.putNumber("Flywheel Manual Speed", 0.0);
    }

    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            FlywheelConstants.FLYWHEEL_VELOCITY_PID.kP(),
            FlywheelConstants.FLYWHEEL_VELOCITY_PID.kI(),
            FlywheelConstants.FLYWHEEL_VELOCITY_PID.kD(),
            new TrapezoidProfile.Constraints(
                    FlywheelConstants.MAX_FLYWHEEL_VELOCITY, FlywheelConstants.MAX_FLYWHEEL_ACCELERATION));

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
    // Sets motor output voltage
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    // Stops the motor.
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    io.setPercent(0);
                },
                this);
    }

    // Gets velocity of the motor in rpm.
    public double getVelocity() {
        return inputs.flywheelMotorVelocity;
    }

    // Checks if flywheel velocity is above threshold necessary to not jam
    public boolean readyToShoot() {
        return inputs.flywheelMotorVelocity > FlywheelConstants.READY_TO_SHOOT_THRESHOLD;
    }

    // Gets current of the motor in rpm.
    public double getCurrent() {
        return inputs.flywheelMotorCurrentDraw;
    }

    // Sets duty cycle of the motor in rpm.
    public double getPosition() {
        return inputs.flywheelMotorPosition;
    }

    /**
     * Setting speed to the specified percent of the max speed
     *
     * @params percent of the max speed
     */
    public Command setPercent(double percent) {
        return Commands.run(
                () -> {
                    io.setPercent(percent);
                },
                this);
    }
    /**
     * Setting speed to the specified percent of the max speed
     *
     * @params percent of the max speed
     */
    public Command setPercentOnce(double percent) {
        return Commands.runOnce(
                () -> {
                    io.setPercent(percent);
                },
                this);
    }
    // Sets the velocity of the serializer motor
    public Command setVelocity(DoubleSupplier velocity) {
        return Commands.run(
                () -> {
                    // Set velocity of the motor
                    io.setVelocity(velocity.getAsDouble());
                },
                this);
    }

    /**
     * Set the position of the pivot in degrees.
     *
     * @param angle the target position of the pivot in degrees.
     */
    public Command setVelocityProf(DoubleSupplier velocity) {
        return Commands.runOnce(
                        () -> {
                            // m_controller.reset(getPosition(), getVelocity());
                            m_controller.setTolerance(FlywheelConstants.FLYWHEEL_VELOCITY_TOLERANCE);
                            Logger.recordOutput("Desired velocity", velocity);
                        },
                        this)
                .andThen(() -> {
                    double errorCorrection = m_controller.calculate(getVelocity(), velocity.getAsDouble());

                    double profiledVelocity = m_controller.getSetpoint().position;

                    double profiledAcceleration = m_controller.getSetpoint().velocity;

                    double arbFF = FlywheelConstants.feedForward.calculate(profiledAcceleration);
                    Logger.recordOutput("Profiled Velocity", profiledVelocity);
                    io.setVelocity(profiledVelocity, errorCorrection + arbFF);
                })
                .finallyDo(() -> io.setVelocity(velocity.getAsDouble()));
    }

    // Sets the velocity of the serializer motor
    public Command setVelocity(double velocity) {
        return Commands.run(
                () -> {
                    // Set velocity of the motor
                    io.setVelocity(velocity);
                },
                this);
    }

    public Command setFlywheelSpeedManual() {
        return Commands.run(
                () -> {
                    double flywheelSpeed = SmartDashboard.getNumber("Flywheel Manual Speed", 0.0);
                    io.setVelocity(flywheelSpeed);
                    Logger.recordOutput("Flywheel/SmartDashNum", flywheelSpeed);
                },
                this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        SmartDashboard.putNumber("Flywheel Speed", getVelocity());
        SmartDashboard.putNumber("Flywheel Position", getPosition());
        SmartDashboard.putNumber("Flywheel Current", getCurrent());
        SmartDashboard.putNumber("Flywheel Voltage", inputs.flywheelMotorAppliedVoltage);
    }
}
