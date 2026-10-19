/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */

package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants.IntakeState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
    private final IntakePivotSubsystemIO io;
    private final IntakePivotSubsystemInputsAutoLogged inputs = new IntakePivotSubsystemInputsAutoLogged();
    private boolean isDefaultStow = true;
    private double voltageValue = 0.0;
    private final SysIdRoutine sysId;

    public IntakePivotSubsystem(IntakePivotSubsystemIO io) {
        this.io = io;
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
        setState(IntakeState.UNKNOWN);
    }

    private final Debouncer debouncer = new Debouncer(IntakePivotConstants.DEBOUNCE_TIME);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            IntakePivotConstants.PIVOT_PROFILE_PID.kP(),
            IntakePivotConstants.PIVOT_PROFILE_PID.kI(),
            IntakePivotConstants.PIVOT_PROFILE_PID.kD(),
            new TrapezoidProfile.Constraints(
                    IntakePivotConstants.PIVOT_MAX_VELOCITY, IntakePivotConstants.PIVOT_MAX_ACCELERATION));

    // Stop the motors.
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    io.setPercent(0);
                },
                this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    // Stop the motors.
    public Command moveDownVoltage() {
        return Commands.run(
                () -> {
                    io.setVoltage(-1 * IntakePivotConstants.PIVOT_ZEROING_VOLTAGE);
                },
                this);
    }

    // Gets velocity of the motor in deg/s.
    public double getVelocity() {
        return inputs.intakePivotMotorVelocity;
    }

    // Get the position of the motor in degrees.
    public double getPosition() {
        return inputs.intakePivotMotorPosition;
    }

    // Get the voltage of the motor in volts.
    public double getVoltage() {
        return inputs.intakePivotMotorAppliedVoltage;
    }

    // Gets the current of the motor in amps.
    public double getCurrent() {
        return inputs.intakePivotMotorCurrentDraw;
    }

    // Gets the position of the absolute encoder in degrees.
    public double getAbsoluteEncoder() {
        return inputs.intakePivotMotorAbsoluteEncoder;
    }

    // Gets whether the motor PID is at the set point
    public BooleanSupplier atGoal() {
        return () -> inputs.atTarget;
    }

    public IntakeState getState() {
        return inputs.state;
    }

    public void setState(IntakeState state) {
        inputs.state = state;
    }

    /**
     * Set the position of the pivot in degrees.
     *
     * @param angle the target position of the pivot in degrees.
     */
    public Command setPosition(double angle) {
        return Commands.run(
                        () -> {
                            m_controller.reset(getPosition(), getVelocity());
                            m_controller.setTolerance(IntakePivotConstants.PIVOT_POSITION_TOLERANCE);
                            Logger.recordOutput("Desired Position", angle);
                            SmartDashboard.putNumber("Desired Pivot Position", angle);
                        },
                        this)
                .andThen(() -> {
                    double arbFF =
                            IntakePivotConstants.feedForward.calculate(angle, m_controller.getSetpoint().velocity);
                    double errorCorrection = m_controller.calculate(getPosition(), angle);
                    io.setVelocity(m_controller.getSetpoint().velocity, errorCorrection + arbFF);
                })
                .finallyDo(() -> io.setPosition(angle));
    }

    /**
     * Sets the position of the motor
     *
     * @param position in degrees
     */
    public Command setPosRough(double position) {
        return Commands.run(
                () -> {
                    io.setPosition(position);
                },
                this);
    }

    /**
     * Sets the position of the motor via MaxMotion
     *
     * @param position in degrees
     */
    public Command setPosProfiled(double position) {
        return Commands.run(
                () -> {
                    io.setProfiledPosition(position);
                },
                this);
    }

    public Command setVoltage() {
        return Commands.run(() -> io.setVoltage(voltageValue));
    }

    // Zero the pivot by checking for a stall current spike
    public Command zero(boolean up) {
        return new FunctionalCommand(
                () -> {
                    io.setVoltage((up ? 1 : -1) * IntakePivotConstants.PIVOT_ZEROING_VOLTAGE);
                    // io.enableSoftLimit(up, false);
                },
                () -> {
                    io.setVoltage((up ? 1 : -1) * IntakePivotConstants.PIVOT_ZEROING_VOLTAGE);
                },
                (interrupted) -> {
                    if (!interrupted) {
                        io.setEncoder(
                                up
                                        ? IntakePivotConstants.PIVOT_ZEROED_UP_POSITION
                                        : IntakePivotConstants.PIVOT_ZEROED_DOWN_POSITION);
                    }
                    io.setVoltage(0);
                    // io.enableSoftLimit(up, true);

                },
                () -> debouncer.calculate(
                        inputs.intakePivotMotorCurrentDraw > IntakePivotConstants.PIVOT_CURRENT_SPIKE),
                this);
    }

    public Command setEncoder(double position) {
        return Commands.runOnce(
                () -> {
                    io.setEncoder(position);
                },
                this);
    }

    public Command setEncoderDown() {
        return Commands.runOnce(
                () -> {
                    io.setEncoder(IntakePivotConstants.PIVOT_ZEROED_DOWN_POSITION);
                    inputs.state = IntakeState.ACTIVE;
                },
                this);
    }

    public void setEncoders(double position) {
        io.setEncoder(position);
    }

    public boolean pivotActive() {
        return Math.abs(getPosition() - IntakePivotConstants.PIVOT_ACTIVE_POSITION)
                < IntakePivotConstants.PIVOT_ACTIVE_THRESHOLD;
    }

    public Command activePos() {
        return Commands.runOnce(() -> {
            io.setPosition(IntakePivotConstants.PIVOT_ACTIVE_POSITION);
            inputs.state = IntakeState.ACTIVE;
        });
    }

    public Command stowedPos() {
        return Commands.run(() -> {
            io.setPosition(IntakePivotConstants.PIVOT_STOW_POSITION);
            inputs.state = IntakeState.STOWED;
        });
    }

    public Command shootingPos() {
        return Commands.run(() -> {
            io.setPosition(IntakePivotConstants.PIVOT_SHOOTING_POSITION);
            inputs.state = IntakeState.SHOOTING;
        });
    }

    public Command shootingPosSecond() {
        return Commands.run(() -> {
            io.setPosition(IntakePivotConstants.PIVOT_SHOOTING_POSITION_SECOND);
            inputs.state = IntakeState.SHOOTING;
        });
    }

    public Command shootingPosOnce() {
        return Commands.runOnce(() -> {
            io.setPosition(IntakePivotConstants.PIVOT_SHOOTING_POSITION);
            inputs.state = IntakeState.SHOOTING;
        });
    }

    public Command agitate() {
        return Commands.run(
                () -> {
                    if (getPosition() < IntakePivotConstants.AGITATING_POSITION_DUMP) {
                        io.setPercent(0.4);
                    } else {
                        io.setPercent(0);
                    }
                },
                this);
    }

    public void defaultIntakePos() {
        if (inputs.state == IntakePivotConstants.IntakeState.STOWED) {
            io.setPosition(IntakePivotConstants.PIVOT_STOW_POSITION);
        } else if (inputs.state == IntakePivotConstants.IntakeState.UNKNOWN) {
            io.setVoltage(0);
        } else {
            io.setPosition(IntakePivotConstants.PIVOT_ACTIVE_POSITION);
        }
    }

    public Command defaultCommand() {
        return Commands.run(
                () -> {
                    defaultIntakePos();
                },
                this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        SmartDashboard.putNumber("Pivot Speed", getVelocity());
        SmartDashboard.putNumber("Pivot X", getVoltage());
        voltageValue = SmartDashboard.getNumber("Pivot X", 0);
        SmartDashboard.putNumber("Pivot Position", getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        SmartDashboard.putNumber("Pivot Speed", getVelocity());

        SmartDashboard.putNumber("Pivot Position", getPosition());
    }
}
