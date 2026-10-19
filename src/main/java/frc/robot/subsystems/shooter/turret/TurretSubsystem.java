/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    private final TurretSubsystemIO io;
    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            TurretConstants.TURRET_VELOCITY_PROFILE_PID.kP(),
            TurretConstants.TURRET_VELOCITY_PROFILE_PID.kI(),
            TurretConstants.TURRET_VELOCITY_PROFILE_PID.kD(),
            new TrapezoidProfile.Constraints(
                    TurretConstants.TURRET_MAX_VELOCITY, TurretConstants.TURRET_MAX_ACCELERATION));
    private final Debouncer debouncer = new Debouncer(TurretConstants.DEBOUNCE_TIME);
    private final TurretSubsystemInputsAutoLogged inputs = new TurretSubsystemInputsAutoLogged();
    private final SysIdRoutine sysId;

    public TurretSubsystem(TurretSubsystemIO io) {
        this.io = io;
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
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

    /**
     * Stop the motor by setting the percentage of the maximum applied output to
     * zero
     */
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    io.setPercent(0);
                },
                this);
    }

    /**
     * Gets the displacement of turret camera from center of robot
     */
    public Transform3d getCameraDisplacement() {
        Pose2d camPose =
                new Pose2d().rotateAround(Constants.Vision.turretCenterToCameraOffset, new Rotation2d(getPosition()));
        return new Transform3d(
                        new Translation3d(camPose.getX(), camPose.getY(), 0.0),
                        new Rotation3d(0.0, 0.0, camPose.getRotation().getDegrees()))
                .plus(Constants.Vision.robotCenterToTurretCenter);
    }

    public Command setPercent(DoubleSupplier percent) {
        return Commands.runOnce(
                () -> {
                    io.setPercent(percent.getAsDouble());
                },
                this);
    }

    /** Obtain the velocity of the motor */
    public double getVelocity() {
        return inputs.turretMotorVelocity;
    }

    /** Return whether the motor is at the target position or not */
    public boolean atTarget() {
        return inputs.atTarget;
    }

    /** Return the motor's current position */
    public double getPosition() {
        return inputs.turretMotorPosition;
    }

    /** Return the motor's current in amps */
    public double getCurrent() {
        return inputs.turretMotorCurrentDraw;
    }

    /**
     * Returns the absolute position of the encoder in rotations
     */
    public double getCANCoderAbsolutePosition() {
        return inputs.turretCANCoderAbsolutePosition;
    }

    /**
     * Set the position that the motor will spin to
     *
     * @param position in degrees
     */
    public Command setPosition(double position) {
        return Commands.run(
                        () -> {
                            m_controller.reset(getPosition(), getVelocity());
                            m_controller.setTolerance(TurretConstants.TURRET_POSITION_TOLERANCE);
                        },
                        this)
                .andThen(() -> {
                    double errorCorrection = m_controller.calculate(getPosition(), position);
                    double arbFF = TurretConstants.turretFeedForward.calculate(m_controller.getSetpoint().velocity);
                    io.setVelocity(m_controller.getSetpoint().velocity, errorCorrection + arbFF);
                })
                .finallyDo(() -> io.setPosition(position));
    }

    /** Cause the turret motor to zero */
    public Command zero() {
        return new FunctionalCommand(
                        () -> {
                            io.setVoltage(TurretConstants.ZEROING_VOLTAGE);
                            io.enableLimit(false, false);
                        },
                        () -> {},
                        interrupted -> {
                            io.setEncoder(TurretConstants.TURRET_ZEROED_POSITION);
                            io.setVoltage(0);
                        },
                        () -> debouncer.calculate(inputs.turretMotorCurrentDraw > TurretConstants.TURRET_CURRENT_SPIKE),
                        this)
                .beforeStarting(() -> {
                    io.enableLimit(true, false);
                })
                .andThen(Commands.waitSeconds(TurretConstants.STALL_TIME)
                        .andThen(Commands.run(
                                () -> {
                                    io.enableLimit(false, true);
                                    io.enableLimit(true, true);
                                },
                                this)));
    }

    /** Sync the motor encoder position with the CANCoder position */
    public Command syncEncoder() {
        return Commands.run(
                () -> {
                    io.syncEncoder();
                },
                this);
    }

    /**
     * Sets the velocity of the motor
     *
     * @param velocity in degrees per second
     */
    public Command setVelocity(double velocity) {
        return Commands.run(
                () -> {
                    io.setVelocity(velocity);
                },
                this);
    }

    /**
     * Sets the position of the motor
     *
     * @param position in degrees
     */
    public Command setPosRough(DoubleSupplier position) {
        return Commands.run(
                () -> {
                    io.setPosition(position.getAsDouble());
                },
                this);
    }

    public void setPosition(DoubleSupplier position) {
        io.setPosition(position.getAsDouble());
    }

    public void setPosProfile(double position) {
        io.setProfiledPosition(position);
    }

    public void setPosProfile(DoubleSupplier position) {
        io.setProfiledPosition(position.getAsDouble());
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

    /**
     * Called once per schedule run
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SmartDashboard.putNumber(getName(), getVelocity());
        Logger.processInputs("TurretSubsystem", inputs);
        SmartDashboard.putNumber("Turret Velocity", getVelocity());
        SmartDashboard.putNumber("Turret Position", getPosition());
        SmartDashboard.putNumber("Turret Voltage", inputs.turretMotorAppliedVoltage);
        SmartDashboard.putNumber("Turret Current Draw", inputs.turretMotorCurrentDraw);
    }

    /** Called once per schedule run in simulation */
    @Override
    public void simulationPeriodic() {
        io.updateInputs(inputs);
        SmartDashboard.putNumber(getName(), getVelocity());
        Logger.processInputs("TurretSubsystem", inputs);
        SmartDashboard.putNumber("Turret Velocity", getVelocity());
        SmartDashboard.putNumber("Turret Position", getPosition());
    }
}
