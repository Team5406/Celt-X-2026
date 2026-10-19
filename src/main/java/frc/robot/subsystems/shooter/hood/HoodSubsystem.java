package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
    private final HoodSubsystemIO io;
    private final Debouncer debouncer = new Debouncer(HoodConstants.HOOD_DEBOUNCE_TIME);
    private final HoodSubsystemInputsAutoLogged inputs = new HoodSubsystemInputsAutoLogged();
    private double lastSpeed = 0;
    private double lastTime = Timer.getFPGATimestamp();
    private final SysIdRoutine sysId;
    private boolean synced = false;

    /** Creates a new HoodSubsystem. */
    public HoodSubsystem(HoodSubsystemIO io) {
        this.io = io;
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        HoodConstants.SYSID_TIMEOUT,
                        (state) -> Logger.recordOutput("Hood/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
        SmartDashboard.putNumber("Hood Manual Position", HoodConstants.HOOD_STOW_POSITION);
        inputs.zeroing = false;
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
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

    // Get motor Position
    public double getVelocity() {
        return inputs.hoodMotorVelocity;
    }

    // Getting motor position
    public double getPosition() {
        return inputs.hoodMotorPosition;
    }

    // Getting motor applied voltage
    public double getVoltage() {
        return inputs.hoodMotorAppliedVoltage;
    }

    // Getting Motor current
    public double getCurrent() {
        return inputs.hoodMotorCurrentDraw;
    }

    // Getting Absolute Encoder Position from REV Throughbore Encoder
    public double getAbsoluteEncoder() {
        return inputs.hoodSparkAbsolutePosition;
    }

    /**
     * Setting the velocity to a specified value
     *
     * @params velocity target
     */
    public Command setVelocity(double velocity) {
        return Commands.run(
                () -> {
                    io.setVelocity(velocity);
                },
                this);
    }

    /**
     * Setting speed to the specified percent of the max speed
     *
     * @params percent of the max speed
     */
    public Command setPercent(DoubleSupplier percent) {
        return Commands.run(
                () -> {
                    io.setPercent(percent.getAsDouble());
                },
                this);
    }

    public Command setHoodAngleManual() {
        return Commands.run(
                () -> {
                    double hoodAngle =
                            SmartDashboard.getNumber("Hood Manual Position", HoodConstants.HOOD_STOW_POSITION);
                    io.setPosition(hoodAngle);
                    Logger.recordOutput("Hood/SmartDashNum", hoodAngle);
                },
                this);
    }

    /**
     * Move to a position \
     *
     * @params angle to move the motor to
     */
    public Command setPosition(DoubleSupplier angle, BooleanSupplier under, Trigger override) {
        return Commands.run(
                () -> {
                    if (under.getAsBoolean() && !override.getAsBoolean()) {
                        io.setPosition(HoodConstants.HOOD_STOW_POSITION);
                    } else {
                        io.setPosition(angle.getAsDouble());
                    }
                },
                this);
    }

    /**
     * Move to a position \
     *
     * @params angle to move the motor to
     */
    public Command setPosition(double angle) {
        return Commands.run(
                () -> {
                    io.setPosition(angle);
                },
                this);
    }

    // Go to the resting position
    public Command stow() {
        return Commands.runOnce(
                () -> {
                    if (!inputs.zeroing) io.setPosition(HoodConstants.HOOD_STOW_POSITION);
                },
                this);
    }

    // Zero the Hood mechanism
    public Command zero() {
        return new FunctionalCommand(
                        () -> {
                            // Start driving for backwards at the start of the command
                            // Disable SoftLimit at the Start of the command
                            io.setPercent(-1 * HoodConstants.HOOD_ZEROING_VOLTAGE);
                            io.enableSoftLimit(false, false);
                        },
                        // No action needed while the command is running
                        () -> {},
                        // Stop driving at the end of the command
                        interrupted -> {
                            io.setEncoder(HoodConstants.HOOD_ZERO_OFFSET);
                            io.setPercent(0);
                        },
                        // End the command when the robot hood's motor current exceeds the specified
                        // threshold
                        () -> debouncer.calculate(inputs.hoodMotorCurrentDraw > HoodConstants.HOOD_CURRENT_SPIKE),
                        this)
                .andThen(() -> {
                    io.setEncoder(HoodConstants.HOOD_ZERO_OFFSET);
                })
                .andThen(() -> {
                    // Re-enable the soft limits at the end of the command, and set voltage to 0 to
                    // prevent any unexpected movement
                    io.setPercent(0);
                    io.enableSoftLimit(false, true);
                });
    }

    // Move the Hood Down
    public Command moveDownVoltage() {
        return Commands.run(
                        () -> {
                            io.setVoltage(-1 * HoodConstants.HOOD_ZEROING_VOLTAGE);
                        },
                        this)
                .beforeStarting(() -> {
                    io.enableSoftLimit(false, false);
                    inputs.zeroing = true;
                });
    }

    // Set the encoder to zero
    public Command setEncoderDown() {
        return Commands.runOnce(
                        () -> {
                            io.setEncoder(HoodConstants.HOOD_ABS_ENCODER_OFFSET);
                        },
                        this)
                .andThen(new WaitCommand(0.1))
                .andThen(() -> {
                    io.enableSoftLimit(false, true);
                    inputs.zeroing = false;
                });
    }

    // Syncronizes the Motor Encoder to the seperate Throughbore
    public Command syncEncoder() {
        return Commands.runOnce(
                () -> {
                    io.syncEncoder();
                },
                this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        SmartDashboard.putNumber("Hood Speed", getVelocity());
        SmartDashboard.putNumber("Hood Position", getPosition());
        if (!synced) {
            io.syncEncoder();
        }
        if (Math.abs(inputs.hoodSparkAbsolutePosition - inputs.hoodMotorPosition) < 0.5) {
            synced = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        SmartDashboard.putNumber("Hood Speed", getVelocity());
        SmartDashboard.putNumber("Hood Position", getPosition());
    }
}
