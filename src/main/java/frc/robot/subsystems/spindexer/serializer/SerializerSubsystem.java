package frc.robot.subsystems.spindexer.serializer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SerializerSubsystem extends SubsystemBase {
    private final SerializerSubsystemIO io;
    private final SerializerSubsystemInputsAutoLogged inputs = new SerializerSubsystemInputsAutoLogged();

    /** Creates a new SerializerSubysystem. */
    public SerializerSubsystem(SerializerSubsystemIO io) {
        this.io = io;
    }

    // Stops the serializer motor
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    // Stop the motor from spinning
                    io.setPercent(0);
                },
                this);
    }

    // Gets the velocity of the serializer motor
    public double getVelocity() {
        return inputs.serializerMotorVelocity;
    }

    // Gets the position of the serializer motor
    public double getPosition() {
        return inputs.serializerMotorPosition;
    }

    // Gets the current draw of the serializer motor
    public double getCurrent() {
        return inputs.serializerMotorCurrentDraw;
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

    public Command setSpinupVelocity() {
        return Commands.run(
                () -> {
                    // Set velocity of the motor
                    io.setVelocity(SerializerConstants.SERIALIZER_VELOCITY);
                },
                this);
    }

    // Sets the percent of max speed of the serializer motor
    public Command setPercent(double percent) {
        return Commands.run(
                () -> {
                    // Enable the percent of the max rpm of the motor
                    io.setPercent(percent);
                },
                this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Serializer", inputs);
        SmartDashboard.putNumber("Serializer Speed", getVelocity());
        SmartDashboard.putNumber("Serializer Position", getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.updateInputs(inputs);
        Logger.processInputs("Serializer", inputs);
        SmartDashboard.putNumber("Serializer Speed", getVelocity());
        SmartDashboard.putNumber("Serializer Position", getPosition());
    }
}
