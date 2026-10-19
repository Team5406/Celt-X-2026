package frc.robot.subsystems.Indexer.rollerFloor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerFloorSubsystem extends SubsystemBase {
    private final RollerFloorSubsystemIO io;
    private final RollerFloorSubsystemInputsAutoLogged inputs = new RollerFloorSubsystemInputsAutoLogged();

    /** Creates a new FloorSubysystem. */
    public RollerFloorSubsystem(RollerFloorSubsystemIO io) {
        this.io = io;
    }

    // Stops the floor motor
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    // Stop the motor from spinning
                    io.setPercent(0);
                    inputs.isShooting = false;
                },
                this);
    }

    // Gets the velocity of the floor motor
    public double getVelocity() {
        return inputs.floorMotorVelocity;
    }

    /** Directly set the velocity of the motor. For use in commands */
    public void setVelocityDirect(double velocity) {
        io.setVelocity(velocity);
        inputs.isShooting = true;
    }

    // Gets the position of the floor motor
    public double getPosition() {
        return inputs.floorMotorPosition;
    }

    // Gets the current draw of the floor motor
    public double getCurrent() {
        return inputs.floorMotorCurrentDraw;
    }

    public boolean isShooting() {
        return inputs.isShooting;
    }

    // Sets the velocity of the floor motor
    public Command setVelocity(double velocity) {
        return Commands.run(
                () -> {
                    // Set velocity of the motor
                    io.setVelocity(velocity);
                },
                this);
    }

    public void setVelocityVoid(double velocity) {
        io.setVelocity(velocity);
    }

    public Command setSpinupVelocity() {
        return Commands.run(
                () -> {
                    // Set velocity of the motor
                    io.setVelocity(RollerFloorConstants.FLOOR_VELOCITY);
                    inputs.isShooting = true;
                },
                this);
    }

    // Sets the percent of max speed of the floor motor
    public Command setPercent(double percent) {
        return Commands.run(
                () -> {
                    // Enable the percent of the max rpm of the motor
                    io.setPercent(percent);
                },
                this);
    }

    public void setspinUp() {
        io.setVelocity(RollerFloorConstants.FLOOR_VELOCITY);
        inputs.isShooting = true;
    }

    public void setUnjam() {
        io.setVelocity(RollerFloorConstants.UNJAM_VELOCITY);
        inputs.isShooting = true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Floor", inputs);
        SmartDashboard.putNumber("Floor Speed", getVelocity());
        SmartDashboard.putNumber("Floor Position", getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.updateInputs(inputs);
        Logger.processInputs("Floor", inputs);
        SmartDashboard.putNumber("Floor Speed", getVelocity());
        SmartDashboard.putNumber("Floor Position", getPosition());
    }
}
