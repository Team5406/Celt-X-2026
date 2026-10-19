package frc.robot.subsystems.LightSubsystem;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LightConstants;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {
    private final LightSubsystemIO io;
    private final LightSubsystemInputsAutoLogged inputs = new LightSubsystemInputsAutoLogged();

    public LightSubsystem(LightSubsystemIO io) {
        this.io = io;
    }

    public Command setColour(RGBWColor colour) {
        return Commands.run(() -> io.setColour(colour), this);
    }

    public Command setAnimation() {
        return Commands.runOnce(() -> io.setAnimation(LightConstants.kGreen), this);
    }

    public Command stopAnimation() {
        return Commands.runOnce(() -> io.stopAnimation());
    }

    public Command noColour() {
        return setColour(LightConstants.kNone);
    }

    public Command greenColour() {
        return setColour(LightConstants.kGreen);
    }

    public Command orangeColour() {
        return setColour(LightConstants.kOrange);
    }

    public Command redColour() {
        return setColour(LightConstants.kRed);
    }

    public Command blueColour() {
        return setColour(LightConstants.kBlue);
    }

    public Command purpleColour() {
        return setColour(LightConstants.kPurple);
    }

    public Command whiteColour() {
        return setColour(LightConstants.kWhite);
    }

    public Command signal() {
        return Commands.run(
                () -> {
                    io.setColour(LightConstants.kGreen);
                },
                this);
    }

    // this does blink but the reef being sensed is always true
    public Command blink() {
        return Commands.repeatingSequence(
                noColour().withTimeout(0.25), greenColour().withTimeout(0.25));
    }

    public Command blinkIntake() {
        return Commands.sequence(
                noColour().withTimeout(0.25),
                purpleColour().withTimeout(0.25),
                noColour().withTimeout(0.25),
                purpleColour().withTimeout(0.25),
                noColour().withTimeout(0.25),
                purpleColour().withTimeout(0.25));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("LightSubsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LightSubsystem", inputs);
        // This method will be called once per scheduler run during simulation
    }
}
