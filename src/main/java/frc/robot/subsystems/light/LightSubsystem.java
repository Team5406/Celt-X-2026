package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {
    private final LightSubsystemIO io;
    private final LightSubsystemInputsAutoLogged inputs = new LightSubsystemInputsAutoLogged();
    private static boolean isHubActive;
    private static double timeOffset;
    private double absoluteOffset = 1.5; // seconds
    private double autoStartTime = -2;
    private double teleopStartTime = -2;

    public enum CandleState {
        DISABLED,
        ACTIVE,
        ACTIVE_ENDING,
        INACTIVE,
        INACTIVE_ENDING,
        GAMEDATA_EMPTY
    }

    CandleState candleState = CandleState.DISABLED;

    public LightSubsystem(LightSubsystemIO io) {
        this.io = io;
    }

    public int getShiftWithOffsets(double matchTime, double[] shiftStartTimes) {
        for (int i = 5; i >= 0; i--) {
            if (matchTime < shiftStartTimes[i]) {
                return i;
            }
        }

        return 0; // transition
    }

    public void flashLights() {
        double matchTime = 0;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        // if auto is enabled, the hub will always be enabled.
        boolean autoEnabled = DriverStation.isAutonomousEnabled();
        boolean teleopEnabled = DriverStation.isTeleopEnabled();

        if (autoEnabled) {
            if (autoStartTime == -2) {
                autoStartTime = Timer.getFPGATimestamp();
            }
            matchTime = 20 - (Timer.getFPGATimestamp() - autoStartTime);
        }

        if (teleopEnabled) {
            if (teleopStartTime == -2) {
                teleopStartTime = Timer.getFPGATimestamp();
            }
            matchTime = 140 - (Timer.getFPGATimestamp() - teleopStartTime);
        }

        if (DriverStation.isDisabled()) {
            matchTime = -1;
            // only if we run twice in a row without restarting robot code in between
            autoStartTime = -2;
            teleopStartTime = -2;
        }

        isHubActive = autoEnabled;

        String gameData = DriverStation.getGameSpecificMessage();

        boolean redInactiveFirst = false;

        redInactiveFirst = (!gameData.isEmpty() && gameData.charAt(0) == 'R');

        boolean wonAuto = redInactiveFirst == (alliance.get() == Alliance.Red);
        timeOffset = absoluteOffset * (wonAuto ? -1 : 1);

        // shift times including offsets
        double shiftStartTimes[] = {
            140, // transition stage
            130 - absoluteOffset, // shift 1
            105 - timeOffset, // shift 2
            80 + timeOffset, // shift 3
            55 - timeOffset, // shift 4
            30 + absoluteOffset, // endgame
        };

        int shiftWithOffset = getShiftWithOffsets(matchTime, shiftStartTimes);

        // if our hub is already in auto, this is true. therefore there is no need to
        // redefine. although it practically doesnt matter much
        if (!isHubActive) {
            if (shiftWithOffset != 0 && shiftWithOffset != 5) { // if not in transition or endgame
                isHubActive = (shiftWithOffset % 2 == 0) ? wonAuto : !wonAuto;
            } else { // in transition or endgame hub is always active
                isHubActive = true;
            }
        }

        // Set color states
        double flashPeriod = 5.0; // seconds
        // two term if statement to determine if flashing should occur.
        boolean shouldFlash;

        if (shiftWithOffset == 0) { // if you won auto you shouldnt flash into first shift
            shouldFlash = wonAuto && matchTime - shiftStartTimes[shiftWithOffset + 1] <= flashPeriod;
        } else if (shiftWithOffset <= 4) {
            shouldFlash = matchTime - shiftStartTimes[shiftWithOffset + 1] <= flashPeriod;
        } else {
            shouldFlash = !autoEnabled && matchTime <= 5;
        }

        // should be flashing + hub is not active
        if (matchTime < 0) {
            candleState = CandleState.DISABLED;
        } else if (shouldFlash && !isHubActive) {
            candleState = CandleState.INACTIVE_ENDING;
        } else if (shouldFlash && isHubActive) {
            candleState = CandleState.ACTIVE_ENDING;
        } else if (isHubActive && !shouldFlash) {
            candleState = CandleState.ACTIVE;
        } else if (!isHubActive && !shouldFlash) {
            candleState = CandleState.INACTIVE;
        }

        // If we have no game data, we should notify drivers to pay attention themselves
        if (gameData.isEmpty()) {
            candleState = CandleState.GAMEDATA_EMPTY;
        }

        if (matchTime <= -1) {
            candleState = CandleState.DISABLED;
        }

        SmartDashboard.putBoolean("isHubActive", isHubActive);
        SmartDashboard.putBoolean("wonAuto", wonAuto);
        SmartDashboard.putNumber("matchTime", matchTime);
        SmartDashboard.putString("gameData", gameData);
        SmartDashboard.putBoolean("shouldFlash", shouldFlash);
        SmartDashboard.putNumber("shiftWithOffset", shiftWithOffset);

        Logger.recordOutput("LED/isHubActive", isHubActive);
        Logger.recordOutput("LED/wonAuto", wonAuto);
        Logger.recordOutput("LED/matchTime", matchTime);
        Logger.recordOutput("LED/gameData", gameData);
        Logger.recordOutput("LED/shouldFlash", shouldFlash);
        Logger.recordOutput("LED/shiftWithOffset", shiftWithOffset);

        switch (candleState) {
            case DISABLED:
                io.flashLarson(LightConstants.DISABLED_COLOUR);
                break;

            case ACTIVE:
                io.setColour(LightConstants.ACTIVE_COLOUR);
                break;

            case ACTIVE_ENDING:
                io.flashColour(LightConstants.ACTIVE_COLOUR);
                break;

            case INACTIVE:
                io.setColour(LightConstants.INACTIVE_COLOUR);
                break;

            case GAMEDATA_EMPTY:
                io.flashLarson(LightConstants.GAMEDATA_EMPTY);
                break;

            case INACTIVE_ENDING:
                io.flashColour(LightConstants.INACTIVE_COLOUR);
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        flashLights();
        Logger.processInputs("LightSubsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.updateInputs(inputs);
        flashLights();
        Logger.processInputs("LightSubsystem", inputs);
    }
}
