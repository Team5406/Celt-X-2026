package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.vision.Vision;
import frc.robot.subsystems.Indexer.feeder.FeederConstants;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorConstants;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class AutoUnjamFeeding extends Command {
    private final RollerFloorSubsystem m_floor;
    private final FeederSubsystem m_feeder;
    private final FlywheelSubsystem m_flywheel;
    private final Drive m_drive;
    private final Vision m_vision;

    public boolean reverse = false;

    private double startTime;
    public double lastTime;
    public double currTime;

    // bool for reversing; time (long or double) to store the time when you start
    // reversing; LOGIC:
    // if the velocity is less than the threshold, reverse and store the time

    public AutoUnjamFeeding(
            RollerFloorSubsystem floor,
            FeederSubsystem feeder,
            FlywheelSubsystem flywheel,
            Drive drive,
            Vision vision) {
        this.m_floor = floor;
        this.m_feeder = feeder;
        this.m_flywheel = flywheel;
        this.m_drive = drive;
        this.m_vision = vision;
        addRequirements(floor);
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        m_floor.setVelocityVoid(RollerFloorConstants.FLOOR_VELOCITY);
        lastTime = Timer.getFPGATimestamp();
        m_drive.setShooting(true);
    }

    @Override
    public void execute() {
        double currSpeed = m_floor.getVelocity();
        currTime = Timer.getFPGATimestamp();

        if (m_flywheel.readyToShoot()
                && new AlignToHub(m_vision, m_drive).close().getAsBoolean()) {
            m_feeder.setVelocityDirect(
                    new AlignToHub(m_vision, m_drive).close().getAsBoolean()
                            ? FeederConstants.FEEDER_FEEDING_VELOCITY
                            : 0.0);

            if (!reverse && (currTime < lastTime + RollerFloorConstants.UNJAM_DEBOUNCE_TIME)) {
                m_floor.setVelocityVoid(RollerFloorConstants.FLOOR_VELOCITY);
            }

            if (!reverse
                    && Math.abs(currSpeed)
                            < (Math.abs(RollerFloorConstants.FLOOR_VELOCITY * RollerFloorConstants.UNJAM_THRESHOLD))
                    && (currTime > lastTime + RollerFloorConstants.UNJAM_DEBOUNCE_TIME)) {
                m_floor.setUnjam();
                reverse = true;
                startTime = currTime;
            }

            if (reverse && (currTime > (startTime + RollerFloorConstants.UNJAM_DURATION))) {
                m_floor.setspinUp();
                reverse = false;
                lastTime = currTime;
            }
        } else {
            m_feeder.setVelocityDirect(0);
            m_floor.setVelocityDirect(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setShooting(false);
    }
}
