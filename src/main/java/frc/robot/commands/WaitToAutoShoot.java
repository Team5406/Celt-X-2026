package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.vision.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.Zone;
import org.littletonrobotics.junction.Logger;

public class WaitToAutoShoot extends Command {
    private final Vision m_Vision;
    private final Drive m_Drive;
    private final int m_TagCount;

    public WaitToAutoShoot(Vision vision, Drive drive, int tagCount) {
        this.m_Vision = vision;
        this.m_Drive = drive;
        this.m_TagCount = tagCount;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        // Wait until we can see X tags && Wait until (roll)(pitch) is within tolerance
        Logger.recordOutput("WaitToShoot/currentZone", m_Drive.currentZone.toString());
        Logger.recordOutput("WaitToShoot/2tags", m_Vision.canSeeTags(m_TagCount));
        Logger.recordOutput("WaitToShoot/isFlat", m_Drive.isFlat());

        return (m_Drive.currentZone == Zone.CELTX_ALLIANCE);
    }
}
