package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AlignToHub extends Command {
    private final Drive m_Drive;
    private final Vision m_Vision;
    private final TurretSubsystem turret;

    public AlignToHub(Vision vision, Drive drive, TurretSubsystem turret) {
        this.m_Vision = vision;
        this.m_Drive = drive;
        this.turret = turret;
        addRequirements(turret);
    }

    public Translation2d getTurretTranslation() {
        Translation2d turretOffset = new Translation2d(
                Constants.Vision.robotCenterToTurretCenter.getTranslation().getX(),
                Constants.Vision.robotCenterToTurretCenter.getTranslation().getY());
        Translation2d baseTranslation = m_Drive.getPose().getTranslation().plus(turretOffset);
        Translation2d rotatedTranslation = baseTranslation.rotateAround(
                m_Drive.getPose().getTranslation(), m_Drive.getPose().getRotation());

        return rotatedTranslation;
    }

    //
    public Translation2d getTurretDisplacementFromTarget() {
        return m_Drive.targetShotPose.getTranslation().minus(getTurretTranslation());
    }

    public double getTurretDistanceFromTarget() {
        return Math.hypot(
                getTurretDisplacementFromTarget().getX(),
                getTurretDisplacementFromTarget().getY());
    }

    /*
     * Uses the yaw of the target tag to calculate the angle the turret needs to
     * turn to
     * to be facing the hub.
     * This is used as a backup in case the global pose is inaccurate or the target
     * tag is not visible.
     * Then this calculates the target angle from 0-360
     */
    public Optional<Double> getTargetTurretAngleFromYaw() {
        Set<Integer> filter = DriverStation.getAlliance().isPresent() // TODO (What happens if no alliance is present?)
                        && DriverStation.getAlliance().get() == Alliance.Blue
                ? Constants.Vision.BLUE_HUB_IDS
                : Constants.Vision.RED_HUB_IDS;
        int centerTag = m_Vision.getCentermostTag(filter).orElse(0);
        Optional<Double> targetYaw = m_Vision.getTagYaw(centerTag);

        return targetYaw.map(yaw -> {
            double robotHeading = m_Drive.getPose().getRotation().getDegrees() >= 0
                    ? m_Drive.getPose().getRotation().getDegrees()
                    : m_Drive.getPose().getRotation().getDegrees() + 360;
            double turretTargetAngle = (robotHeading + yaw) % 360;
            return turretTargetAngle > 0 ? turretTargetAngle : turretTargetAngle + 360;
        });
    }

    /*
     * Using the global pose and the target tag pose, calculates the angle the
     * turret
     * needs to turn to to be facing the hub.
     */
    public double getTargetTurretAngle() {
        Translation2d displacement = getTurretDisplacementFromTarget();

        double angleToTarget = Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())) >= 0
                ? Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()))
                : Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())) + 360;

        double robotHeading = m_Drive.getPose().getRotation().getDegrees() >= 0
                ? m_Drive.getPose().getRotation().getDegrees()
                : m_Drive.getPose().getRotation().getDegrees() + 360;

        double turretTargetAngle = angleToTarget - robotHeading;

        return turretTargetAngle > 0 ? turretTargetAngle : turretTargetAngle + 360;
    }

    // Debugging
    public double getAngle() {
        Translation2d displacement = getTurretDisplacementFromTarget();
        return Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())) >= 0
                ? Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()))
                : Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())) + 360;
    }

    /*
     * Calculates the optimal angle using displacement for the turret to turn to,
     * taking into account
     * the
     * current angle of the turret and the target angle.
     */

    public double getOptimalAngle() {
        double current = turret.getPosition();
        double target = getTargetTurretAngle();
        double dist1 = Math.abs(current - target);
        double dist2 = Math.abs(360 - dist1);
        double chosen = 0;

        if (Math.abs(dist1 - dist2) < TurretConstants.PADDING) {
            TurretConstants.PADDING = TurretConstants.EXIT;
        } else {
            TurretConstants.PADDING = TurretConstants.ENTER;
        }

        if (Math.abs(dist1 - dist2) < TurretConstants.PADDING) {
            return target;
        }

        if (dist1 < dist2) {
            chosen = dist1;
        } else {
            chosen = dist2;
        }

        if (current + Math.signum(current - target) * dist2 > TurretConstants.ALLOWABLE_FORWARD_LIMIT
                || current + Math.signum(current - target) * dist2 < TurretConstants.ALLOWABLE_REVERSE_LIMIT) {
            return target;
        }

        if (current > target) {
            if (chosen == dist1) chosen *= -1;
        }
        if (current < target) {
            if (chosen == dist2) chosen *= -1;
        }

        return current + chosen;
    }

    // Logs out everything
    public void log() {
        // Global Pose Logs
        Logger.recordOutput("Turret Target Optimal Angle", getOptimalAngle());
        Logger.recordOutput("Turret Target Angle", getTargetTurretAngle());
        Logger.recordOutput("Hub Pose", m_Drive.targetShotPose);
        Logger.recordOutput("Turret Target Displacement", getTurretDisplacementFromTarget());
        Logger.recordOutput("Turret Target Distance", getTurretDistanceFromTarget());
        Logger.recordOutput(
                "Turret Target YawAngle", getTargetTurretAngleFromYaw().orElse(0.5));
        Logger.recordOutput(
                "Turret Pose", new Pose2d(getTurretTranslation(), new Rotation2d(Math.toRadians(getAngle()))));
        Logger.recordOutput("Padding: ", TurretConstants.PADDING);
        Logger.recordOutput(
                "Robot Pose Degrees", (((m_Drive.getPose().getRotation().getDegrees()) % 360) + 360) % 360);
        Logger.recordOutput("Distance1", Math.abs(turret.getPosition() - getTargetTurretAngle()));
        Logger.recordOutput("Distance2", Math.abs(360.0 - Math.abs(turret.getPosition() - getTargetTurretAngle())));
    }

    public Command align() {
        return Commands.run(() -> {
            turret.setPosition(() -> getOptimalAngle());
        }); // .andThen(() ->
        // turret.setPosProfile((getTargetTurretAngleFromYaw().orElse(getOptimalAngle()))));
    }
}
