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
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignToHub extends Command {
    private final Drive m_Drive;
    private final Vision m_Vision;
    private TurnToAngle turnToAngle;

    public AlignToHub(Vision vision, Drive drive) {
        this.m_Vision = vision;
        this.m_Drive = drive;
        turnToAngle = new TurnToAngle(
                DriveConstants.AUTO_AIM_MARGIN,
                DriveConstants.AUTO_AIM_SETPOINT_MARGIN,
                DriveConstants.ALIGNMENT_TOLERANCE_KP,
                DriveConstants.ALIGNMENT_TOLERANCE_KD);
        addRequirements(drive);
    }

    public Translation2d getShooterTranslation() {
        Translation2d shooterOffset = new Translation2d(
                Constants.Vision.robotCenterToShooterCenter.getTranslation().getX(),
                Constants.Vision.robotCenterToShooterCenter.getTranslation().getY());
        Translation2d baseTranslation = m_Drive.getPose().getTranslation().plus(shooterOffset);
        Translation2d rotatedTranslation = baseTranslation.rotateAround(
                m_Drive.getPose().getTranslation(), m_Drive.getPose().getRotation());

        return rotatedTranslation;
    }

    //
    public Translation2d getShooterDisplacementFromTarget() {
        return m_Drive.targetShotPose.getTranslation().minus(getShooterTranslation());
    }

    public double getShooterDistanceFromTarget() {
        return Math.hypot(
                getShooterDisplacementFromTarget().getX(),
                getShooterDisplacementFromTarget().getY());
    }

    /*
     * Uses the yaw of the target tag to calculate the angle the shooter needs to
     * turn to
     * to be facing the hub.
     * This is used as a backup in case the global pose is inaccurate or the target
     * tag is not visible.
     * Then this calculates the target angle from 0-360
     */
    public Optional<Double> getTargetShooterAngleFromYaw() {
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
            double shooterTargetAngle = (robotHeading + yaw) % 360;
            return shooterTargetAngle > 0 ? shooterTargetAngle : shooterTargetAngle + 360;
        });
    }

    /*
     * Using the global pose and the target tag pose, calculates the angle the
     * shooter
     * needs to turn to to be facing the hub.
     */
    public Rotation2d getTargetShooterAngle() {
        Translation2d displacement = getShooterDisplacementFromTarget();
        double robotHeading = m_Drive.getRotation().getRadians();
        double angleToTarget = Math.atan2(displacement.getY(), displacement.getX());

        return new Rotation2d(angleToTarget).relativeTo(Rotation2d.k180deg);
    }

    // Debugging
    public double getAngle() {
        Translation2d displacement = getShooterDisplacementFromTarget();
        return Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())) >= 0
                ? Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX()))
                : Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())) + 360;
    }

    public double delta() {
        double robotHeading = m_Drive.getRotation().getRadians();
        return getTargetShooterAngle().getRadians() - robotHeading;
    }

    public BooleanSupplier close() {
        return () -> Math.abs(turnToAngle.delta()) <= DriveConstants.ALIGNMENT_TOLERANCE;
    }

    // Logs out everything
    public void log() {
        // Global Pose Logs
        Logger.recordOutput("Shooter Target Angle", getTargetShooterAngle());
        Logger.recordOutput(
                "targetspeeds", turnToAngle.getTargetSpeeds(m_Drive.getRotation(), getTargetShooterAngle()));
        Logger.recordOutput("Target Close", close());
        Logger.recordOutput("In margin", turnToAngle.isInPidMargin());
        Logger.recordOutput("Hub Pose", m_Drive.targetShotPose);
        Logger.recordOutput("Shooter Target Displacement", getShooterDisplacementFromTarget());
        Logger.recordOutput("Shooter Target Distance", getShooterDistanceFromTarget());
        Logger.recordOutput(
                "Shooter Target YawAngle", getTargetShooterAngleFromYaw().orElse(0.5));
        Logger.recordOutput(
                "Shooter Pose", new Pose2d(getShooterTranslation(), new Rotation2d(Math.toRadians(getAngle()))));
        Logger.recordOutput("Robot Pose Radians", m_Drive.getRotation().getRadians());
    }

    public Command align(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return DriveCommands.joystickDrive(
                        m_Drive,
                        xSupplier,
                        ySupplier,
                        () -> (turnToAngle.getTargetSpeeds(m_Drive.getRotation(), getTargetShooterAngle())
                                / DriveConstants.AUTO_AIM_MAX_ROT_VEL))
                .alongWith(Commands.run(() -> {
                    if (turnToAngle.isInPidMargin()
                            && m_Drive.shooting
                            && xSupplier.getAsDouble() == 0
                            && ySupplier.getAsDouble() == 0) {
                        m_Drive.stopWithX();
                    }
                }));
    }

    public Command align() {
        return Commands.run(() -> {
                    m_Drive.snapToAngle(
                            () -> turnToAngle.getTargetSpeeds(m_Drive.getRotation(), getTargetShooterAngle()));
                })
                .alongWith(Commands.run(() -> {
                    if (turnToAngle.isInPidMargin() && m_Drive.shooting) {
                        m_Drive.stopWithX();
                    }
                }));
    }
}
