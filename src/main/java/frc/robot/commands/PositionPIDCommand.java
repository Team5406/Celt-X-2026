package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

public class PositionPIDCommand extends Command {
    private Drive drive;
    private Pose2d goalPose;
    private PPHolonomicDriveController mDriveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.5);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.75);

    private PositionPIDCommand(Drive drive, Pose2d goalPose) {
        this.drive = drive;
        this.goalPose = goalPose;

        this.addRequirements(drive);
    }

    private boolean finished() {
        Pose2d diff = drive.getPose().relativeTo(goalPose);

        var rotation =
                MathUtil.isNear(0.0, diff.getRotation().getRotations(), kRotationTolerance.getRotations(), 0.0, 1.0);

        var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

        var speed = drive.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

        return rotation && position && speed;
    }

    public static Command generateCommand(Drive drive, Pose2d goalPose, Time timeout) {
        return new PositionPIDCommand(drive, goalPose).withTimeout(timeout).finallyDo(() -> {
            drive.runVelocity(new ChassisSpeeds(0, 0, 0));
            // drive.lockModules();
        });
    }

    public static Command generateCommand(
            Drive drive, IntSupplier tagID, Time timeout, Translation2d m_offset, Rotation2d m_alignment) {
        Optional<Pose3d> tag3D = Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID.getAsInt());
        Pose2d tag2D = new Pose2d(
                tag3D.get().getX(),
                tag3D.get().getY(),
                new Rotation2d(tag3D.get().getRotation().getZ()).plus(m_alignment));
        Pose2d goalPose = tag2D.plus(new Transform2d(m_offset, new Rotation2d()));

        Logger.recordOutput("Odometry/AprilTagTargetPose", tag2D);
        Logger.recordOutput("Odometry/AprilTagTransformedTargetPose", goalPose);

        return new PositionPIDCommand(drive, goalPose).withTimeout(timeout).finallyDo(() -> {
            drive.runVelocity(new ChassisSpeeds(0, 0, 0));
            // drive.lockModules();
        });
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        drive.runVelocity(mDriveController.calculateRobotRelativeSpeeds(drive.getPose(), goalState));
    }

    @Override
    public boolean isFinished() {
        return finished();
    }
}
