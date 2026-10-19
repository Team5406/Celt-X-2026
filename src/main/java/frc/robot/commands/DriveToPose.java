package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private final Drive drive;
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(5.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.5);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(3);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.75);
    public static final PathConstraints kPathConstraints =
            new PathConstraints(3.5, 3, 1 / 2 * Math.PI, 1 * Math.PI); // The

    // constraints
    // for
    // this
    // path.

    public DriveToPose(Drive drive) {
        this.drive = drive;
    }

    public Command generateCommand() {
        return Commands.defer(
                () -> {
                    return getPathFromPose(drive, drive.getTargetPose());
                },
                Set.of(drive));
    }

    public Command generateCommand(boolean invert) {
        return Commands.defer(
                () -> {
                    return getPathFromPose(drive, drive.getTargetPose(), invert);
                },
                Set.of(drive));
    }

    public Command generateCommand(boolean invert, boolean finalCorrection) {
        return Commands.defer(
                () -> {
                    return getPathFromPose(drive, drive.getTargetPose(), invert, finalCorrection, false);
                },
                Set.of(drive));
    }

    public Command generateCommand(boolean invert, boolean finalCorrection, boolean noInvert) {
        return Commands.defer(
                () -> {
                    return getPathFromPose(drive, drive.getTargetPose(), invert, finalCorrection, noInvert);
                },
                Set.of(drive));
    }

    private static Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target, Drive drive, boolean invert) {
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            return invert ? drive.getRotation().rotateBy(Rotation2d.k180deg) : drive.getRotation();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private static LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    public static Command getPathFromPose(Drive drive, Pose2d pose) {
        return getPathFromPose(drive, pose, false);
    }

    public static Command getPathFromPose(Drive drive, Pose2d pose, boolean invert) {
        return getPathFromPose(drive, pose, invert, true, false);
    }

    public static Command getPathFromPose(
            Drive drive, Pose2d pose, boolean invert, boolean finalCorrection, boolean noInvert) {
        // System.out.println("called drivetopose");
        // get waypoints from current pose and target pose distance
        // System.out.println("didn't crash getting field velocity");
        ChassisSpeeds fieldChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
        // System.out.println("didnt crash getting chassisspeeds");

        Pose2d adjustedPose;
        if (invert && !noInvert) {
            adjustedPose = new Pose2d(pose.getTranslation(), pose.getRotation().rotateBy(Rotation2d.k180deg));
        } else {
            adjustedPose = pose;
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                        drive.getPose().getTranslation(),
                        getPathVelocityHeading(fieldChassisSpeeds, adjustedPose, drive, invert)),
                adjustedPose);

        if (waypoints.size() != 0) {
            // System.out.println("waypoints size " + waypoints.size());
            if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
                return Commands.sequence(
                        Commands.print("start position PID loop"),
                        PositionPIDCommand.generateCommand(drive, drive.getTargetPose2(), kAutoAlignAdjustTimeout),
                        Commands.print("end position PID loop"));
            }
            ;

        } else {
            System.out.println("waypoints size " + waypoints.size());
        }

        try {
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    kPathConstraints,
                    new IdealStartingState(getVelocityMagnitude(fieldChassisSpeeds), drive.getRotation()),
                    // new IdealStartingState(drive.getSpeed(), drive.getRotation()),
                    new GoalEndState(finalCorrection ? 0.5 : 0, pose.getRotation()));
            Logger.recordOutput(
                    "Odometry/PathHeading", getPathVelocityHeading(fieldChassisSpeeds, adjustedPose, drive, invert));
            path.preventFlipping = true;
            // return AutoBuilder.followPath(path);
            return AutoBuilder.followPath(path)
                    .andThen(new ConditionalCommand(
                            PositionPIDCommand.generateCommand(drive, drive.getTargetPose2(), kAutoAlignAdjustTimeout),
                            Commands.none(),
                            () -> finalCorrection))
                    .finallyDo(() -> {
                        ChassisSpeeds vel =
                                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
                        DriveCommands.setLastVelocity(new Translation2d(
                                vel.vxMetersPerSecond / DriveConstants.MAX_TELEOP_SPEED,
                                vel.vyMetersPerSecond / DriveConstants.MAX_TELEOP_SPEED));
                    });
        } catch (Exception e) {
            System.out.println(e.getMessage());
            System.out.println(e.getStackTrace());
            return Commands.none();
        }
    }
}
