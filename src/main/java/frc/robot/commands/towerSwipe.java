package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class towerSwipe extends SequentialCommandGroup {
    //     public static edu.wpi.first.math.geometry.Pose2d mirrorFieldPose(edu.wpi.first.math.geometry.Pose2d pose) {
    //         return pose;
    //     }

    public towerSwipe(
            Drive drive,
            IntakeRollerSubsystem rollers,
            IntakePivotSubsystem pivot,
            DriveToPose alignmentCommand,
            Path path) {
        try {
            // PathPlannerPath towerSwipe = PathPlannerPath.fromPathFile("TowerSwipe");
            // PathPlannerPath towerSwipe2 = PathPlannerPath.fromPathFile("TowerSwipe2");

            Path towerSwipeDown = new Path("TowerSwipeDown");
            Path towerSwipeUp = new Path("TowerSwipeUp");
            Path towerSwipeUpOpp = new Path("TowerSwipeUpOpp");
            Path towerSwipeDownOpp = new Path("TowerSwipeDownOpp");

            // Create path builder
            FollowPath.Builder pathBuilder = new FollowPath.Builder(
                            drive,
                            drive::getPose,
                            drive::getChassisSpeeds,
                            drive::runVelocity,
                            new PIDController(3.0, 0.0, 0.0),
                            new PIDController(3.0, 0.0, 0.0),
                            new PIDController(3.0, 0.0, 0.0))
                    .withDefaultShouldFlip();

            AtomicReference<Path> chosenPathRef = new AtomicReference<>();

            addCommands(
                    new InstantCommand(() -> {
                                Pose2d currentPose = drive.getPose();
                                boolean isBlue = DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Blue;
                                boolean isUpperHalf =
                                        isBlue ? currentPose.getY() >= 4.375 : currentPose.getY() <= 4.375;
                                boolean isOpposite = isBlue ? currentPose.getX() >= 7.860 : currentPose.getX() <= 7.860;

                                Path chosenPath;
                                if (!isOpposite && isUpperHalf) {
                                    chosenPath = towerSwipeUp;
                                } else if (!isOpposite && !isUpperHalf) {
                                    chosenPath = towerSwipeDown;
                                } else if (isOpposite && isUpperHalf) {
                                    chosenPath = towerSwipeUpOpp;
                                } else {
                                    chosenPath = towerSwipeDownOpp;
                                }

                                chosenPathRef.set(chosenPath);
                                Pose2d startPose = chosenPath.getStartPose();

                                Pose2d targetPose = isBlue ? startPose : FlippingUtil.flipFieldPose(startPose);
                                drive.setTargetPose(targetPose);

                                Logger.recordOutput("TowerSwipe/targetPose", targetPose);
                                Logger.recordOutput("TowerSwipe/isUpperHalf", isUpperHalf);
                                Logger.recordOutput("TowerSwipe/isOpposite", isOpposite);
                                Logger.recordOutput("TowerSwipe/isBlue", isBlue);

                                // Logger.recordOutput(
                                //         "TowerSwipe/endPose",
                                //         chosenPath
                                //                 .getPathElement()
                                //                 .get(chosenPath.getStartPose().size() - 1));
                            })
                            .andThen(alignmentCommand.generateCommand(false, false)),
                    pivot.activePos(),
                    new WaitUntilCommand(pivot::pivotActive),
                    rollers.setVelocityOnce(2500, pivot),
                    new DeferredCommand(() -> pathBuilder.build(chosenPathRef.get()), Set.of(drive)),
                    rollers.setVelocityOnce(0, pivot));

        } catch (Exception e) {
            DriverStation.reportError("MASSIVE oppsies, lol: Tower" + e.getMessage(), e.getStackTrace());
        }
    }
}
