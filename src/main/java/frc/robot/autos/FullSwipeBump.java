package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.vision.Vision;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoUnjamFeeding;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class FullSwipeBump extends SequentialCommandGroup {
    public FullSwipeBump(
            Drive drive,
            HoodSubsystem hood,
            FlywheelSubsystem flywheel,
            FeederSubsystem feeder,
            IntakeRollerSubsystem rollers,
            IntakePivotSubsystem pivot,
            RollerFloorSubsystem serializer,
            Vision vision,
            ShooterSubsystem shooter) {
        try {
            // Loads deploy/autos/paths/myPathFile.json
            // Note: .json extension is added automatically
            Path path1 = new Path("FullSwipeBump");
            Path path2 = new Path("DepotDrive");

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

            addCommands(
                    hood.syncEncoder(),
                    AutoBuilder.resetOdom(path1.getStartPose()),
                    new SequentialCommandGroup(
                            pivot.setEncoder(IntakePivotConstants.PIVOT_ZEROED_UP_POSITION),
                            new WaitCommand(1),
                            pivot.activePos(),
                            new WaitCommand(2),
                            new WaitUntilCommand(pivot::pivotActive),
                            new ParallelDeadlineGroup(
                                    pathBuilder.build(path1),
                                    new SequentialCommandGroup(
                                            new AutoIntake(rollers, serializer, feeder).until(drive::allianceZone),
                                            rollers.setVelocityOnceOver(-1500, pivot),
                                            new WaitCommand(1),
                                            new AutoIntake(rollers, serializer, feeder))),
                            new ParallelDeadlineGroup(
                                    new WaitCommand(3),
                                    rollers.stop(),
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> false)),
                                    new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision)),
                            new ParallelDeadlineGroup(
                                    pathBuilder.build(path2), new AutoIntake(rollers, serializer, feeder)),
                            new ParallelCommandGroup(
                                    rollers.setVelocityOnceOver(1500, pivot),
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> false)),
                                    new SequentialCommandGroup(
                                            new WaitCommand(1),
                                            new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision)))));

        } catch (Exception e) {
            DriverStation.reportError("MASSIVE oppsies lol: Middle" + e.getMessage(), e.getStackTrace());
        }
    }
}
