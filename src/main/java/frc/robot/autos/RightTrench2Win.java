package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.commands.RampUp;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class RightTrench2Win extends SequentialCommandGroup {
    public static double turretAdvancePosition = 310; // needed to prevent turret flip
    public static double turretAdvancePosition2 = 180;

    public RightTrench2Win(
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
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("RightTrench1a");
            PathPlannerPath path3 = PathPlannerPath.fromPathFile("RightTrench1cAlt");
            PathPlannerPath path4 = PathPlannerPath.fromPathFile("RightTrench2a");
            addCommands(
                    hood.syncEncoder(),
                    AutoBuilder.resetOdom(path1.getStartingDifferentialPose()),
                    new SequentialCommandGroup(
                            pivot.setEncoder(IntakePivotConstants.PIVOT_ZEROED_UP_POSITION),
                            new ParallelDeadlineGroup(
                                    AutoBuilder.followPath(path1),
                                    new SequentialCommandGroup(
                                            new WaitCommand(0.5),
                                            pivot.activePos(),
                                            new WaitUntilCommand(pivot::pivotActive),
                                            new AutoIntake(rollers, serializer, feeder))),
                            new ParallelDeadlineGroup(
                                    new WaitCommand(3.2),
                                    /*new SequentialCommandGroup(
                                    new InstantCommand(() ->
                                                    drive.setTargetPose(path4.getStartingDifferentialPose()))
                                            .andThen(new DriveToPose(drive)
                                                    .generateCommand(
                                                            false,
                                                            false,
                                                            new PathConstraints(
                                                                    .4, .2, .5 * Math.PI, .5 * Math.PI)))),*/
                                    rollers.stop()
                                            .andThen(rollers.setVelocityOnceOver(1500, pivot)
                                                    .beforeStarting(new WaitCommand(2))),
                                    new RampUp(pivot).rampUp().beforeStarting(new WaitCommand(2)),
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> true)),
                                    new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision)),
                            new ParallelDeadlineGroup(
                                    AutoBuilder.followPath(path4),
                                    pivot.activePos(),
                                    hood.stow(),
                                    flywheel.stop(),
                                    new AutoIntake(rollers, serializer, feeder)),
                            new ParallelCommandGroup(
                                    rollers.stop()
                                            .andThen(rollers.setVelocityOnceOver(1500, pivot)
                                                    .beforeStarting(new WaitCommand(2))),
                                    new RampUp(pivot).rampUp().beforeStarting(new WaitCommand(2)),
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> true)),
                                    new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision))));

        } catch (Exception e) {
            DriverStation.reportError("MASSIVE oppsies lol: Middle" + e.getMessage(), e.getStackTrace());
        }
    }
}
