package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RampUp;
import frc.robot.commands.WaitToAutoShoot;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class LeftTrench2 extends SequentialCommandGroup {
    public static double turretAdvancePosition = 310; // needed to prevent turret flip
    public static double turretAdvancePosition2 = 180;
    public static double rotationSpeed = Math.toRadians(540);
    public static double rotationAccel = Math.toRadians(720);

    public LeftTrench2(
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
            DriveToPose alignment = new DriveToPose(drive);
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("LeftTrench1a");
            // PathPlannerPath path3 = PathPlannerPath.fromPathFile("RightTrench1cAlt");
            PathPlannerPath path4 = PathPlannerPath.fromPathFile("LeftTrench2a");

            // Loads deploy/autos/paths/myPathFile.json
            // Note: .json extension is added automatically
            Path newPath1a = new Path("LeftTrench1a");
            Path newPathTran = new Path("LeftTrenchTran");
            Path newPath2a = new Path("LeftTrench2a");

            // Create path builder
            FollowPath.Builder pathBuilder = new FollowPath.Builder(
                            drive,
                            drive::getPose,
                            drive::getChassisSpeeds,
                            drive::runVelocity,
                            new PIDController(4.0, 0.0, 0.5),
                            new PIDController(5.0, 0.0, 0.1),
                            new PIDController(3.0, 0.0, 0.0))
                    .withDefaultShouldFlip();

            // Then build commands for specific paths:

            addCommands(
                    hood.syncEncoder(),
                    AutoBuilder.resetOdom(newPath1a.getStartPose()),
                    new SequentialCommandGroup(
                            pivot.setEncoder(IntakePivotConstants.PIVOT_ZEROED_UP_POSITION),
                            new ParallelDeadlineGroup(
                                    new SequentialCommandGroup(
                                            pathBuilder.build(newPath1a).withTimeout(8),
                                            new WaitToAutoShoot(vision, drive, 2)),
                                    new SequentialCommandGroup(
                                            new WaitCommand(0.5),
                                            pivot.activePos(),
                                            new WaitUntilCommand(pivot::pivotActive),
                                            new AutoIntake(rollers, serializer, feeder)),
                                    flywheel.setPercentOnce(FlywheelConstants.FLYWHEEL_IDLE_PERCENT)),
                            new ParallelDeadlineGroup(
                                    new WaitCommand(3.2),
                                    rollers.stop()
                                            .andThen(rollers.setVelocityOnceOver(1500, pivot)
                                                    .beforeStarting(new WaitCommand(1))),
                                    new RampUp(pivot).rampUp().beforeStarting(new WaitCommand(1)),
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> true)),
                                    new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision)),
                            new ParallelDeadlineGroup(
                                    new SequentialCommandGroup(
                                            new WaitUntilCommand(pivot::pivotActive), pathBuilder.build(newPath2a)),
                                    pivot.activePos(),
                                    hood.stow(),
                                    flywheel.setPercentOnce(FlywheelConstants.FLYWHEEL_IDLE_PERCENT),
                                    new AutoIntake(rollers, serializer, feeder)),
                            new ParallelCommandGroup(
                                    rollers.stop()
                                            .andThen(rollers.setVelocityOnceOver(1500, pivot)
                                                    .beforeStarting(new WaitCommand(1))),
                                    new RampUp(pivot).rampUp().beforeStarting(new WaitCommand(1)),
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> true)),
                                    new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision))));

        } catch (Exception e) {
            DriverStation.reportError("MASSIVE oppsies lol: Middle" + e.getMessage(), e.getStackTrace());
        }
    }
}
