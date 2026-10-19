package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.vision.Vision;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AutoUnjamFeeding;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class Middle extends SequentialCommandGroup {
    public Middle(
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
            addCommands(
                    hood.syncEncoder(),
                    new SequentialCommandGroup(
                            pivot.setEncoder(IntakePivotConstants.PIVOT_ZEROED_UP_POSITION),
                            new ParallelCommandGroup(
                                    new AlignToHub(vision, drive).align(),
                                    shooter.spinUp(new Trigger(() -> true)), // Flywheel = 3050, Hood = 22.5
                                    new SequentialCommandGroup(
                                            new WaitCommand(0.5),
                                            new AutoUnjamFeeding(serializer, feeder, flywheel, drive, vision)))));

        } catch (Exception e) {
            DriverStation.reportError("MASSIVE oppsies lol: Middle" + e.getMessage(), e.getStackTrace());
        }
    }
}
