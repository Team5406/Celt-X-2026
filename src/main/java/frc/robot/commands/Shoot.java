package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.spindexer.feeder.SpindexerFeederSubsystem;
import frc.robot.subsystems.spindexer.serializer.SerializerSubsystem;

public class Shoot extends SequentialCommandGroup {

    public Shoot(Drive drive, SpindexerFeederSubsystem feeder, SerializerSubsystem serializer) {
        addCommands(
                feeder.setTargetVelocity(),
                new ParallelCommandGroup(
                        serializer.setSpinupVelocity()

                        // Add !intaking -> pivot = shooting
                        ));
    }
}
