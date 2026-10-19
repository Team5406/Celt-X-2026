package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.spindexer.feeder.FeederConstants;
import frc.robot.subsystems.spindexer.feeder.SpindexerFeederSubsystem;
import frc.robot.subsystems.spindexer.serializer.SerializerConstants;
import frc.robot.subsystems.spindexer.serializer.SerializerSubsystem;

public class Feeding extends ParallelCommandGroup {
    public Feeding(SpindexerFeederSubsystem feeder, SerializerSubsystem serializer) {
        addCommands(
                feeder.setVelocity(FeederConstants.FEEDER_MAX_VELOCITY),
                serializer.setVelocity(SerializerConstants.SERIALIZER_VELOCITY));
    }
}
