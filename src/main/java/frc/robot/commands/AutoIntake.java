package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer.feeder.FeederConstants;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorConstants;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;

public class AutoIntake extends ParallelCommandGroup {

    public AutoIntake(IntakeRollerSubsystem roller, RollerFloorSubsystem floor, FeederSubsystem feeder) {
        addCommands(
                roller.setVelocityAuto(3000),
                feeder.setVelocity(FeederConstants.FEEDER_INTAKING_VELOCITY),
                floor.setVelocity(RollerFloorConstants.FLOOR_INTAKING_VELOCITY));
    }
}
