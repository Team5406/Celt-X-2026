package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;

public class StowIntake extends ParallelCommandGroup {

    public StowIntake(IntakePivotSubsystem pivot, IntakeRollerSubsystem roller) {
        addCommands(roller.stop(), pivot.stow());
    }
}
