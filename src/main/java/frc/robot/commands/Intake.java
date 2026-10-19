package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;

public class Intake extends SequentialCommandGroup {

    public Intake(IntakeRollerSubsystem rollers, IntakePivotSubsystem pivot) {
        addCommands(new SequentialCommandGroup(pivot.activePosition(), new WaitCommand(0.5), rollers.intake()));
    }
}
