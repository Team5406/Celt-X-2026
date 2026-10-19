package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;

public class RampUp extends SequentialCommandGroup {
    private IntakePivotSubsystem pivot;
    private SequentialCommandGroup rampUp = new SequentialCommandGroup();

    public RampUp(IntakePivotSubsystem pivot) {
        this.pivot = pivot;
        double lower = IntakePivotConstants.PIVOT_SHOOTING_POSITION_SECOND;
        double upper = IntakePivotConstants.PIVOT_SHOOTING_POSITION;

        while (upper <= IntakePivotConstants.AGITATING_POSITION_FINAL) {
            rampUp.addCommands(pivot.setPosRough(lower).withTimeout(IntakePivotConstants.PIVOT_BOUNCE_TEST_TIMEOUT));
            rampUp.addCommands(pivot.setPosRough(upper).withTimeout(IntakePivotConstants.PIVOT_BOUNCE_TEST_TIMEOUT));
            lower += IntakePivotConstants.AGITATION_INCREMENT;
            upper += IntakePivotConstants.AGITATION_INCREMENT;
        }
    }

    public void stop() {
        CommandScheduler.getInstance().cancel(this);
    }

    public Command rampUp() {
        return rampUp.andThen(new RepeatCommand(pivot.setPosRough(IntakePivotConstants.AGITATING_POSITION_FINAL)
                .withTimeout(IntakePivotConstants.PIVOT_BOUNCE_TEST_TIMEOUT)
                .andThen(pivot.setPosRough(IntakePivotConstants.AGITATING_POSITION_FINAL_SECOND)
                        .withTimeout(IntakePivotConstants.PIVOT_BOUNCE_TEST_TIMEOUT))));
    }
}
