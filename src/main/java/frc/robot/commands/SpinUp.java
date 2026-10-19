package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class SpinUp extends SequentialCommandGroup {

    public SpinUp(HoodSubsystem hood, FlywheelSubsystem flywheel, ShooterSubsystem shooter) {
        addCommands(
                hood.setPosition(() -> shooter.getHoodAngle()), flywheel.setVelocity(() -> shooter.getFlywheelSpeed()));
    }
}
