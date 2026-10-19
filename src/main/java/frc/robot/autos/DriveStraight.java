package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.path.ChoreoClosedLoopTrajectoryFollower;
import frc.lib.path.ChoreoOpenLoopTrajectoryFollower;
import frc.lib.subsystems.drive.Drive;

public class DriveStraight extends SequentialCommandGroup{
    
    public DriveStraight(Drive drive){
        addCommands(
        drive.followTrajectory(new ChoreoOpenLoopTrajectoryFollower("DriveStraight", 0), true),
        drive.followTrajectory(new ChoreoOpenLoopTrajectoryFollower("DriveStraight", 1), false)
        );
    }

}
