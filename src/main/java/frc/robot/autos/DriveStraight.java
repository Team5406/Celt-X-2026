package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveStraight extends SequentialCommandGroup {

    public DriveStraight() {
        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("DriveStraightMiddle");

            addCommands(AutoBuilder.resetOdom(path1.getStartingDifferentialPose()), AutoBuilder.followPath(path1));

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
    }
}
