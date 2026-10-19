package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.Constants.AprilTag2d;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AlignToTag extends Command {
    private final Drive m_Drive;
    private final Translation2d m_offset;
    private final Supplier<ArrayList<Pose2d>> m_filter;
    private Pose2d targetPose = null;
    private Pose2d transformedTargetPose = null;
    private Pose2d transformedTargetPose2 = null;
    private final Rotation2d m_alignment;
    private boolean finished = false;

    public AlignToTag(
            Vision vision,
            Drive drive,
            Translation2d offset,
            Rotation2d alignment,
            Supplier<ArrayList<Pose2d>> filter) {
        this.m_Drive = drive;
        this.m_offset = offset;
        this.m_filter = filter;
        this.m_alignment = alignment;
    }

    @Override
    public void initialize() {
        targetPose = null;
        transformedTargetPose = null;
        finished = false;
    }

    public Pose2d getClosestReefAprilTag(Pose2d pose) {
        return pose.nearest(m_filter.get());
    }

    public int getTagIDFromPose(Pose2d targetPose) {
        for (AprilTag2d tag : Constants.APRIL_TAG_FIELD_LAYOUT_2D) {
            if (tag.pose.equals(targetPose)) {
                return tag.ID;
            }
        }
        return 0;
    }

    @Override
    public void execute() {
        // m_Vision.getCentermostTag(m_filter).ifPresent((tag) -> {

        Pose2d originalTag = null;

        // int tagID = tag;
        // System.out.println(tagID);

        // Optional<Pose3d> tag3D =
        // Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID);
        // if (!tag3D.isEmpty()){
        // tag2D = new Pose2d(tag3D.get().getX(), tag3D.get().getY(), new
        // Rotation2d(tag3D.get().getRotation().getZ()).plus(m_alignment));
        originalTag = getClosestReefAprilTag(m_Drive.getPose());

        Pose2d tag2D = new Pose2d(
                originalTag.getX(),
                originalTag.getY(),
                originalTag.getRotation().plus(m_alignment));
        targetPose = tag2D;
        transformedTargetPose = tag2D.plus(new Transform2d(m_offset, new Rotation2d()));
        transformedTargetPose2 = tag2D.plus(
                new Transform2d(new Translation2d(m_offset.getX() - 0.1, m_offset.getY()), new Rotation2d()));
        int targetID = getTagIDFromPose(originalTag);
        finished = true;
        m_Drive.setTargetPose(transformedTargetPose2);
        m_Drive.setTargetPose2(transformedTargetPose);
        m_Drive.setTargetTagID(targetID);
        Logger.recordOutput("Odometry/AprilTagTargetPose", targetPose);
        Logger.recordOutput("Odometry/AprilTagTargetID", targetID);
        Logger.recordOutput("Odometry/AprilTagTransformedTargetPose", transformedTargetPose);
        // }
        // });
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
