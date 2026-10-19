/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.subsystems.vision.VisionFilterParameters;
import frc.lib.util.Sysid;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.simulation.SimCameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Add this ShooterHardware class
    public static class ShooterHardware {
        // These are the constants you need for SimpleMotorFeedforward
        public static final double SHOOTER_KS = 0.1; // Example value for static friction
        public static final double SHOOTER_KV = 0.2; // Example value for velocity gain
        public static final double SHOOTER_KA = 0.3; // Example value for acceleration gain

        // Other existing nested classes, like Field, etc.
        public static class Field {
            public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            public static final Distance FIELD_WIDTH = Meters.of(8.2042);
            public static final Distance FIELD_LENGTH = Meters.of(16.5412);
        }
    }

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Sysid SYSID_MODE = Sysid.NOTSYSID;
    public static final double LOOPER_DT = 0.2;
    public static final double CONTROLLER_DEADBAND = 0.04481;

    public static class EjectorHardware {
        public static final double EJECTOR_CONVERSION_FACTOR = 1.0;
        public static final int EJECTOR_MOTOR_ID = 9;
        public static final double EJECTOR_TARGET_SPEED = 0.5;
    }

    public static class HardwareMap {
        public static final int PDH_ID = 1;
    }

    public static class Vision {
        /**
         * Transform3d object that represents the translation and rotation from the robot centre to the
         * camera
         */
        // global pose
        public static final Transform3d robotToCameraFront = new Transform3d(
                new Translation3d(0.133, -0.178, 0.20), // in meters
                new Rotation3d(Math.toRadians(0), Math.toRadians(-17), Math.toRadians(0))); // FIXME ANGLE

        // alignment
        public static final Transform3d robotToCameraBack = new Transform3d(
                new Translation3d(0.133, -0.102, 0.20), // in meters
                new Rotation3d(Math.toRadians(0), Math.toRadians(-17), Math.toRadians(0)));

        public static final SimCameraProperties CAMERA_SIM_PROPERTIES = new SimCameraProperties();

        // Setup the camera sim properties
        static {
            CAMERA_SIM_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(100));
            CAMERA_SIM_PROPERTIES.setCalibError(0.25, 0.08);
            CAMERA_SIM_PROPERTIES.setFPS(20);
            CAMERA_SIM_PROPERTIES.setAvgLatencyMs(35);
            CAMERA_SIM_PROPERTIES.setLatencyStdDevMs(5);
        }

        public static final String GLOBAL_POSE_CAMERA = "Front Camera";
        public static final String ALIGNMENT_POSE_CAMERA = "FrontLeft Camera";

        public static final int[] BLUE_REEF_IDS = {17, 18, 19, 20, 21, 22};

        public static final int[] RED_REEF_IDS = {6, 7, 8, 9, 10, 11};

        public static final int BLUE_CORAL_ID_LEFT = 13;
        public static final int BLUE_CORAL_ID_RIGHT = 12;

        public static final int RED_CORAL_ID_LEFT = 1;
        public static final int RED_CORAL_ID_RIGHT = 2;

        public static final int RED_AUTO_PROCESSOR_FIRST_ID = 9;
        public static final int RED_AUTO_PROCESSOR_SECOND_ID = 8;
        public static final int BLUE_AUTO_PROCESSOR_FIRST_ID = 22;
        public static final int BLUE_AUTO_PROCESSOR_SECOND_ID = 17;

        public static final int RED_AUTO_FIRST_ID = 11;
        public static final int RED_AUTO_SECOND_ID = 6;
        public static final int BLUE_AUTO_FIRST_ID = 20;
        public static final int BLUE_AUTO_SECOND_ID = 19;

        public static final int RED_AUTO_MIDDLE_ID = 10;
        public static final int BLUE_AUTO_MIDDLE_ID = 21;

        public static final int RED_BARGE_RIGHT_ID = 5;
        public static final int RED_BARGE_LEFT_ID = 15;
        public static final int BLUE_BARGE_LEFT_ID = 14;
        public static final int BLUE_BARGE_RIGHT_ID = 4;

        public static final Translation2d leftReefOffset =
                new Translation2d(Meter.convertFrom(-16.25, Inch), Meter.convertFrom(11.5, Inch));
        public static final Translation2d rightReefOffset =
                new Translation2d(Meter.convertFrom(-16.26, Inch), Meter.convertFrom(-1, Inch));
        public static final Translation2d leftCoralOffset =
                new Translation2d(Meter.convertFrom(17, Inch), Meter.convertFrom(10, Inch));
        public static final Translation2d rightCoralOffset =
                new Translation2d(Meter.convertFrom(17, Inch), Meter.convertFrom(-10, Inch));
        public static final Translation2d bargeOffset =
                new Translation2d(Meter.convertFrom(-27, Inch), Meter.convertFrom(0, Inch));

        public static final double LEFT_REEF_TARGET_YAW = 36.7;
        public static final double RIGHT_REEF_TARGET_YAW = 10.7;
        public static final double FAKE_TARGET_YAW = 0;

        public static final double singleTagOffset = 0.1;

        public static final VisionFilterParameters VISION_FILTER_PARAMETERS = new VisionFilterParameters(
                0.5,
                1.5,
                Millimeters.of(165),
                0.5,
                Rotation2d.fromDegrees(60),
                Field.FIELD_WIDTH,
                Field.FIELD_LENGTH,
                Centimeter.of(15));

        public static final double CLOSE_TAG_THRESHOLD = 1.0;
    }

    public static ArrayList<Pose2d> BLUE_REEF_TAG_POSES = new ArrayList<>();
    public static ArrayList<Pose2d> RED_REEF_TAG_POSES = new ArrayList<>();
    public static ArrayList<AprilTag2d> APRIL_TAG_FIELD_LAYOUT_2D = new ArrayList<>();

    public static void bluePoses() {
        for (int i : Constants.Vision.BLUE_REEF_IDS) {
            Optional<Pose3d> x = Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(i);
            BLUE_REEF_TAG_POSES.add(new Pose2d(
                    x.get().getMeasureX(),
                    x.get().getMeasureY(),
                    x.get().getRotation().toRotation2d()));
        }
    }

    public static void redPoses() {
        for (int i : Constants.Vision.RED_REEF_IDS) {
            Optional<Pose3d> x = Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(i);
            RED_REEF_TAG_POSES.add(new Pose2d(
                    x.get().getMeasureX(),
                    x.get().getMeasureY(),
                    x.get().getRotation().toRotation2d()));
        }
    }

    public static void aprilTagFieldLayout2d() {
        for (AprilTag tag : Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTags()) {
            Pose2d tagPose = new Pose2d(
                    tag.pose.getMeasureX(),
                    tag.pose.getMeasureY(),
                    tag.pose.getRotation().toRotation2d());
            APRIL_TAG_FIELD_LAYOUT_2D.add(new AprilTag2d(tag.ID, tagPose));
        }
    }

    public static class Field {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Distance FIELD_WIDTH = Meters.of(8.2042);
        public static final Distance FIELD_LENGTH = Meters.of(16.5412);
    }

    public static class AprilTag2d {
        public int ID;
        public Pose2d pose;

        public AprilTag2d(int ID, Pose2d pose) {
            this.ID = ID;
            this.pose = pose;
        }
    }
}
