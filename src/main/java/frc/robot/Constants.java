/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.subsystems.vision.VisionFilterParameters;
import frc.lib.util.Sysid;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import org.photonvision.simulation.SimCameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Pose2d BLUE_HUB =
            new Pose2d(Units.inchesToMeters(182.1), Units.inchesToMeters(158.7), new Rotation2d(Math.toRadians(90)));
    public static final Pose2d RED_HUB =
            new Pose2d(Units.inchesToMeters(469.0), Units.inchesToMeters(158.7), new Rotation2d(Math.toRadians(90)));
    public static ArrayList<Pose2d> BLUE_SNOWBLOW_POSES = new ArrayList<>(Arrays.asList(
            new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(220.0), new Rotation2d(Math.toRadians(90))),
            new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(100.0), new Rotation2d(Math.toRadians(90)))));

    public static final ArrayList<Pose2d> RED_SNOWBLOW_POSES = new ArrayList<>(Arrays.asList(
            new Pose2d(Units.inchesToMeters(551.1), Units.inchesToMeters(220.0), new Rotation2d(Math.toRadians(90))),
            new Pose2d(Units.inchesToMeters(551.1), Units.inchesToMeters(100.0), new Rotation2d(Math.toRadians(90)))));

    // Add this ShooterHardware class
    public static class ShooterHardware {
        // These are the constants you need for SimpleMotorFeedforward
        public static final double SHOOTER_KS = 0.1; // Example value for static friction
        public static final double SHOOTER_KV = 0.2; // Example value for velocity gain
        public static final double SHOOTER_KA = 0.3; // Example value for acceleration gain
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
         * Transform3d object that represents the translation and rotation from the
         * robot centre to the
         * camera
         */
        // back global pose
        public static final Transform3d robotToCameraBack = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(-11.637),
                        Units.inchesToMeters(-2.845e-7),
                        Units.inchesToMeters(14.248189)), // in meters FIX
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(180))); // FIXME
        // left global pose
        public static final Transform3d robotToCameraLeft = new Transform3d(
                new Translation3d(-0.178, 0.318, 0.226), // in meters FIX
                new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(90))); // FIXME
        // right global pose
        public static final Transform3d robotToCameraRight = new Transform3d(
                new Translation3d(-0.178, -0.318, 0.226), // in meters FIX
                new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(270))); // FIXME

        // shooter center pose
        public static final Transform3d robotCenterToShooterCenter = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(-4.699939),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(17.82000)), // in meters FIX
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))); // FIXME

        public static final SimCameraProperties CAMERA_SIM_PROPERTIES = new SimCameraProperties();

        // Setup the camera sim properties
        static {
            CAMERA_SIM_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(100));
            CAMERA_SIM_PROPERTIES.setCalibError(0.25, 0.08);
            CAMERA_SIM_PROPERTIES.setFPS(20);
            CAMERA_SIM_PROPERTIES.setAvgLatencyMs(35);
            CAMERA_SIM_PROPERTIES.setLatencyStdDevMs(5);
        }

        public static final String BACK_GLOBAL_POSE_CAMERA_NAME = "backCam";
        public static final String LEFT_GLOBAL_POSE_CAMERA_NAME = "leftCam";
        public static final String RIGHT_GLOBAL_POSE_CAMERA_NAME = "rightCam";

        public static final Set<Integer> BLUE_HUB_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

        public static final Set<Integer> RED_HUB_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

        public static final Integer[] RED_TOWER_IDS = {15, 16};
        public static final Integer[] BLUE_TOWER_IDS = {31, 32};

        // Offsets from the center of the hub to the front, right, and left tags
        // respectively
        public static final Translation2d hubFrontOffset =
                new Translation2d(Meter.convertFrom(-16.25, Inch), Meter.convertFrom(11.5, Inch));
        public static final Translation2d hubRightOffset =
                new Translation2d(Meter.convertFrom(-16.26, Inch), Meter.convertFrom(-1, Inch));
        public static final Translation2d hubLeftOffset =
                new Translation2d(Meter.convertFrom(17, Inch), Meter.convertFrom(10, Inch));

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

    public static ArrayList<Pose2d> BLUE_HUB_TAG_POSES = new ArrayList<>();
    public static ArrayList<Pose2d> RED_HUB_TAG_POSES = new ArrayList<>();
    public static ArrayList<AprilTag2d> APRIL_TAG_FIELD_LAYOUT_2D = new ArrayList<>();

    public static void bluePoses() {
        for (int i : Constants.Vision.BLUE_HUB_IDS) {
            Optional<Pose3d> x = Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(i);
            BLUE_HUB_TAG_POSES.add(new Pose2d(
                    x.get().getMeasureX(),
                    x.get().getMeasureY(),
                    x.get().getRotation().toRotation2d()));
        }
    }

    public static void redPoses() {
        for (int i : Constants.Vision.RED_HUB_IDS) {
            Optional<Pose3d> x = Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(i);
            RED_HUB_TAG_POSES.add(new Pose2d(
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
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static final Distance FIELD_WIDTH = Meters.of(8.2042);
        public static final Distance FIELD_LENGTH = Meters.of(16.5412);
        public static final double EDGE_OF_BLUE_ALLIANCE_ZONE = Units.inchesToMeters(158.6);
        public static final double EDGE_OF_RED_ALLIANCE_ZONE = Units.inchesToMeters(492.6);
        public static final double TRENCH_THRESHOLD_BLUE = Units.inchesToMeters(205.611);
        public static final double TRENCH_THRESHOLD_RED = Units.inchesToMeters(444.6);
    }

    public static class AprilTag2d {
        public int ID;
        public Pose2d pose;

        public AprilTag2d(int ID, Pose2d pose) {
            this.ID = ID;
            this.pose = pose;
        }
    }

    public static final double AUTO_WAIT_ROLL_TOLERANCE = 2.0;
    public static final double AUTO_WAIT_PITCH_TOLERANCE = 2.0;
    public static final int MITOCANDRIA_ID = 0;
}
