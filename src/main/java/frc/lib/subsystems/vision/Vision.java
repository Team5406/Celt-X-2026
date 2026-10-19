/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.CouldNotGetException;
import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    VisionIO[] visionIOs; // Array of vision IOs representing the different cameras
    VisionInputsAutoLogged[] inputs;
    VisionFilterParameters filterParameters;
    Pose2d actualRobotPose = new Pose2d();
    private MitoCANdria mito = new MitoCANdria(Constants.MITOCANDRIA_ID);
    /**
     * Creates a new Vision subsystem
     *
     * @param ios the vision IOs representing the different cameras
     */
    public Vision(VisionFilterParameters filterParameters, VisionIO... ios) {
        super("Vision");

        this.filterParameters = filterParameters;

        visionIOs = ios;
        // Create the array of vision inputs
        inputs = new VisionInputsAutoLogged[visionIOs.length];
        for (int i = 0; i < visionIOs.length; i++) {
            inputs[i] = new VisionInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update each io, which represents the different cameras
        for (int i = 0; i < visionIOs.length; i++) {
            // Update the inputs
            visionIOs[i].updateInputs(inputs[i]);
            visionIOs[i].setRobotRotation(actualRobotPose.getRotation());

            Logger.processInputs("Vision/" + inputs[i].cameraName + " (" + i + ")", inputs[i]);
        }
        try {
            logMito();
        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    public void logMito() throws CouldNotGetException {

        mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                .ifPresent(current -> Logger.recordOutput("MitoCANdria 5VA Current  ", current));
        mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                .ifPresent(current -> Logger.recordOutput("MitoCANdria 5VB Current  ", current));
        mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                .ifPresent(voltage -> Logger.recordOutput("MitoCANdria 5VA Voltage  ", voltage));
        mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                .ifPresent(voltage -> Logger.recordOutput("MitoCANdria 5VB Voltage  ", voltage));
        mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                .ifPresent(enabled -> Logger.recordOutput("MitoCANdria 5VA Enabled  ", enabled));
        mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                .ifPresent(enabled -> Logger.recordOutput("MitoCANdria 5VB Enabled  ", enabled));
    }

    public Command processVision(Supplier<Pose2d> robotPoseSupplier, Consumer<VisionMeasurement> measurementConsumer) {
        return Commands.run(
                () -> {
                    // First update the actual robot pose
                    actualRobotPose = robotPoseSupplier.get();

                    // Cycle through all cameras
                    for (int i = 0; i < visionIOs.length; i++) {
                        // Check for results, the hasResults is also false if there is something
                        // wrong with the camera object
                        if (!inputs[i].hasResults) {
                            continue;
                        }

                        // If ambiguity ratio is too high, continue to the next IO
                        if (inputs[i].ambiguityRatio > filterParameters.maxAmbiguityRatio()) {
                            continue;
                        }

                        // Select the pose closest to the actual robot pos
                        Pose3d selectedPose = selectClosestPose(inputs[i].fieldSpaceRobotPoses, actualRobotPose);
                        if (selectedPose == null) {
                            continue;
                        }

                        // Determine the standard deviation of the measurement using the distance
                        // The distance is estimated based on the tag area in percent (0-100)
                        double tagDistance = calculateAverageTagDistance(inputs[i].tagAreas);

                        Matrix<N3, N1> stdDevMat =
                                determineStandardDeviation(tagDistance, inputs[i].fieldSpaceRobotPoses.length > 1);

                        // Retrieve the timestamp of the measurement
                        double timestamp = inputs[i].timeStamp;

                        // Log the accepted pose
                        Logger.recordOutput(
                                "Vision/Accepted Poses/" + inputs[i].cameraName + " (" + i + ")", selectedPose);
                        // Add the measurement to the queue
                        measurementConsumer.accept(
                                new VisionMeasurement(selectedPose.toPose2d(), timestamp, stdDevMat));
                    }
                },
                this);
    }

    private Pose3d selectClosestPose(Pose3d[] fieldSpaceRobotPoses, Pose2d actualRobotPose) {
        // Get the distance closest to the current robot pose if multiple poses are
        // determined for a
        // camera
        double minDistance = Double.POSITIVE_INFINITY;
        Pose3d selectedPose = null;
        Pose3d actualRobotPose3d = new Pose3d(actualRobotPose);

        for (Pose3d fieldSpaceRobotPose : fieldSpaceRobotPoses) {
            double distance = fieldSpaceRobotPose.getTranslation().getDistance(actualRobotPose3d.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                selectedPose = fieldSpaceRobotPose;
            }
        }
        return selectedPose;
    }

    private boolean outsideFieldBounds(Pose3d selectedPose) {
        return selectedPose.getTranslation().getX() < 0
                || selectedPose.getTranslation().getX()
                        > filterParameters.fieldLength().in(Meters)
                || selectedPose.getTranslation().getY() < 0
                || selectedPose.getTranslation().getY()
                        > filterParameters.fieldWidth().in(Meters)
                || selectedPose.getTranslation().getZ()
                        < -filterParameters.zMargin().in(Meters)
                || selectedPose.getTranslation().getZ()
                        > filterParameters.zMargin().in(Meters);
    }

    public Optional<Transform3d> getTargetToRobotTranslation(int tagID) {

        // PhotonTrackedTarget tag = visionIOs[0].getBestTag();
        Optional<Transform3d> targetToRobotTranslation = Optional.empty();
        for (int j = 0; j < inputs.length; j++) {
            if (inputs[j].useGlobal == false) {
                if (inputs[j].hasResults) {
                    for (int i = 0; i < inputs[j].visibleTagIDs.length; i++) {
                        if (inputs[j].visibleTagIDs[i] == tagID) {
                            targetToRobotTranslation = Optional.of(inputs[j].targetToRobotTranslation);
                        }
                    }
                }
            }
        }
        return targetToRobotTranslation;
    }

    private double calculateAverageTagDistance(double[] tagAreas) {
        double averageTagArea = 0;
        for (double area : tagAreas) {
            averageTagArea += area;
        }
        averageTagArea /= tagAreas.length;
        // Estimate of distance when tag fills up 100 percent of the screen
        double minTagDistance = filterParameters.aprilTagWidth().in(Meters)
                / (2 * Math.tan(filterParameters.estimatedFOV().getRadians() / 2));
        // Estimate the tag distance in meters
        return (1 / Math.sqrt(averageTagArea / 100)) * minTagDistance;
    }

    private Matrix<N3, N1> determineStandardDeviation(double tagDistance, boolean isMultiPose) {
        // Determine the standard deviation of the measurement
        double xyStdDev;
        double rotStdDev;

        if (isMultiPose) {
            // Multi tag result
            xyStdDev = filterParameters.xyStandardDevBase();
            rotStdDev = filterParameters.rotStandardDevBase();
        } else {
            // Single tag result
            // Determine the corresponding standard deviation for this result
            // When the distance is 1 meter, the standard deviation is the base value
            if (tagDistance < 0.5) {
                xyStdDev = filterParameters.xyStandardDevBase() * Math.pow(tagDistance, 4);
            } else {
                xyStdDev = filterParameters.xyStandardDevBase() * Math.pow(tagDistance, 2);
            }
            rotStdDev = 1e4; // Don't use rotation result when only one tag is visible
        }
        return MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, rotStdDev);
    }

    public Optional<Integer> getCentermostTag(Set<Integer> filter) {
        // PhotonTrackedTarget tag = visionIOs[0].getBestTag();
        double smallestYaw = Double.NEGATIVE_INFINITY;
        Optional<Integer> closestTarget = Optional.empty();

        for (int j = 0; j < inputs.length; j++) {
            for (int i = 0; i < inputs[j].visibleTagYaws.length; i++) {
                if (filter.contains(inputs[j].visibleTagIDs[i])) {
                    if (inputs[j].visibleTagYaws[i] > smallestYaw) {
                        smallestYaw = inputs[j].visibleTagYaws[i];
                        closestTarget = Optional.of(inputs[j].visibleTagIDs[i]);
                    }
                }
            }
        }

        return closestTarget;
    }

    /*
     * For each camera in inputs, check the number of visible tags and add it to an array
     * Return the array
     */
    public boolean canSeeTags(int minNumTags) {
        for (int i = 0; i < inputs.length; i++) { // For each camera
            if (inputs[i].visibleTagIDs.length >= minNumTags) { // IF we can see X or greater cameras
                return true;
            }
        }

        return false;
    }

    public Optional<Double> getTagYaw(int tagID) {
        // PhotonTrackedTarget tag = visionIOs[0].getBestTag();
        Optional<Double> targetYaw = Optional.empty();
        for (int j = 0; j < inputs.length; j++) {
            if (inputs[j].useGlobal == false) {
                if (inputs[j].hasResults) {
                    for (int i = 0; i < inputs[j].visibleTagIDs.length; i++) {
                        if (inputs[j].visibleTagIDs[i] == tagID) {
                            targetYaw = Optional.of(inputs[j].visibleTagYaws[i]);
                        }
                    }
                }
            }
        }
        return targetYaw;
    }

    public Optional<Double> getTagPitch(int tagID) {
        // PhotonTrackedTarget tag = visionIOs[0].getBestTag();
        Optional<Double> targetPitch = Optional.empty();
        for (int j = 0; j < inputs.length; j++) {
            if (inputs[j].useGlobal == false) {
                if (inputs[j].hasResults) {
                    for (int i = 0; i < inputs[j].visibleTagIDs.length; i++) {
                        if (inputs[j].visibleTagIDs[i] == tagID) {
                            targetPitch = Optional.of(inputs[j].visibleTagPitches[i]);
                        }
                    }
                }
            }
        }
        return targetPitch;
    }
}
