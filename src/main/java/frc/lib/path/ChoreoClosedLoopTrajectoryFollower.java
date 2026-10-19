/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.kinematics.ChassisState;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ChoreoClosedLoopTrajectoryFollower implements TrajectoryFollower {
    Optional<Trajectory<SwerveSample>> moreTrajectory;
    Trajectory<SwerveSample> trajectory;
    Trajectory<SwerveSample> trajectory2;
    PathPlannerTrajectoryState pathPlannerTrajectory;
    private double startTime;
    private boolean isFinished = false;

    private final PIDController xController = new PIDController(0.75, 0, 0);
    private final PIDController yController = new PIDController(0.75, 0, 0);
    private final PIDController headingController = new PIDController(10, 0, 0);

    public ChoreoClosedLoopTrajectoryFollower(String trajectoryName, int splitIndex) {
        var trajectoryOptional = Choreo.loadTrajectory(trajectoryName).get().getSplit(splitIndex);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        if (trajectoryOptional.isPresent()) {
            try {
                // TODO check if there is a better way to do this
                trajectory = (Trajectory<SwerveSample>) trajectoryOptional.get();
            } catch (ClassCastException e) {
                System.err.println("Trajectory " + trajectoryName + " is not a Swerve trajectory");
            }
        } else {
            System.err.println("Trajectory " + trajectoryName + " not found");
        }
    }

    /** Record the starting time of the trajectory */
    @Override
    public void recordStartingTime() {
        isFinished = false;
        startTime = Logger.getTimestamp() / 1e6;
    }
    ;

    /** {@inheritDoc} */
    public Pose2d getStartPose() {
        Optional<SwerveSample> sample = trajectory.sampleAt(
                0,
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        return new Pose2d(sample.get().x, sample.get().y, new Rotation2d(sample.get().heading));
    }

    /** {@inheritDoc} */
    @Override
    public ChassisState getState(Pose2d currentPose, ChassisSpeeds currentSpeed) {
        double timestamp = Logger.getTimestamp() / 1e6;
        // Sample the trajectory based on time

        Optional<SwerveSample> sample = trajectory.sampleAt(
                timestamp - startTime,
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        Logger.recordOutput("Trajectory", sample.get().getPose());

        double xCorrection = xController.calculate(currentPose.getX(), sample.get().x);
        double yCorrection = yController.calculate(currentPose.getY(), sample.get().y);
        double headingCorrection =
                headingController.calculate(currentPose.getRotation().getRadians(), sample.get().heading);
        if (timestamp - startTime > trajectory.getTotalTime()) {
            xCorrection = 0;
            yCorrection = 0;
            headingCorrection = 0;
            isFinished = true;
        }

        // compute chassis speeds
        return ChassisState.fromFieldRelativeState(
                sample.get().vx + xCorrection,
                sample.get().vy + yCorrection,
                sample.get().omega + headingCorrection,
                sample.get().ax,
                sample.get().ay,
                sample.get().alpha,
                currentPose.getRotation());
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
