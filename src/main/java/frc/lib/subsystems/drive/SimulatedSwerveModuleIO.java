/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.subsystems.drive;

import static frc.lib.hardware.constants.GearRatios.RevRobotics.MaxSwerve.REDUCTION_AZIMUTH;
import static frc.lib.hardware.constants.GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.DriveConstants;

import org.littletonrobotics.junction.Logger;

/**
 * A simulated swerve module for use in physics simulation. The model simulates the physical
 * properties of a NEO Vortex for the drive motor and those of a NEO 550 for the turn motor.
 *
 * <p>This class is not meant to accurately represent a real swerve module. Yet it should be a good
 * enough approximation for testing purposes.
 *
 * <p>The setpoints for the simulated module are derived using computed torque control.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Computed_torque_control">Computed Torque Control</a>
 */
public class SimulatedSwerveModuleIO implements SwerveModuleIO {

  private static final double LOOP_DELTA_TIME = 0.02;
  private static final double WHEEL_RADIUS = Units.Meter.convertFrom(2, Units.Inch);

  private static final double DRIVE_INERTIA = 0.15;
  private static final double TURN_INERTIA = 0.05;

  private static final double DRIVE_RATIO = DriveConstants.DRIVE_GEAR_RATIO;
  private static final double TURN_RATIO = DriveConstants.TURN_GEAR_RATIO;

  private PIDController driveController = new PIDController(0.05, 0, 0);
  private PIDController turnController = new PIDController(4, 0, 0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private final DCMotor driveMotor = DCMotor.getNEO(1);
  private final DCMotor turnMotor = DCMotor.getNEO(1);
  

  double driveDesiredVelocity = 0; // Desired velocity in rad/s of the wheel
  double turnDesiredPosition = 0; // Desired rotation of the wheel in rad

  /** Creates a new {@code SimulatedSwerveModuleIO}. */
  public SimulatedSwerveModuleIO() {
    // The inertia that DCMotorSim takes as input is most likely the inertia felt by the motor
    // Therefore the gear ratio should be taken into account
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveMotor, 0.025, DRIVE_RATIO),
            driveMotor);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotor, 0.004, TURN_RATIO),
            turnMotor);
    
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    // Random offset when the robot starts to make the simulation more interesting
    //Rotation2d arbitraryTurnOffset = new Rotation2d();
    //turnSim.setState(arbitraryTurnOffset.getRadians(), 0);
  }

  /**
   * Updates the system with new inputs from the module.
   *
   * @param inputs The new inputs.
   */
  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // Determine the voltage that needs to be sent to the sim objects using Inverse Dynamics,
    // also known as computed torque control.
    // The desired acceleration is determined based on the error in velocity and position
    // This acceleration is then used to compute the voltage that needs to be sent to the motors
    driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS);
    turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(LOOP_DELTA_TIME);
    turnSim.update(LOOP_DELTA_TIME);

    inputs.driveConnected = true;
    inputs.drivePosition = driveSim.getAngularPositionRad() * WHEEL_RADIUS;
    inputs.driveVelocity = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS;
    inputs.driveAppliedVoltage = driveAppliedVolts;
    inputs.driveCurrent = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnAbsoluteRotation = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnAngularVelocity = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVoltage = turnAppliedVolts;
    inputs.turnCurrent = Math.abs(turnSim.getCurrentDrawAmps());

    // No high speed odometry thread is running in the sim, so just fill the arrays with the
    // current value
    inputs.odometryTimestamps = new double[] {Logger.getTimestamp() / 1e6};
    inputs.odometryDrivePositions = new double[] {inputs.drivePosition};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnAbsoluteRotation};
  }

  /** {@inheritDoc} */
  @Override
  public void setDriveState(double velocity, double acceleration) {
    driveFFVolts = 0 * Math.signum(velocity) + 0.135 * velocity / WHEEL_RADIUS;
    driveController.setSetpoint(velocity);
  }

  /** {@inheritDoc} */
  @Override
  public void setTurnState(double rotation, double angularVelocity) {
    turnController.setSetpoint(rotation);
  }

}
