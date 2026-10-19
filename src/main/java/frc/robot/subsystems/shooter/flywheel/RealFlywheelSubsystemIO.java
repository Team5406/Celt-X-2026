/*
 * Copyright (c) 2026 FRC 5406 - Team CeltX.
 * https://github.com/FRC-5406-Team-CeltX.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealFlywheelSubsystemIO implements FlywheelSubsystemIO {

    private final SparkFlex flywheelMotor;
    private final SparkFlex flywheelSecondMotor;
    private final SparkFlex flywheelMotorFollower;
    private final SparkFlex flywheelSecondMotorFollower;
    private final RelativeEncoder flywheelMotorEncoder;
    private final RelativeEncoder flywheelSecondMotorEncoder;
    private final RelativeEncoder flywheelMotorFollowerEncoder;
    private final RelativeEncoder flywheelSecondMotorFollowerEncoder;
    private final SparkClosedLoopController flywheelController;
    private final SparkClosedLoopController flywheelSecondController;
    private final SparkClosedLoopController flywheelFollowerController;
    private final SparkClosedLoopController flywheelSecondFollowerController;

    public RealFlywheelSubsystemIO() {
        flywheelMotor = new SparkFlex(FlywheelConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelSecondMotor = new SparkFlex(FlywheelConstants.FLYWHEEL_MOTOR_SECOND_ID, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(FlywheelConstants.FLYWHEEL_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        flywheelSecondMotorFollower =
                new SparkFlex(FlywheelConstants.FLYWHEEL_FOLLOWER_MOTOR_SECOND_ID, MotorType.kBrushless);
        flywheelMotorEncoder = flywheelMotor.getEncoder();
        flywheelSecondMotorEncoder = flywheelSecondMotor.getEncoder();
        flywheelMotorFollowerEncoder = flywheelMotorFollower.getEncoder();
        flywheelSecondMotorFollowerEncoder = flywheelSecondMotorFollower.getEncoder();
        flywheelController = flywheelMotor.getClosedLoopController();
        flywheelSecondController = flywheelSecondMotor.getClosedLoopController();
        flywheelFollowerController = flywheelMotorFollower.getClosedLoopController();
        flywheelSecondFollowerController = flywheelSecondMotorFollower.getClosedLoopController();

        flywheelMotor.configure(
                FlywheelConstants.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelSecondMotor.configure(
                FlywheelConstants.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotorFollower.configure(
                FlywheelConstants.flywheelConfigFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        flywheelSecondMotorFollower.configure(
                FlywheelConstants.flywheelConfigFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(FlywheelSubsystemInputs inputs) {
        inputs.flywheelMotorPosition = flywheelMotorEncoder.getPosition();
        inputs.flywheelMotorVelocity = flywheelMotorEncoder.getVelocity();
        inputs.flywheelMotorAppliedVoltage = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
        inputs.flywheelMotorCurrentDraw = flywheelMotor.getOutputCurrent();
        inputs.flywheelMotorTemperature = flywheelMotor.getMotorTemperature();
        inputs.flywheelSecondMotorPosition = flywheelSecondMotorEncoder.getPosition();
        inputs.flywheelSecondMotorVelocity = flywheelSecondMotorEncoder.getVelocity();
        inputs.flywheelSecondMotorAppliedVoltage =
                flywheelSecondMotor.getAppliedOutput() * flywheelSecondMotor.getBusVoltage();
        inputs.flywheelSecondMotorCurrentDraw = flywheelSecondMotor.getOutputCurrent();
        inputs.flywheelSecondMotorTemperature = flywheelSecondMotor.getMotorTemperature();
        inputs.flywheelMotorFollowerPosition = flywheelMotorFollowerEncoder.getPosition();
        inputs.flywheelMotorFollowerVelocity = flywheelMotorFollowerEncoder.getVelocity();
        inputs.flywheelMotorFollowerAppliedVoltage =
                flywheelMotorFollower.getAppliedOutput() * flywheelMotorFollower.getBusVoltage();
        inputs.flywheelMotorFollowerCurrentDraw = flywheelSecondMotorFollower.getOutputCurrent();
        inputs.flywheelMotorFollowerTemperature = flywheelSecondMotorFollower.getMotorTemperature();
        inputs.flywheelSecondMotorFollowerPosition = flywheelSecondMotorFollowerEncoder.getPosition();
        inputs.flywheelSecondMotorFollowerVelocity = flywheelSecondMotorFollowerEncoder.getVelocity();
        inputs.flywheelSecondMotorFollowerAppliedVoltage =
                flywheelSecondMotorFollower.getAppliedOutput() * flywheelSecondMotorFollower.getBusVoltage();
        inputs.flywheelSecondMotorFollowerCurrentDraw = flywheelSecondMotorFollower.getOutputCurrent();
        inputs.flywheelSecondMotorFollowerTemperature = flywheelSecondMotorFollower.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        flywheelMotor.set(percent);
        flywheelMotorFollower.set(percent);
        flywheelSecondMotor.set(percent);
        flywheelSecondMotorFollower.set(percent);
    }

    @Override
    public void setVelocity(double velocity) {
        flywheelController.setSetpoint(velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT);
        flywheelFollowerController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT);
        flywheelSecondController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT);
        flywheelSecondFollowerController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT);
    }

    @Override
    public void setVelocity(double velocity, double arbFF) {
        flywheelController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT, arbFF);
        flywheelFollowerController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT, arbFF);
        flywheelSecondController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT, arbFF);
        flywheelSecondFollowerController.setSetpoint(
                velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT, arbFF);
    }

    @Override
    public void setVoltage(double volts) {
        flywheelMotor.setVoltage(volts);
        flywheelMotorFollower.setVoltage(volts);
        flywheelSecondMotor.setVoltage(volts);
        flywheelSecondMotorFollower.setVoltage(volts);
    }
}
