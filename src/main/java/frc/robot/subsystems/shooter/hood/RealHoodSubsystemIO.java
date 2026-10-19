/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealHoodSubsystemIO implements HoodSubsystemIO {
    private final SparkFlex hoodMotor;
    private final RelativeEncoder hoodMotorEncoder;
    private final SparkClosedLoopController hoodController;
    private final SparkAbsoluteEncoder hoodSparkAbsoluteEncoder;

    public RealHoodSubsystemIO() {
        hoodMotor = new SparkFlex(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodMotorEncoder = hoodMotor.getEncoder();

        hoodSparkAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();
        hoodController = hoodMotor.getClosedLoopController();
        syncEncoder();
        // DO NOT PUT MOTOR INSTRUCTIONS AFTER CONFIG - IT IS BLOCKING
        hoodMotor.configure(HoodConstants.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(HoodSubsystemInputs inputs) {
        inputs.hoodMotorPosition = hoodMotorEncoder.getPosition();
        inputs.hoodMotorVelocity = hoodMotorEncoder.getVelocity();
        inputs.hoodMotorAppliedVoltage = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.hoodMotorCurrentDraw = hoodMotor.getOutputCurrent();
        inputs.hoodMotorTemperature = hoodMotor.getMotorTemperature();
        inputs.atTarget = hoodController.isAtSetpoint();
        inputs.hoodSparkAbsolutePosition = getAbsolutePosition();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity, double arbFF) {
        hoodController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, HoodConstants.HOOD_PID_VELOCITY_SLOT, arbFF);
    }

    public double getAbsolutePosition() {
        return hoodSparkAbsoluteEncoder.getPosition() + HoodConstants.HOOD_ABS_ENCODER_OFFSET;
    }

    @Override
    public void syncEncoder() {
        setEncoder(getAbsolutePosition());
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        hoodController.setSetpoint(
                velocity,
                SparkBase.ControlType.kVelocity,
                HoodConstants.HOOD_PID_VELOCITY_SLOT,
                HoodConstants.feedForward.calculate(velocity));
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        hoodMotor.set(percent);
    }

    /** {@inheritDoc} */
    @Override
    public void setPosition(double setpoint) {
        hoodController.setSetpoint(setpoint, SparkBase.ControlType.kPosition, HoodConstants.HOOD_PID_POSITION_SLOT);
    }

    /** {@inheritDoc} */
    @Override
    public void setEncoder(double angle) {
        hoodMotorEncoder.setPosition(angle);
    }

    /** {@inheritDoc} */
    @Override
    public void setVoltage(double volt) {
        hoodMotor.setVoltage(volt);
    }

    /** {@inheritDoc} */
    @Override
    public void setSoftLimit(boolean direction, double limit) {
        if (direction == true) {
            HoodConstants.hoodConfig.softLimit.forwardSoftLimit(limit);
        } else {
            HoodConstants.hoodConfig.softLimit.reverseSoftLimit(limit);
        }

        hoodMotor.configure(HoodConstants.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void enableSoftLimit(boolean direction, boolean enabled) {
        if (direction == true) {
            HoodConstants.hoodConfig.softLimit.forwardSoftLimitEnabled(enabled);
        } else {
            HoodConstants.hoodConfig.softLimit.reverseSoftLimitEnabled(enabled);
        }

        hoodMotor.configure(HoodConstants.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
