/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.pivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealIntakePivotSubsystemIO implements IntakePivotSubsystemIO {
    private final SparkFlex pivotMotor;
    // private final SparkFlex pivotMotorFollower;
    private final RelativeEncoder pivotMotorEncoder;
    // private final RelativeEncoder pivotMotorFollowerEncoder;
    private final SparkClosedLoopController pivotController;

    public RealIntakePivotSubsystemIO() {
        pivotMotor = new SparkFlex(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        // pivotMotorFollower = new SparkFlex(IntakePivotConstants.PIVOT_MOTOR_FOLLOWER_ID,
        // MotorType.kBrushless);
        pivotMotorEncoder = pivotMotor.getEncoder();
        // pivotMotorFollowerEncoder = pivotMotorFollower.getEncoder();

        pivotController = pivotMotor.getClosedLoopController();

        pivotMotor.configure(
                IntakePivotConstants.pivotConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pivotMotorFollower.configure(
        //         IntakePivotConstants.pivotConfigFollower,
        //         ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(IntakePivotSubsystemInputs inputs) {
        inputs.intakePivotMotorPosition = pivotMotorEncoder.getPosition();
        inputs.intakePivotMotorVelocity = pivotMotorEncoder.getVelocity();
        inputs.intakePivotMotorAppliedVoltage = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.intakePivotMotorCurrentDraw = pivotMotor.getOutputCurrent();
        inputs.intakePivotMotorTemperature = pivotMotor.getMotorTemperature();
        inputs.maxMotionPositionSetpoint = pivotController.getMAXMotionSetpointPosition();
        inputs.maxMotionVelocitySetpoint = pivotController.getMAXMotionSetpointVelocity();
        inputs.controllerSetpoint = pivotController.getSetpoint();
        // inputs.intakePivotMotorFollowPosition = pivotMotorFollowerEncoder.getPosition();
        // inputs.intakePivotMotorFollowVelocity = pivotMotorFollowerEncoder.getVelocity();
        // inputs.intakePivotMotorFollowAppliedVoltage =
        //         pivotMotorFollower.getAppliedOutput() * pivotMotorFollower.getBusVoltage();
        // inputs.intakePivotMotorFollowCurrentDraw = pivotMotorFollower.getOutputCurrent();
        // inputs.intakePivotMotorFollowTemperature = pivotMotorFollower.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        pivotMotor.set(percent);
    }

    @Override
    public void setPosition(double setpoint) {
        pivotController.setSetpoint(
                setpoint, SparkBase.ControlType.kPosition, IntakePivotConstants.PIVOT_PID_POSITION_SLOT);
    }

    @Override
    public void setVelocity(double velocity) {
        pivotController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, IntakePivotConstants.PIVOT_PID_VELOCITY_SLOT);
    }

    @Override
    public void setVelocity(double velocity, double arbFF) {
        pivotController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, IntakePivotConstants.PIVOT_PID_VELOCITY_SLOT, arbFF);
    }

    @Override
    public void setEncoder(double position) {
        pivotMotorEncoder.setPosition(position);
        // pivotMotorFollowerEncoder.setPosition(position);
    }

    @Override
    public void setVoltage(double volt) {
        pivotMotor.setVoltage(volt);
    }

    public void setProfiledPosition(double position) {
        pivotController.setSetpoint(
                position,
                SparkBase.ControlType.kMAXMotionPositionControl,
                IntakePivotConstants.PIVOT_PID_POSITION_SLOT);
    }

    @Override
    public void enableSoftLimit(boolean forward, boolean enabled) {
        if (forward) {
            IntakePivotConstants.pivotConfigLeader.softLimit.forwardSoftLimitEnabled(enabled);
        } else {
            IntakePivotConstants.pivotConfigLeader.softLimit.reverseSoftLimitEnabled(enabled);
        }
        pivotMotor.configure(
                IntakePivotConstants.pivotConfigLeader,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void setSoftLimit(boolean forward, double val) {
        if (forward) {
            IntakePivotConstants.pivotConfigLeader.softLimit.forwardSoftLimit(val);
        } else {
            IntakePivotConstants.pivotConfigLeader.softLimit.reverseSoftLimit(val);
        }
        pivotMotor.configure(
                IntakePivotConstants.pivotConfigLeader,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }
}
