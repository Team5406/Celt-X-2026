/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealTurretSubsystemIO implements TurretSubsystemIO {
    private final SparkFlex turretMotor;
    private final RelativeEncoder turretMotorEncoder;
    private final CANcoder turretCANcoder;
    private final SparkClosedLoopController turretController;

    public RealTurretSubsystemIO() {
        turretMotor = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
        turretMotorEncoder = turretMotor.getEncoder();
        turretCANcoder = new CANcoder(TurretConstants.TURRET_CANCODER_ID);

        turretController = turretMotor.getClosedLoopController();

        turretMotor.configure(
                TurretConstants.turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        syncEncoder();
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(TurretSubsystemInputs inputs) {
        inputs.turretMotorPosition = turretMotorEncoder.getPosition();
        inputs.turretMotorVelocity = turretMotorEncoder.getVelocity();
        inputs.turretMotorAppliedVoltage = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.turretMotorCurrentDraw = turretMotor.getOutputCurrent();
        inputs.turretMotorTemperature = turretMotor.getMotorTemperature();
        inputs.atTarget = turretController.isAtSetpoint();
        inputs.maxMotionPositionSetpoint = turretController.getMAXMotionSetpointPosition();
        inputs.maxMotionVelocitySetpoint = turretController.getMAXMotionSetpointVelocity();
        inputs.controllerSetpoint = turretController.getSetpoint();
    }

    @Override
    public void setVelocity(double velocity, double arbFF) {
        turretController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, TurretConstants.TURRET_PID_VELOCITY_SLOT, arbFF);
    }

    @Override
    public void setVelocity(double velocity) {
        double arbFF = TurretConstants.turretFeedForward.calculate(velocity);
        setVelocity(velocity, arbFF);
    }

    @Override
    public void setVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    @Override
    public void setEncoder(double position) {
        turretMotorEncoder.setPosition(position);
    }

    public double getCANCoderPosition() {
        return turretCANcoder.getPosition().getValueAsDouble() * TurretConstants.TURRET_CANCODER_CONVERSION_FACTOR;
    }

    @Override
    public void syncEncoder() {
        setEncoder(getCANCoderPosition());
    }

    @Override
    public void setPosition(double position) {
        turretController.setSetpoint(
                position, SparkBase.ControlType.kPosition, TurretConstants.TURRET_PID_POSITION_SLOT);
    }

    public void setProfiledPosition(double position) {
        turretController.setSetpoint(
                position, SparkBase.ControlType.kMAXMotionPositionControl, TurretConstants.TURRET_PID_POSITION_SLOT);
    }

    @Override
    public void setPercent(double percent) {
        turretMotor.set(percent);
    }

    @Override
    public void enableLimit(boolean forward, boolean enabled) {
        if (forward) {
            TurretConstants.turretConfig.softLimit.forwardSoftLimitEnabled(enabled);
        } else {
            TurretConstants.turretConfig.softLimit.reverseSoftLimitEnabled(enabled);
        }
        turretMotor.configure(
                TurretConstants.turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setLimit(boolean forward, double val) {
        if (forward) {
            TurretConstants.turretConfig.softLimit.forwardSoftLimit(val);
        } else {
            TurretConstants.turretConfig.softLimit.reverseSoftLimit(val);
        }
        turretMotor.configure(
                TurretConstants.turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
