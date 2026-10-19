/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.Indexer.rollerFloor;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystemIO.RollerFloorSubsystemInputs;

public class RealRollerFloorSubsystemIO implements RollerFloorSubsystemIO {
    private final SparkFlex floorMotor;
    private final RelativeEncoder floorMotorEncoder;
    private final SparkClosedLoopController floorController;

    public RealRollerFloorSubsystemIO() {
        floorMotor = new SparkFlex(RollerFloorConstants.FLOOR_MOTOR_ID, MotorType.kBrushless);
        floorMotorEncoder = floorMotor.getEncoder();

        floorController = floorMotor.getClosedLoopController();

        floorMotor.configure(
                RollerFloorConstants.floorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(RollerFloorSubsystemInputs inputs) {
        inputs.floorMotorPosition = floorMotorEncoder.getPosition();
        inputs.floorMotorVelocity = floorMotorEncoder.getVelocity();
        inputs.floorMotorAppliedVoltage = floorMotor.getAppliedOutput() * floorMotor.getBusVoltage();
        inputs.floorMotorCurrentDraw = floorMotor.getOutputCurrent();
        inputs.floorMotorTemperature = floorMotor.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity, double arbFF) {
        floorController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, RollerFloorConstants.FLOOR_PID_VELOCITY_SLOT, arbFF);
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        floorController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, RollerFloorConstants.FLOOR_PID_VELOCITY_SLOT);
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        floorMotor.set(percent);
    }

    /** {@inheritDoc} */
    @Override
    public void setEncoder(double position) {
        floorMotorEncoder.setPosition(position);
    }

    /** {@inheritDoc} */
    @Override
    public void setVoltage(double volt) {
        floorMotor.setVoltage(volt);
    }
}
