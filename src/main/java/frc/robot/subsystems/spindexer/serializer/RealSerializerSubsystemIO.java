/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.spindexer.serializer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.spindexer.serializer.SerializerSubsystemIO.SerializerSubsystemInputs;

public class RealSerializerSubsystemIO implements SerializerSubsystemIO {
    private final SparkFlex serializerMotor;
    private final RelativeEncoder serializerMotorEncoder;
    private final SparkClosedLoopController serializerController;

    public RealSerializerSubsystemIO() {
        serializerMotor = new SparkFlex(SerializerConstants.SERIALIZER_MOTOR_ID, MotorType.kBrushless);
        serializerMotorEncoder = serializerMotor.getEncoder();

        serializerController = serializerMotor.getClosedLoopController();

        serializerMotor.configure(
                SerializerConstants.serializerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(SerializerSubsystemInputs inputs) {
        inputs.serializerMotorPosition = serializerMotorEncoder.getPosition();
        inputs.serializerMotorVelocity = serializerMotorEncoder.getVelocity();
        inputs.serializerMotorAppliedVoltage = serializerMotor.getAppliedOutput() * serializerMotor.getBusVoltage();
        inputs.serializerMotorCurrentDraw = serializerMotor.getOutputCurrent();
        inputs.serializerMotorTemperature = serializerMotor.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity, double arbFF) {
        serializerController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, SerializerConstants.SERIALIZER_PID_VELOCITY_SLOT, arbFF);
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        serializerController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, SerializerConstants.SERIALIZER_PID_VELOCITY_SLOT);
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        serializerMotor.set(percent);
    }

    /** {@inheritDoc} */
    @Override
    public void setEncoder(double position) {
        serializerMotorEncoder.setPosition(position);
    }

    /** {@inheritDoc} */
    @Override
    public void setVoltage(double volt) {
        serializerMotor.setVoltage(volt);
    }
}
