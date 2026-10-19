/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.spindexer.serializer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedSerializerSubsystemIO implements SerializerSubsystemIO {
    private final SparkFlex serializerMotor;
    private final SparkFlexSim serializerMotorSim;

    public SimulatedSerializerSubsystemIO() {
        serializerMotor = new SparkFlex(SerializerConstants.SERIALIZER_MOTOR_ID, MotorType.kBrushless);

        SerializerConstants.serializerConfig.encoder.velocityConversionFactor(
                SerializerConstants.SERIALIZER_CONVERSION_FACTOR);

        serializerMotor.configure(
                SerializerConstants.serializerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        serializerMotorSim =
                new SparkFlexSim(serializerMotor, DCMotor.getNeoVortex(SerializerConstants.SERIALIZER_MOTOR_COUNT));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(SerializerSubsystemInputs inputs) {
        inputs.serializerMotorPosition = serializerMotorSim.getPosition();
        inputs.serializerMotorVelocity = serializerMotorSim.getVelocity();
        inputs.serializerMotorAppliedVoltage =
                serializerMotorSim.getAppliedOutput() * serializerMotorSim.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity, double arbFF) {
        serializerMotorSim.setVelocity(velocity);
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        serializerMotorSim.setVelocity(velocity);
    }

    @Override
    public void setPosition(double setpoint) {
        serializerMotorSim.setPosition(setpoint);
    }
}
