/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.Indexer.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealFeederSubsystemIO implements FeederSubsystemIO {
    private final SparkFlex feederMotor;
    private final RelativeEncoder feederMotorEncoder;
    private final SparkClosedLoopController feederController;

    public RealFeederSubsystemIO() {
        feederMotor = new SparkFlex(FeederConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        feederMotorEncoder = feederMotor.getEncoder();
        feederController = feederMotor.getClosedLoopController();

        feederMotor.configure(
                FeederConstants.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(FeederSubsystemInputs inputs) {
        inputs.feederMotorPosition = feederMotorEncoder.getPosition();
        inputs.feederMotorVelocity = feederMotorEncoder.getVelocity();
        inputs.feederMotorAppliedVoltage = feederMotor.getAppliedOutput() * feederMotor.getBusVoltage();
        inputs.feederMotorCurrentDraw = feederMotor.getOutputCurrent();
        inputs.feederMotorTemperature = feederMotor.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        feederMotor.set(percent);
    }

    @Override
    public void setVelocity(double velocity) {
        double arbFF = FeederConstants.feederFeedForward.calculate(velocity);
        feederController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, FeederConstants.FEEDER_PID_VELOCITY_SLOT, arbFF);
    }

    @Override
    public void setVoltage(double volt) {
        feederMotor.setVoltage(volt);
    }
}
