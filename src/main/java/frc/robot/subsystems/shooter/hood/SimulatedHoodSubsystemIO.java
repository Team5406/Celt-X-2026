/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedHoodSubsystemIO implements HoodSubsystemIO {
    private final SparkFlex hoodMotor;
    private final SparkFlexSim hoodMotorSim;

    public SimulatedHoodSubsystemIO() {
        hoodMotor = new SparkFlex(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.encoder.velocityConversionFactor(HoodConstants.HOOD_CONVERSION_FACTOR);

        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMotorSim = new SparkFlexSim(hoodMotor, DCMotor.getNeoVortex(HoodConstants.MOTOR_COUNT));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(HoodSubsystemInputs inputs) {
        inputs.hoodMotorPosition = hoodMotorSim.getPosition();
        inputs.hoodMotorVelocity = hoodMotorSim.getVelocity();
        inputs.hoodMotorAppliedVoltage = hoodMotorSim.getAppliedOutput() * hoodMotorSim.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity, double arbFF) {
        hoodMotorSim.setVelocity(velocity);
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        hoodMotorSim.setVelocity(velocity);
    }
}
