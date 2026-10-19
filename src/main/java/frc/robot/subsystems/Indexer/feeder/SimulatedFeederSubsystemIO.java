/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.Indexer.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedFeederSubsystemIO implements FeederSubsystemIO {
    private final SparkFlex feederMotor;
    private final SparkFlexSim feederMotorSim;

    public SimulatedFeederSubsystemIO() {
        feederMotor = new SparkFlex(FeederConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);

        feederMotor.configure(
                FeederConstants.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotorSim = new SparkFlexSim(feederMotor, DCMotor.getNeoVortex(1));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(FeederSubsystemInputs inputs) {
        inputs.feederMotorPosition = feederMotorSim.getPosition();
        inputs.feederMotorVelocity = feederMotorSim.getVelocity();
        inputs.feederMotorAppliedVoltage = feederMotorSim.getAppliedOutput() * feederMotorSim.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        feederMotorSim.setVelocity(velocity);
    }
}
