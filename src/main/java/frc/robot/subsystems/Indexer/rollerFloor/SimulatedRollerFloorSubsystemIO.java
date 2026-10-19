/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.Indexer.rollerFloor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedRollerFloorSubsystemIO implements RollerFloorSubsystemIO {
    private final SparkFlex floorMotor;
    private final SparkFlexSim floorMotorSim;

    public SimulatedRollerFloorSubsystemIO() {
        floorMotor = new SparkFlex(RollerFloorConstants.FLOOR_MOTOR_ID, MotorType.kBrushless);

        RollerFloorConstants.floorConfig.encoder.velocityConversionFactor(RollerFloorConstants.FLOOR_CONVERSION_FACTOR);

        floorMotor.configure(
                RollerFloorConstants.floorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        floorMotorSim = new SparkFlexSim(floorMotor, DCMotor.getNeoVortex(RollerFloorConstants.FLOOR_MOTOR_COUNT));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(RollerFloorSubsystemInputs inputs) {
        inputs.floorMotorPosition = floorMotorSim.getPosition();
        inputs.floorMotorVelocity = floorMotorSim.getVelocity();
        inputs.floorMotorAppliedVoltage = floorMotorSim.getAppliedOutput() * floorMotorSim.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity, double arbFF) {
        floorMotorSim.setVelocity(velocity);
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        floorMotorSim.setVelocity(velocity);
    }

    @Override
    public void setPosition(double setpoint) {
        floorMotorSim.setPosition(setpoint);
    }

    @Override
    public void setPercent(double percent) {
        floorMotorSim.setAppliedOutput(percent);
    }
}
