/*
 * Copyright (c) 2026 FRC 5406 - Celt-X.
 * https://github.com/Team5406
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedTurretSubsystemIO implements TurretSubsystemIO {
    private final SparkFlex turretMotor;
    private final SparkFlexSim turretMotorSim;

    public SimulatedTurretSubsystemIO() {
        turretMotor = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
        turretMotor.configure(
                TurretConstants.turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        turretMotorSim = new SparkFlexSim(turretMotor, DCMotor.getNeoVortex(1));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(TurretSubsystemInputs inputs) {
        inputs.turretMotorPosition = turretMotorSim.getPosition();
        inputs.turretMotorVelocity = turretMotorSim.getVelocity();
        inputs.turretMotorAppliedVoltage = turretMotorSim.getAppliedOutput() * turretMotorSim.getBusVoltage();
    }

    @Override
    public void setVelocity(double velocity) {
        turretMotorSim.setVelocity(velocity);
    }
}
