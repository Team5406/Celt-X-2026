/*
 * Copyright (c) 2026 FRC 5406 - Team CeltX.
 * https://github.com/FRC-5406-Team-CeltX.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedFlywheelSubsystemIO implements FlywheelSubsystemIO {
    private final SparkFlex flywheelMotor;
    private final SparkFlexSim flywheelMotorSim;
    private final SparkFlex flywheelMotorFollower;
    private final SparkFlexSim flywheelMotorFollowerSim;

    public SimulatedFlywheelSubsystemIO() {
        flywheelMotor = new SparkFlex(FlywheelConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(FlywheelConstants.FLYWHEEL_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        flywheelMotor.configure(
                FlywheelConstants.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelMotorSim = new SparkFlexSim(flywheelMotor, DCMotor.getNeoVortex(1));

        flywheelMotorFollower.configure(
                FlywheelConstants.flywheelConfigFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        flywheelMotorFollowerSim = new SparkFlexSim(flywheelMotorFollower, DCMotor.getNeoVortex(1));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(FlywheelSubsystemInputs inputs) {
        inputs.flywheelMotorPosition = flywheelMotorSim.getPosition();
        inputs.flywheelMotorVelocity = flywheelMotorSim.getVelocity();
        inputs.flywheelMotorAppliedVoltage = flywheelMotorSim.getAppliedOutput() * flywheelMotorSim.getBusVoltage();
        inputs.flywheelMotorFollowerPosition = flywheelMotorFollowerSim.getPosition();
        inputs.flywheelMotorFollowerVelocity = flywheelMotorFollowerSim.getVelocity();
        inputs.flywheelMotorFollowerAppliedVoltage =
                flywheelMotorFollowerSim.getAppliedOutput() * flywheelMotorFollowerSim.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        flywheelMotorSim.setAppliedOutput(percent);
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        flywheelMotorSim.setVelocity(velocity);
    }
}
