/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.pivot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedIntakePivotSubsystemIO implements IntakePivotSubsystemIO {
    private final SparkFlex pivotMotor;
    // private final SparkFlex pivotMotorFollower;
    private final SparkFlexSim pivotMotorSim;

    // private final SparkFlexSim pivotMotorFollowerSim;

    public SimulatedIntakePivotSubsystemIO() {
        pivotMotor = new SparkFlex(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        // pivotMotorFollower = new SparkFlex(IntakePivotConstants.PIVOT_MOTOR_FOLLOWER_ID,
        // MotorType.kBrushless);

        pivotMotor.configure(
                IntakePivotConstants.pivotConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pivotMotorFollower.configure(
        //         IntakePivotConstants.pivotConfigFollower,
        //         ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);

        pivotMotorSim = new SparkFlexSim(pivotMotor, DCMotor.getNeoVortex(1));
        // pivotMotorFollowerSim = new SparkFlexSim(pivotMotorFollower, DCMotor.getNeoVortex(1));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(IntakePivotSubsystemInputs inputs) {
        inputs.intakePivotMotorPosition = pivotMotorSim.getPosition();
        inputs.intakePivotMotorVelocity = pivotMotorSim.getVelocity();
        inputs.intakePivotMotorAppliedVoltage = pivotMotorSim.getAppliedOutput() * pivotMotor.getBusVoltage();

        //     inputs.intakePivotMotorFollowPosition = pivotMotorFollowerSim.getPosition();
        //     inputs.intakePivotMotorFollowVelocity = pivotMotorFollowerSim.getVelocity();
        //     inputs.intakePivotMotorFollowAppliedVoltage =
        //             pivotMotorFollowerSim.getAppliedOutput() * pivotMotorFollower.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        pivotMotorSim.setAppliedOutput(percent);
    }

    @Override
    public void setVelocity(double velocity, double arbFF) {
        pivotMotorSim.setVelocity(velocity);
    }
}
