/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedIntakeRollerSubsystemIO implements IntakeRollerSubsystemIO {
    private final SparkFlex rollerMotor;
    private final SparkFlex rollerMotorFollower;
    private final SparkFlexSim rollerMotorSim;
    private final SparkFlexSim rollerMotorFollowerSim;

    public SimulatedIntakeRollerSubsystemIO() {
        rollerMotor = new SparkFlex(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        rollerMotorFollower = new SparkFlex(RollerConstants.ROLLER_MOTOR_FOLLOWER_ID, MotorType.kBrushless);

        rollerMotor.configure(
                RollerConstants.rollerConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotorFollower.configure(
                RollerConstants.rollerConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerMotorSim = new SparkFlexSim(rollerMotor, DCMotor.getNeoVortex(1));
        rollerMotorFollowerSim = new SparkFlexSim(rollerMotorFollower, DCMotor.getNeoVortex(1));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(IntakeRollerSubsystemInputs inputs) {
        inputs.intakeRollerMotorPosition = rollerMotorSim.getPosition();
        inputs.intakeRollerMotorVelocity = rollerMotorSim.getVelocity();
        inputs.intakeRollerMotorAppliedVoltage = rollerMotorSim.getAppliedOutput() * rollerMotorSim.getBusVoltage();

        inputs.intakeRollerMotorFollowerPosition = rollerMotorSim.getPosition();
        inputs.intakeRollerMotorFollowerVelocity = rollerMotorSim.getVelocity();
        inputs.intakeRollerMotorFollowerAppliedVoltage =
                rollerMotorSim.getAppliedOutput() * rollerMotorSim.getBusVoltage();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        rollerMotorSim.setAppliedOutput(percent);
    }

    @Override
    public void setVelocity(double velocity) {
        rollerMotorSim.setVelocity(velocity);
    }
}
