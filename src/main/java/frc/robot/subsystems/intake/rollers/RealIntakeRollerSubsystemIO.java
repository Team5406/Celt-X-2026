/*
 * Copyright (c) 2026 FRC 5406 - Celt-X
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.intake.rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealIntakeRollerSubsystemIO implements IntakeRollerSubsystemIO {
    private final SparkFlex rollerMotor;
    private final RelativeEncoder rollerMotorEncoder;
    private final SparkFlex rollerMotorFollower;
    private final SparkClosedLoopController rollerController;

    public RealIntakeRollerSubsystemIO() {
        rollerMotor = new SparkFlex(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        rollerMotorFollower = new SparkFlex(RollerConstants.ROLLER_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
        rollerMotorEncoder = rollerMotor.getEncoder();
        rollerController = rollerMotor.getClosedLoopController();

        rollerMotor.configure(
                RollerConstants.rollerConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotorFollower.configure(
                RollerConstants.rollerConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(IntakeRollerSubsystemInputs inputs) {
        inputs.intakeRollerMotorPosition = rollerMotorEncoder.getPosition();
        inputs.intakeRollerMotorVelocity = rollerMotorEncoder.getVelocity();
        inputs.intakeRollerMotorAppliedVoltage = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
        inputs.intakeRollerMotorCurrentDraw = rollerMotor.getOutputCurrent();
        inputs.intakeRollerMotorTemperature = rollerMotor.getMotorTemperature();

        inputs.intakeRollerMotorFollowerPosition = rollerMotorEncoder.getPosition();
        inputs.intakeRollerMotorFollowerVelocity = rollerMotorEncoder.getVelocity();
        inputs.intakeRollerMotorFollowerAppliedVoltage =
                rollerMotorFollower.getAppliedOutput() * rollerMotorFollower.getBusVoltage();
        inputs.intakeRollerMotorFollowerCurrentDraw = rollerMotorFollower.getOutputCurrent();
        inputs.intakeRollerMotorFollowerTemperature = rollerMotorFollower.getMotorTemperature();
    }

    @Override
    public void setPercent(double percent) {
        rollerMotor.set(percent);
    }

    @Override
    public void setVelocity(double velocity) {
        rollerController.setSetpoint(
                velocity, SparkBase.ControlType.kVelocity, RollerConstants.ROLLER_PID_VELOCITY_SLOT);
    }

    public void setCurrentLimit() {
        RollerConstants.rollerConfigBase.smartCurrentLimit(RollerConstants.ROLLER_TELEOP_CURRENT_LIMIT);
        rollerMotor.configure(
                RollerConstants.rollerConfigBase, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
