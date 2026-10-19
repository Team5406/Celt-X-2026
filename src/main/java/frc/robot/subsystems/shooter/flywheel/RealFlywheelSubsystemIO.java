/*
 * Copyright (c) 2026 FRC 5406 - Team CeltX.
 * https://github.com/FRC-5406-Team-CeltX.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealFlywheelSubsystemIO implements FlywheelSubsystemIO {

    private final SparkFlex flywheelMotor;
    private final SparkFlex flywheelMotorFollower;
    private final RelativeEncoder flywheelMotorEncoder;
    private final RelativeEncoder flywheelMotorFollowerEncoder;
    private final SparkClosedLoopController flywheelController;

    public RealFlywheelSubsystemIO() {
        flywheelMotor = new SparkFlex(FlywheelConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(FlywheelConstants.FLYWHEEL_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        flywheelMotorEncoder = flywheelMotor.getEncoder();
        flywheelMotorFollowerEncoder = flywheelMotorFollower.getEncoder();
        flywheelController = flywheelMotor.getClosedLoopController();

        flywheelMotor.configure(
                FlywheelConstants.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelMotorFollower.configure(
                FlywheelConstants.flywheelConfigFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(FlywheelSubsystemInputs inputs) {
        inputs.flywheelMotorPosition = flywheelMotorEncoder.getPosition();
        inputs.flywheelMotorVelocity = flywheelMotorEncoder.getVelocity();
        inputs.flywheelMotorAppliedVoltage = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
        inputs.flywheelMotorCurrentDraw = flywheelMotor.getOutputCurrent();
        inputs.flywheelMotorTemperature = flywheelMotor.getMotorTemperature();
        inputs.flywheelMotorFollowerPosition = flywheelMotorFollowerEncoder.getPosition();
        inputs.flywheelMotorFollowerVelocity = flywheelMotorFollowerEncoder.getVelocity();
        inputs.flywheelMotorFollowerAppliedVoltage =
                flywheelMotorFollower.getAppliedOutput() * flywheelMotorFollower.getBusVoltage();
        inputs.flywheelMotorFollowerCurrentDraw = flywheelMotorFollower.getOutputCurrent();
        inputs.flywheelMotorFollowerTemperature = flywheelMotorFollower.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setPercent(double percent) {
        flywheelMotor.set(percent);
    }

    @Override
    public void setVelocity(double velocity) {
        flywheelController.setSetpoint(velocity, ControlType.kVelocity, FlywheelConstants.FLYWHEEL_PID_VELOCITY_SLOT);
    }

    @Override
    public void setVoltage(double volts) {
        flywheelMotor.setVoltage(volts);
    }
}
