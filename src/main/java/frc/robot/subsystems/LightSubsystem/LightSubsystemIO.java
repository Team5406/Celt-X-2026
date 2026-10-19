/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.LightSubsystem;

import com.ctre.phoenix6.signals.RGBWColor;
import org.littletonrobotics.junction.AutoLog;

public interface LightSubsystemIO {

    @AutoLog
    class LightSubsystemInputs {
        public double current;
        public double temperature;
        public double voltage;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(LightSubsystemInputs inputs) {}
    ;

    /** Set the colour */
    default void setColour(RGBWColor colour) {}
    ;

    default void setAnimation(RGBWColor colour) {}
    ;

    default void stopAnimation() {}
    ;
}
