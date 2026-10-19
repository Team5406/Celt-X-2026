/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.light;

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

    /**
     * Set the LEDs to a solid colour
     *
     * @param colour colour to set
     **/
    default void setColour(RGBWColor colour) {}
    ;

    /**
     * Clear the animation
     *
     * @param slot animation slot to empty
     **/
    default void stopAnimation(int slot) {}
    ;

    /**
     * Apply strobe animation to flash LEDs
     *
     * @param colour colour to flash
     **/
    default void flashColour(RGBWColor colour) {}
    ;

    /**
     * Apply a bouncing effect to LEDs
     *
     * @param colour colour to flash
     **/
    default void flashLarson(RGBWColor colour) {}
    ;
}
