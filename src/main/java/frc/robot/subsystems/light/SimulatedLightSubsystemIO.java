package frc.robot.subsystems.light;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.constants.LightConstants;

public class SimulatedLightSubsystemIO implements LightSubsystemIO {

    static CANdle candle = new CANdle(LightConstants.CANDLE_CAN_ID);

    CANdleConfiguration config = new CANdleConfiguration();

    public SimulatedLightSubsystemIO() {
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 0.5;
        candle.getConfigurator().apply(config);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(LightSubsystemInputs inputs) {
        inputs.current = candle.getOutputCurrent().getValueAsDouble();
        inputs.temperature = candle.getDeviceTemp().getValueAsDouble();
        inputs.voltage = candle.getFiveVRailVoltage().getValueAsDouble();
    }

    @Override
    public void setColour(RGBWColor colour) {
        candle.setControl(new SolidColor(0, LightConstants.TOTAL_LEDS).withColor(colour));
    }
}
