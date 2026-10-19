package frc.robot.subsystems.light;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class RealLightSubsystemIO implements LightSubsystemIO {

    static CANdle candle = new CANdle(LightConstants.CANDLE_CAN_ID);
    // private ColorFlowAnimation animate;

    CANdleConfiguration config = new CANdleConfiguration();

    public RealLightSubsystemIO() {
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = LightConstants.LIGHT_BRIGHT_SCALER;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(config);
        stopAnimations();
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
        stopAnimation(0);
        candle.setControl(new SolidColor(0, LightConstants.TOTAL_LEDS).withColor(colour));
    }

    @Override
    public void flashColour(RGBWColor colour) {
        candle.setControl(new StrobeAnimation(0, LightConstants.TOTAL_LEDS)
                .withColor(colour)
                .withFrameRate(LightConstants.LED_SPEED));
    }

    public void stopAnimations() {
        for (int i = 0; i < 8; ++i) {
            stopAnimation(i);
        }
    }

    @Override
    public void flashLarson(RGBWColor colour) {
        candle.setControl(new LarsonAnimation(0, LightConstants.TOTAL_LEDS)
                .withColor(colour)
                .withSize(15)
                .withFrameRate(100));
    }

    @Override
    public void stopAnimation(int slot) {
        candle.setControl(new EmptyAnimation(slot));
    }
}
