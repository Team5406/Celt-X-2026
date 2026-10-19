package frc.robot.constants;

import com.ctre.phoenix6.signals.RGBWColor;

public class LightConstants {
    public static final int CANDLE_CAN_ID = 30;
    public static final int TOTAL_LEDS = 68; // 30 each side + 8 for CANdle
    public static final double LED_SPEED = 0.3;

    public static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    public static final RGBWColor kNone = new RGBWColor(0, 0, 0, 0);
    public static final RGBWColor kOrange = new RGBWColor(255, 120, 0, 0);
    public static final RGBWColor kCyan = new RGBWColor(0, 128, 255, 0);
    public static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    public static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
    public static final RGBWColor kPurple = new RGBWColor(117, 0, 205, 0);
    public static final RGBWColor kWhite = new RGBWColor(255, 255, 255, 0);
}
