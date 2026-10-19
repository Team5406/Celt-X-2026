package frc.robot.subsystems.light;

import com.ctre.phoenix6.signals.RGBWColor;

public class LightConstants {
    public static final int CANDLE_CAN_ID = 30;
    public static final int TOTAL_LEDS = 95;
    public static final double LED_SPEED = 5;
    public static final double LIGHT_BRIGHT_SCALER = 0.75;

    public static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    public static final RGBWColor kNone = new RGBWColor(0, 0, 0, 0);
    public static final RGBWColor kOrange = new RGBWColor(255, 120, 0, 0);
    public static final RGBWColor kCyan = new RGBWColor(0, 128, 255, 0);
    public static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    public static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
    public static final RGBWColor kPurple = new RGBWColor(117, 0, 205, 0);
    public static final RGBWColor kWhite = new RGBWColor(255, 255, 255, 0);
    public static final RGBWColor kYellow = new RGBWColor(255, 255, 0, 0);

    public static RGBWColor ACTIVE_COLOUR = kGreen;
    public static RGBWColor INACTIVE_COLOUR = kRed;
    public static RGBWColor DISABLED_COLOUR = kPurple;
    public static RGBWColor GAMEDATA_EMPTY = kYellow;
}
