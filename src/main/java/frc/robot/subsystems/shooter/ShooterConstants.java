package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final double[] timeCoeffs = {1.1829, -0.2159, 0.0428};
    public static final double[] flywheelCoeffsAlliance = {3044.4, -231.96, 75.497};
    public static final double[] flywheelCoeffsSnowblow = {2840.4, 70.684, 24.39};
    public static final double[] hoodCoeffsAlliance = {-1.022, 11.569, -1.0229};
    public static final double[] hoodCoeffsSnowblow = {15.266, 7.4959, -0.4473};

    public static final double MAX_FLYWHEEL_VELOCITY = 8000.0;
    public static final double MIN_FLYWHEEL_VELOCITY = 0.0;
    public static final double SNOWBLOW_DISTANCE = 5.6;

    public static final double MAX_HOOD_ANGLE = 50.0;
    public static final double MIN_HOOD_ANGLE = 20.0;

    public static final double THRESHOLD = 0.4;
    public static final double MAX_DISTANCE = 8.4;
    public static final double MAX_HOOD_CALCULATED_ANGLE = 46.6702;
    public static final double FLYWHEEL_SPEED_SCALER = 1.01;
    public static final double DISPLACEMENT_SCALER = 0.05;
}
