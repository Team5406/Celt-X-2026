package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final double[] timeCoeffs = {0, 0, 0};
    public static final double[] flywheelCoeffs = {2273.6, 795.56, -80.76};
    // public static final double[] flywheelCoeffs = {2195.1, 804.23, -77.049};
    public static final double[] hoodCoeffs = {12.2301, 9.7692, -0.9396};

    public static final double MAX_FLYWHEEL_VELOCITY = 6000.0;
    public static final double MIN_FLYWHEEL_VELOCITY = 0.0;

    public static final double MAX_HOOD_ANGLE = 50.0;
    public static final double MIN_HOOD_ANGLE = 20.0;

    public static final double THRESHOLD = 0.4;
}

/*
public static final double[] flywheelCoeffs = {1418.5, 1016.3, -105.54};  // Original
public static final double[] flywheelCoeffs = {2016.8, 931.07, -95.707};  // Too far generally
 public static final double[] flywheelCoeffs = {2097.6, 875.23, -88.86};  // Untested

 */
