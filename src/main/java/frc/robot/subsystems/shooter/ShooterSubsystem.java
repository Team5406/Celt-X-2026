package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private double flywheelSpeed; // = 700;
    private double hoodAngle;
    private final Drive drive;
    private final HoodSubsystem hood;
    private final FlywheelSubsystem flywheel;

    public ShooterSubsystem(Drive drive, HoodSubsystem hood, FlywheelSubsystem flywheel) {
        this.drive = drive;
        this.hood = hood;
        this.flywheel = flywheel;
    }

    public Command spinUp() {
        return Commands.parallel(flywheel.setVelocity(() -> flywheelSpeed), hood.setPosition(() -> hoodAngle));
        /*return Commands.run(() -> {

            flywheel.setVelocity(1000);
            Logger.recordOutput("Flywheel Speed haha", flywheelSpeed);
        });*/
    }

    public void calcShooterSettings(Drive drive) {
        Pose2d robotPosition = drive.getPose();
        SpinUp(drive);
    }

    public void SpinUp(Drive drive) {
        Pose2d robotPosition = drive.getPose();

        Transform2d TurretTransform = new Transform2d(getTurretTranslation(), new Rotation2d(0));

        Transform2d initialShotDisplacement = drive.targetShotPose.minus(robotPosition.plus(TurretTransform));
        Translation2d adjustedDistance2d =
                calcDist(initialShotDisplacement, drive.getSpeed(), drive.getPose(), drive.targetShotPose, 0);
        double adjustedDistance = Math.hypot(adjustedDistance2d.getX(), adjustedDistance2d.getY());

        flywheelSpeed = calcFlywheelSpeed(adjustedDistance);
        hoodAngle = calcHoodAngle(adjustedDistance);

        // flywheelSpeed = MathUtil.clamp(flywheelSpeed, ShooterConstants.MIN_FLYWHEEL_VELOCITY,
        // ShooterConstants.MAX_FLYWHEEL_VELOCITY);
        // hoodAngle = MathUtil.clamp(hoodAngle, ShooterConstants.MIN_HOOD_ANGLE, ShooterConstants.MAX_HOOD_ANGLE);

        Logger.recordOutput("Calculated Flywheel Speed", flywheelSpeed);
        Logger.recordOutput("Calculated Hood Angle", hoodAngle);
        Logger.recordOutput("Init Distance to Hub", initialShotDisplacement);
        Logger.recordOutput("Final Dist to Hub", adjustedDistance);
    }

    public double evalPoly(double[] coeffs, double distance) {
        double sum = 0.0;

        for (int i = 0; i < coeffs.length; i++) {
            sum += coeffs[i] * Math.pow(distance, i);
        }
        return sum;
    }

    public Translation2d calcDist(
            Transform2d shotPrevDisplacement,
            double robotVelocity,
            Pose2d robotPosition,
            Pose2d targetPosition,
            int tries) {

        double flightTime = evalPoly(ShooterConstants.timeCoeffs, 0.0);
        double shotOffset = robotVelocity * flightTime;

        Translation2d shotLastDisplacement =
                shotPrevDisplacement.getTranslation().plus(new Translation2d(shotOffset, 0));
        Pose2d shotEndpoint = robotPosition.plus(new Transform2d(shotLastDisplacement, new Rotation2d(0)));
        Transform2d shotError = targetPosition.minus(shotEndpoint);

        double shotErrDouble = Math.hypot(shotError.getX(), shotError.getY());
        Rotation2d rotation = new Rotation2d(Math.toRadians(0)); // NO ROTATION
        Transform2d shotLastDisplacementTransform2d = new Transform2d(shotLastDisplacement, rotation);
        if (Math.abs(shotErrDouble) > ShooterConstants.THRESHOLD && tries < 7) {
            shotLastDisplacement = calcDist(
                    shotLastDisplacementTransform2d,
                    robotVelocity,
                    robotPosition,
                    targetPosition,
                    ++tries); // shotLastDisplacement needs to be a transform2d
        }

        // Convert shotLastDisplacement Transform2d to a double and return
        // double value = Math.sqrt( Math.pow(shotLastDisplacement.getX(),2) +
        // Math.pow(shotLastDisplacement.getY(),2));
        return shotLastDisplacement;
    }

    public Translation2d getTurretTranslation() {
        Translation2d turretOffset = new Translation2d(
                Constants.Vision.robotCenterToTurretCenter.getTranslation().getX(),
                Constants.Vision.robotCenterToTurretCenter.getTranslation().getY());
        Translation2d baseTranslation = drive.getPose().getTranslation().plus(turretOffset);
        Translation2d rotatedTranslation = baseTranslation.rotateAround(
                drive.getPose().getTranslation(), drive.getPose().getRotation());

        return rotatedTranslation;
    }

    public double calcFlywheelSpeed(double Distance) {
        double flywheelSpeed = evalPoly(ShooterConstants.flywheelCoeffs, Distance);
        return flywheelSpeed;
    }

    public double getFlywheelSpeed(double Distance) {
        double flywheelSpeed = evalPoly(ShooterConstants.flywheelCoeffs, Distance);
        return flywheelSpeed;
    }

    public double calcHoodAngle(double Distance) {
        double hoodAngle = evalPoly(ShooterConstants.hoodCoeffs, Distance);
        return hoodAngle;
    }

    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getHoodAngle(double Distance) {
        double hoodAngle = evalPoly(ShooterConstants.hoodCoeffs, Distance);
        return hoodAngle;
    }

    @Override
    public void periodic() {
        calcShooterSettings(drive);
    }

    @Override
    public void simulationPeriodic() {}
}
