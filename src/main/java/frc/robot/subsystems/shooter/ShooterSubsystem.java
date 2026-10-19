package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.Zone;
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

    public Command spinUp(Trigger override) {
        return Commands.parallel(
                flywheel.setVelocityProf(() -> flywheelSpeed),
                hood.setPosition(() -> hoodAngle, () -> inTrench(), override));
    }

    public Command spinUpShot(Trigger override) {
        return Commands.parallel(
                flywheel.setVelocity(() -> flywheelSpeed),
                hood.setPosition(() -> hoodAngle, () -> inTrench(), override));
    }


    public void calcShooterSettings(Drive drive) {
        Pose2d robotPosition = drive.getPose();
        SpinUp(drive);
    }

    public void SpinUp(Drive drive) {
        Pose2d targetPosition = drive.calcTargetZone();
        Pose2d adjustedTargetPosition = calcDist(targetPosition, 0);
        Translation2d adjustedTargetDisplacement =
                adjustedTargetPosition.getTranslation().minus(getShooterTranslation());
        drive.setTargetZone(adjustedTargetPosition);

        double adjustedDistance = adjustedTargetDisplacement.getNorm()
                * (1.0
                        + (Math.hypot(
                                        drive.fieldRelativeVelocity().vxMetersPerSecond,
                                        drive.fieldRelativeVelocity().vyMetersPerSecond))
                                * ShooterConstants.DISPLACEMENT_SCALER
                                * Math.signum(-adjustedTargetDisplacement.dot(new Translation2d(
                                        drive.fieldRelativeVelocity().vxMetersPerSecond,
                                        drive.fieldRelativeVelocity().vyMetersPerSecond))));

        flywheelSpeed = calcFlywheelSpeed(adjustedDistance);
        hoodAngle = calcHoodAngle(adjustedDistance);

        Logger.recordOutput("Adjusted Target", adjustedTargetPosition);
        Logger.recordOutput("Calculated Flywheel Speed", flywheelSpeed);
        Logger.recordOutput("Calculated Hood Angle", hoodAngle);
        Logger.recordOutput("Final Dist to Hub", adjustedDistance);
        Logger.recordOutput(
                "Percent Term",
                Math.hypot(
                                drive.fieldRelativeVelocity().vxMetersPerSecond,
                                drive.fieldRelativeVelocity().vyMetersPerSecond)
                        * ShooterConstants.DISPLACEMENT_SCALER
                        * Math.signum(-adjustedTargetDisplacement.dot(new Translation2d(
                                drive.fieldRelativeVelocity().vxMetersPerSecond,
                                drive.fieldRelativeVelocity().vyMetersPerSecond))));
    }

    public Translation2d getShooterDisplacementFromTarget() {
        return drive.targetShotPose.getTranslation().minus(getShooterTranslation());
    }

    public Translation2d getShooterTranslation() {
        Translation2d shooterOffset = new Translation2d(
                Constants.Vision.robotCenterToShooterCenter.getTranslation().getX(),
                Constants.Vision.robotCenterToShooterCenter.getTranslation().getY());
        Translation2d baseTranslation = drive.getPose().getTranslation().plus(shooterOffset);
        Translation2d rotatedTranslation = baseTranslation.rotateAround(
                drive.getPose().getTranslation(), drive.getPose().getRotation());

        return rotatedTranslation;
    }

    public boolean inTrench() {
        double posX = getShooterTranslation().getX();
        return posX < Constants.Field.TRENCH_THRESHOLD_BLUE && posX > Constants.Field.EDGE_OF_BLUE_ALLIANCE_ZONE
                || posX > Constants.Field.TRENCH_THRESHOLD_RED && posX < Constants.Field.EDGE_OF_RED_ALLIANCE_ZONE;
    }

    public double getShooterDistanceFromTarget() {
        return Math.hypot(
                getShooterDisplacementFromTarget().getX(),
                getShooterDisplacementFromTarget().getY());
    }

    public double evalPoly(double[] coeffs, double distance) {
        double sum = 0.0;

        for (int i = 0; i < coeffs.length; i++) {
            sum += coeffs[i] * Math.pow(distance, i);
        }
        return sum;
    }

    public Pose2d calcDist(Pose2d targetPosition, Pose2d shotEndpoint, int tries) {
        Translation2d targetDisplacement = shotEndpoint.getTranslation().minus(getShooterTranslation());
        double flightTime = evalPoly(ShooterConstants.timeCoeffs, targetDisplacement.getNorm());
        Translation2d shooterVelocity = new Translation2d(
                -getShooterTranslation().minus(drive.getPose().getTranslation()).getY()
                        * drive.fieldRelativeVelocity().omegaRadiansPerSecond,
                getShooterTranslation().minus(drive.getPose().getTranslation()).getX()
                        * drive.fieldRelativeVelocity().omegaRadiansPerSecond);
        if (drive.currentZone != Zone.CELTX_ALLIANCE) {
            shooterVelocity = shooterVelocity.times(0.5);
        }
        Translation2d shotOffset = (new Translation2d(
                        drive.fieldRelativeVelocity().vxMetersPerSecond,
                        drive.fieldRelativeVelocity().vyMetersPerSecond))
                // .plus(shooterVelocity))
                .times(-flightTime);
        Translation2d adjustedShotOffset = shotOffset.rotateBy(Rotation2d.kCW_90deg);
        shotEndpoint = targetPosition.plus(new Transform2d(adjustedShotOffset, new Rotation2d()));
        double shotError = shotEndpoint.minus(targetPosition).getTranslation().getNorm();

        Logger.recordOutput("Calc Dist/flightTime", flightTime);
        Logger.recordOutput("Calc Dist/shotOffset", shotOffset);
        Logger.recordOutput("Calc Dist/shotEndpoint", shotEndpoint);
        Logger.recordOutput("Calc Dist/shotError", shotError);

        if (!(drive.currentZone == Zone.CELTX_ALLIANCE))
            return new Pose2d(shotEndpoint.getX(), shotEndpoint.getY(), shotEndpoint.getRotation());
        else if (Math.abs(shotError) > ShooterConstants.THRESHOLD && tries < 5) {
            shotEndpoint = calcDist(targetPosition, shotEndpoint, ++tries);
        }

        return new Pose2d(shotEndpoint.getX(), shotEndpoint.getY(), shotEndpoint.getRotation());
    }

    public Pose2d calcDist(Pose2d targetPosition, int tries) {
        return calcDist(targetPosition, targetPosition, tries);
    }

    public double calcFlywheelSpeed(double distance) {
        double flywheelSpeed = distance < ShooterConstants.SNOWBLOW_DISTANCE
                ? evalPoly(ShooterConstants.flywheelCoeffsAlliance, distance)
                : evalPoly(ShooterConstants.flywheelCoeffsSnowblow, distance);
        return flywheelSpeed * ShooterConstants.FLYWHEEL_SPEED_SCALER;
    }

    public double calcHoodAngle(double distance) {
        if (distance > ShooterConstants.MAX_DISTANCE) return ShooterConstants.MAX_HOOD_CALCULATED_ANGLE;
        double hoodAngle = distance < ShooterConstants.SNOWBLOW_DISTANCE
                ? evalPoly(ShooterConstants.hoodCoeffsAlliance, distance)
                : evalPoly(ShooterConstants.hoodCoeffsSnowblow, distance);
        return hoodAngle;
    }

    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    @Override
    public void periodic() {
        calcShooterSettings(drive);
    }

    @Override
    public void simulationPeriodic() {}
}
