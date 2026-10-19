package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;

public class TurnToAngle {

    private double pidMargin;
    private double setpointMargin;
    private double rotationKP;
    private boolean isInPidMargin;
    private boolean isInSetpointMargin;
    private double delta;
    private PIDController pid;

    /**
     * Construct a new TurnToPose object to determine the rotational speed to look at a certain position
     * @param marginPid The margin in degrees within which the desired speed is 0
     * @param kP The constant with which the difference in angle is multiplied to determine the speed
     */
    public TurnToAngle(double marginPid, double marginSetpoint, double kP, double kD) {
        pidMargin = marginPid;
        setpointMargin = marginSetpoint;

        rotationKP = kP;

        pid = new PIDController(kP, 0, kD);
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(marginPid);
    }

    public double getTargetSpeeds(Rotation2d currentAngle, Rotation2d targetAngle) {

        // determines how much the robots needs to rotate
        Rotation2d rotateToTarget = targetAngle.minus(currentAngle);
        delta = rotateToTarget.getRadians();

        SmartDashboard.putNumber("error/error ratate", rotateToTarget.getDegrees());

        double rotationSpeed = 0;
        isInPidMargin = (rotateToTarget.getDegrees() <= pidMargin) && (rotateToTarget.getDegrees() >= -pidMargin);
        if (!isInPidMargin) {
            // Pid control
            rotationSpeed = pid.calculate(currentAngle.getDegrees(), targetAngle.getDegrees());
            // Feedforward control using simple trig
            rotationSpeed = MathUtil.clamp(
                    rotationSpeed, -DriveConstants.AUTO_AIM_MAX_ROT_VEL, DriveConstants.AUTO_AIM_MAX_ROT_VEL);
        }
        isInSetpointMargin = (rotateToTarget.getDegrees() <= setpointMargin)
                && (rotateToTarget.getDegrees() >= -setpointMargin)
                && Math.abs(rotationSpeed) < DriveConstants.AUTO_AIM_MAX_VEL_SETPOINT;

        SmartDashboard.putNumber("error/kp", rotationKP);
        SmartDashboard.putNumber("error/delta", delta);
        SmartDashboard.putNumber("error/getDegree", rotateToTarget.getDegrees());
        SmartDashboard.putNumber("error/omegaRadians", rotationSpeed);
        SmartDashboard.putBoolean("error/isinmargin", isInPidMargin);
        return rotationSpeed;
    }

    public boolean isInSetpointMargin() {
        return isInSetpointMargin;
    }

    public double delta() {
        return delta;
    }

    public boolean isInPidMargin() {
        return isInPidMargin;
    }
}
