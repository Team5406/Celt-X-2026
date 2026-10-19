/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.simulation.VisionEnvironmentSimulator;
import frc.lib.subsystems.vision.RealPhotonVisionIO;
import frc.lib.subsystems.vision.ReplayVisionIO;
import frc.lib.subsystems.vision.SimulatedPhotonVisionIO;
import frc.lib.subsystems.vision.Vision;
import frc.robot.autos.DriveStraight;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.LightSubsystem.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.LightSubsystemIO;
import frc.robot.subsystems.LightSubsystem.RealLightSubsystemIO;
import frc.robot.subsystems.LightSubsystem.SimulatedLightSubsystemIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive = createDrive();
    private final LightSubsystem light = createLight();
    private final Vision vision = createVision();
    public DriveToPose alignmentCommand;

    private LightSubsystem createLight() {
        return switch (Constants.currentMode) {
            case REAL -> new LightSubsystem(new RealLightSubsystemIO());
            case SIM -> new LightSubsystem(new SimulatedLightSubsystemIO());
            default -> new LightSubsystem(new LightSubsystemIO() {});
        };
    }

    private Vision createVision() {
        switch (Constants.currentMode) {
            case REAL:
                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new RealPhotonVisionIO(
                                Constants.Vision.GLOBAL_POSE_CAMERA,
                                Constants.Vision.robotToCameraFront,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                true),
                        new RealPhotonVisionIO(
                                Constants.Vision.ALIGNMENT_POSE_CAMERA,
                                Constants.Vision.robotToCameraBack,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                false));
            case SIM:
                VisionEnvironmentSimulator.getInstance().addAprilTags(Constants.Field.APRIL_TAG_FIELD_LAYOUT);
                VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(drive.getRobotPoseSupplier());

                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new SimulatedPhotonVisionIO(
                                Constants.Vision.GLOBAL_POSE_CAMERA,
                                Constants.Vision.robotToCameraFront,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES,
                                true),
                        new SimulatedPhotonVisionIO(
                                Constants.Vision.ALIGNMENT_POSE_CAMERA,
                                Constants.Vision.robotToCameraBack,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES,
                                false));
            default:
                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new ReplayVisionIO(Constants.Vision.GLOBAL_POSE_CAMERA));
        }
    }

    // Controller
    private final CommandXboxController controller = new CommandXboxController(1);
    private final CommandXboxController operatorController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> sysidChooser;
    private final LoggedDashboardChooser<Command> autoChooser;

    private static SendableChooser<Command> m_chooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        alignmentCommand = new DriveToPose(drive);

        SmartDashboard.putString("RunMode", String.valueOf(Constants.currentMode));
        SmartDashboard.putString("SysidMode", String.valueOf(Constants.SYSID_MODE));

        // Set up auto routines
        autoModeChooser();
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", m_chooser);

        // Set up SysId routines
        sysidChooser = new LoggedDashboardChooser<>("Sysid Choices", m_chooser);

        sysidChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        sysidChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        sysidChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the triggers
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link CommandGenericHID}'s subclasses for {@link CommandXboxController
     * Xbox}/{@link CommandPS4Controller PS4} controllers or {@link CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
                        * (controller.rightStick().getAsBoolean() ? 1 : DriveConstants.DRIVE_TURN_MODIFIER)));

        controller
                .povUp()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> Rotation2d.fromDegrees(180)));

        light.setDefaultCommand(light.signal());

        // Add the process vision command as default command for vision
        vision.setDefaultCommand(
                vision.processVision(drive.getRobotPoseSupplier(), drive.getVisionMeasurementConsumer())
                        .ignoringDisable(true));

        controller
                .start()
                .onTrue(Commands.runOnce(
                                () -> {
                                    boolean isFlipped = DriverStation.getAlliance()
                                                    .isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                                    if (isFlipped) {
                                        drive.setPose(new Pose2d(
                                                drive.getPose().getTranslation(),
                                                new Rotation2d().plus(new Rotation2d(Math.PI))));
                                    } else {
                                        drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                                    }
                                    drive.syncCANcoders();
                                },
                                drive)
                        .ignoringDisable(true));
    }

    private void autoModeChooser() {
        m_chooser.setDefaultOption("0 - Do Nothing", new SequentialCommandGroup());
        m_chooser.addOption("1 - DriveStraight", new DriveStraight());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void setAnimation() {
        CommandScheduler.getInstance().schedule(light.setAnimation().ignoringDisable(true));
    }

    public void stopAnimation() {
        CommandScheduler.getInstance().schedule(light.stopAnimation());
    }

    public void stopMotors() {
        drive.stop();
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue;
    }

    private Drive createDrive() {

        return switch (Constants.currentMode) {
            case REAL ->
            // Real robot, instantiate hardware IO implementations
            new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3));

            case SIM ->
            // Sim robot, instantiate physics sim IO implementations
            new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());

            default ->
            // Replayed robot, disable IO implementations
            new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        };
    }

    public void zeroMechanisms() {}

    public void stopWhenDisabled() {}

    public void setDriveCoast() {
        // drive.setCoast();
    }

    public void setCurrentLimit() {
        // drive.setCurrentLimit();
    }

    public void zeroMechanismsAuto() {}
}
