/*
 * Copyright (c) 2026 FRC 5406 - Team Celt-X.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.AlignToHub;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.Feeding;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystemIO;
import frc.robot.subsystems.intake.pivot.RealIntakePivotSubsystemIO;
import frc.robot.subsystems.intake.pivot.SimulatedIntakePivotSubsystemIO;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.rollers.IntakeRollerSubsystemIO;
import frc.robot.subsystems.intake.rollers.RealIntakeRollerSubsystemIO;
import frc.robot.subsystems.intake.rollers.RollerConstants;
import frc.robot.subsystems.intake.rollers.SimulatedIntakeRollerSubsystemIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIO;
import frc.robot.subsystems.shooter.flywheel.RealFlywheelSubsystemIO;
import frc.robot.subsystems.shooter.flywheel.SimulatedFlywheelSubsystemIO;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIO;
import frc.robot.subsystems.shooter.hood.RealHoodSubsystemIO;
import frc.robot.subsystems.shooter.hood.SimulatedHoodSubsystemIO;
import frc.robot.subsystems.shooter.turret.RealTurretSubsystemIO;
import frc.robot.subsystems.shooter.turret.SimulatedTurretSubsystemIO;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystemIO;
import frc.robot.subsystems.spindexer.feeder.FeederConstants;
import frc.robot.subsystems.spindexer.feeder.RealSpindexerFeederSubsystemIO;
import frc.robot.subsystems.spindexer.feeder.SimulatedSpindexerFeederSubsystemIO;
import frc.robot.subsystems.spindexer.feeder.SpindexerFeederSubsystem;
import frc.robot.subsystems.spindexer.feeder.SpindexerFeederSubsystemIO;
import frc.robot.subsystems.spindexer.serializer.RealSerializerSubsystemIO;
import frc.robot.subsystems.spindexer.serializer.SerializerConstants;
import frc.robot.subsystems.spindexer.serializer.SerializerSubsystem;
import frc.robot.subsystems.spindexer.serializer.SerializerSubsystemIO;
import frc.robot.subsystems.spindexer.serializer.SimulatedSerializerSubsystemIO;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive = createDrive();
    // private final LightSubsystem light = createLight();
    private Vision vision = createVision();
    private TurretSubsystem turret = createTurret();
    private HoodSubsystem hood = createHood();
    private IntakeRollerSubsystem roller = createRoller();
    private IntakePivotSubsystem pivot = createPivot();
    private SpindexerFeederSubsystem feeder = createFeeder();
    private FlywheelSubsystem flywheel = createFlywheel();
    public DriveToPose alignmentCommand;
    public AlignToHub alignToHub;
    private SerializerSubsystem serializer = createSerializer();
    private ShooterSubsystem shooter = createShooter();
    private SpinUp spinUpCommand = new SpinUp(hood, flywheel, shooter);
    private Shoot shootCommand = new Shoot(drive, feeder, serializer);

    // private LightSubsystem createLight() {
    // return switch (Constants.currentMode) {
    // case REAL -> new LightSubsystem(new RealLightSubsystemIO());
    // case SIM -> new LightSubsystem(new SimulatedLightSubsystemIO());
    // default -> new LightSubsystem(new LightSubsystemIO() {});
    // };
    // }

    private TurretSubsystem createTurret() {
        return switch (Constants.currentMode) {
            case REAL -> new TurretSubsystem(new RealTurretSubsystemIO());
            case SIM -> new TurretSubsystem(new SimulatedTurretSubsystemIO());
            default -> new TurretSubsystem(new TurretSubsystemIO() {});
        };
    }

    private SerializerSubsystem createSerializer() {
        return switch (Constants.currentMode) {
            case REAL -> new SerializerSubsystem(new RealSerializerSubsystemIO());
            case SIM -> new SerializerSubsystem(new SimulatedSerializerSubsystemIO());
            default -> new SerializerSubsystem(new SerializerSubsystemIO() {});
        };
    }

    private Vision createVision() {
        switch (Constants.currentMode) {
            case REAL:
                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new RealPhotonVisionIO(
                                Constants.Vision.BACK_GLOBAL_POSE_CAMERA_NAME,
                                Constants.Vision.robotToCameraBack,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                true),
                        new RealPhotonVisionIO(
                                Constants.Vision.LEFT_GLOBAL_POSE_CAMERA_NAME,
                                Constants.Vision.robotToCameraLeft,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                true),
                        new RealPhotonVisionIO(
                                Constants.Vision.TURRET_POSE_CAMERA_NAME,
                                Constants.Vision.robotCenterToTurretCenter,
                                // turret.getCameraDisplacement(),
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                false));
            case SIM:
                VisionEnvironmentSimulator.getInstance().addAprilTags(Constants.Field.APRIL_TAG_FIELD_LAYOUT);
                VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(drive.getRobotPoseSupplier());

                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new SimulatedPhotonVisionIO(
                                Constants.Vision.BACK_GLOBAL_POSE_CAMERA_NAME,
                                Constants.Vision.robotToCameraBack,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES,
                                true),
                        new SimulatedPhotonVisionIO(
                                Constants.Vision.LEFT_GLOBAL_POSE_CAMERA_NAME,
                                Constants.Vision.robotToCameraLeft,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES,
                                true),
                        new SimulatedPhotonVisionIO(
                                Constants.Vision.TURRET_POSE_CAMERA_NAME,
                                new Transform3d(),
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES,
                                false));
            default:
                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new ReplayVisionIO(Constants.Vision.LEFT_GLOBAL_POSE_CAMERA_NAME),
                        new ReplayVisionIO(Constants.Vision.BACK_GLOBAL_POSE_CAMERA_NAME));
        }
    }

    private HoodSubsystem createHood() {
        return switch (Constants.currentMode) {
            case REAL -> new HoodSubsystem(new RealHoodSubsystemIO());
            case SIM -> new HoodSubsystem(new SimulatedHoodSubsystemIO());
            default -> new HoodSubsystem(new HoodSubsystemIO() {});
        };
    }

    private FlywheelSubsystem createFlywheel() {
        return switch (Constants.currentMode) {
            case REAL -> new FlywheelSubsystem(new RealFlywheelSubsystemIO());
            case SIM -> new FlywheelSubsystem(new SimulatedFlywheelSubsystemIO());
            default -> new FlywheelSubsystem(new FlywheelSubsystemIO() {});
        };
    }

    private SpindexerFeederSubsystem createFeeder() {
        return switch (Constants.currentMode) {
            case REAL -> new SpindexerFeederSubsystem(new RealSpindexerFeederSubsystemIO());
            case SIM -> new SpindexerFeederSubsystem(new SimulatedSpindexerFeederSubsystemIO());
            default -> new SpindexerFeederSubsystem(new SpindexerFeederSubsystemIO() {});
        };
    }

    private IntakePivotSubsystem createPivot() {
        return switch (Constants.currentMode) {
            case REAL -> new IntakePivotSubsystem(new RealIntakePivotSubsystemIO());
            case SIM -> new IntakePivotSubsystem(new SimulatedIntakePivotSubsystemIO());
            default -> new IntakePivotSubsystem(new IntakePivotSubsystemIO() {});
        };
    }

    private IntakeRollerSubsystem createRoller() {
        return switch (Constants.currentMode) {
            case REAL -> new IntakeRollerSubsystem(new RealIntakeRollerSubsystemIO());
            case SIM -> new IntakeRollerSubsystem(new SimulatedIntakeRollerSubsystemIO());
            default -> new IntakeRollerSubsystem(new IntakeRollerSubsystemIO() {});
        };
    }

    private ShooterSubsystem createShooter() {
        return switch (Constants.currentMode) {
                // case REAL -> new ShooterSubsystem(new RealShooterSubsystemIO());
                // case SIM -> new ShooterSubsystem(new SimulatedShooterSubsystemIO());
            default -> new ShooterSubsystem(drive, hood, flywheel) {};
        };
    }

    private void trackCameras() {
        vision = createVision();
    }

    // Controller
    private final CommandXboxController controller = new CommandXboxController(1);
    private final CommandXboxController operatorController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> sysidChooser;
    private final LoggedDashboardChooser<Command> autoChooser;

    private static SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        alignmentCommand = new DriveToPose(drive);
        alignToHub = new AlignToHub(vision, drive, turret);
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

        sysidChooser.addOption(
                "Sysid/Hood SysId (Quasistatic Forward)", hood.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Hood SysId (Quasistatic Reverse)", hood.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Hood SysId (Dynamic Forward)", hood.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Turret SysId (Quasistatic Forward)", turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Turret SysId (Quasistatic Reverse)", turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Turret SysId (Dynamic Forward)", turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Turret SysId (Dynamic Reverse)", turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Hood SysId (Dynamic Reverse)", hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Quasistatic Forward)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Quasistatic Reverse)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Dynamic Forward)", pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Dynamic Reverse)", pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the triggers
        configureBindings();

        // Keeps track of the cameras
        trackCameras();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the
     * named factories in {@link CommandGenericHID}'s subclasses for
     * {@link CommandXboxController
     * Xbox}/{@link CommandPS4Controller PS4} controllers or {@link CommandJoystick
     * Flight joysticks}.
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

        // light.setDefaultCommand(light.signal());
        roller.setDefaultCommand(roller.stop());
        pivot.setDefaultCommand(pivot.activePosition());

        serializer.setDefaultCommand(serializer.stop());
        feeder.setDefaultCommand(feeder.stop());
        flywheel.setDefaultCommand(flywheel.stop());
        // Add the process vision command as default command for vision
        vision.setDefaultCommand(
                vision.processVision(drive.getRobotPoseSupplier(), drive.getVisionMeasurementConsumer())
                        .ignoringDisable(true));

        // Always keep Hood in stow position
        hood.setDefaultCommand(hood.stow());

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

        // Manual Turret Override
        operatorController
                .leftTrigger()
                .and(operatorController.axisMagnitudeGreaterThan(
                        XboxController.Axis.kRightX.value, TurretConstants.TURRET_DEADBAND))
                .whileTrue(turret.setPercent(() -> -operatorController.getRightX()));
        // Hold Position during override
        operatorController
                .leftTrigger()
                .and(operatorController
                        .axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, TurretConstants.TURRET_DEADBAND)
                        .negate())
                .whileTrue(turret.setPercent(() -> 0.0));

        // Manual Hood Override
        operatorController
                .leftTrigger()
                .and(operatorController.axisMagnitudeGreaterThan(
                        XboxController.Axis.kLeftY.value, HoodConstants.HOOD_DEADBAND))
                .whileTrue(hood.setPercent(() -> -operatorController.getLeftY()));
        // Hold Position during override
        operatorController
                .leftTrigger()
                .and(operatorController
                        .axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, HoodConstants.HOOD_DEADBAND)
                        .negate())
                .whileTrue(hood.setPercent(() -> 0.0));

        controller.leftTrigger().whileTrue(roller.setVelocity(-RollerConstants.ROLLER_ACTIVE_VELOCITY));
        controller.rightTrigger().whileTrue(roller.setVelocity(RollerConstants.ROLLER_ACTIVE_VELOCITY));
        controller.a().whileTrue(pivot.zero());
        controller.leftBumper().whileTrue(pivot.setPosition(IntakePivotConstants.PIVOT_STOW_POSITION));
        controller.rightBumper().whileTrue(pivot.setPosition(IntakePivotConstants.PIVOT_ACTIVE_POSITION));
        operatorController.rightBumper().whileTrue(new Feeding(feeder, serializer));
        operatorController.leftStick().whileTrue(turret.setPercent(() -> operatorController.getLeftX()));

        operatorController
                .rightTrigger()
                .and(controller.rightTrigger().negate())
                .whileTrue(new ParallelCommandGroup(
                        shooter.spinUp(),
                        new AlignToHub(vision, drive, turret).align()));

        operatorController
                .rightTrigger()
                .and(controller.rightTrigger())
                .whileTrue(new ParallelCommandGroup(shooter.spinUp(), new AlignToHub(vision, drive, turret).align()));

        controller.povDown().whileTrue(feeder.setPercent(-0.2));
        operatorController
                .leftBumper()
                .whileTrue(new ParallelCommandGroup(
                        feeder.setVelocity(-FeederConstants.FEEDER_FEEDING_VELOCITY),
                        serializer.setVelocity(-SerializerConstants.SERIALIZER_VELOCITY)));

        operatorController
                .leftTrigger()
                .whileTrue(new ParallelCommandGroup(
                        flywheel.setVelocity(FlywheelConstants.FLYWHEEL_FIXED_SHOT_VELOCITY),
                        hood.setPosition(HoodConstants.HOOD_FIXED_SHOT_ANGLE)));
        /*** END TESTING CONTROLS ***/
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

    // public void setAnimation() {
    // CommandScheduler.getInstance().schedule(light.setAnimation().ignoringDisable(true));
    // }

    // public void stopAnimation() {
    // CommandScheduler.getInstance().schedule(light.stopAnimation());
    // }

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

    public void zeroMechanisms() {
        pivot.setEncoder(IntakePivotConstants.PIVOT_ZEROED_POSITION);
    }

    public void stopWhenDisabled() {}

    public void setDriveCoast() {
        // drive.setCoast();
    }

    public void setCurrentLimit() {
        // drive.setCurrentLimit();
    }

    public void zeroMechanismsAuto() {}

    public void printThing() {
        alignToHub.log();
    }
}
