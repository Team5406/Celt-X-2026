/*

* Copyright (c) 2026 FRC 5406 - Team Celt-X.
*
* Open Source Software; you can modify and/or share it under the terms of
* the WPILib BSD license file in the root directory of this project.
*/
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.autos.FullSwipeBump;
import frc.robot.autos.FullSwipeTrench;
import frc.robot.autos.LeftTrench;
import frc.robot.autos.LeftTrench2;
import frc.robot.autos.LeftTrench2Win;
import frc.robot.autos.LeftTrenchBlow;
import frc.robot.autos.LeftTrenchDepot;
import frc.robot.autos.LeftTrenchWin;
import frc.robot.autos.Middle;
import frc.robot.autos.MiddleLeftBump;
import frc.robot.autos.MiddleRightBump;
import frc.robot.autos.RightTrench;
import frc.robot.autos.RightTrench2;
import frc.robot.autos.RightTrench2Win;
import frc.robot.autos.RightTrenchBlow;
import frc.robot.autos.RightTrenchOutpost;
import frc.robot.autos.RightTrenchWin;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AutoUnjamFeeding;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RampUp;
import frc.robot.commands.towerSwipe;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Indexer.feeder.FeederConstants;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.Indexer.feeder.FeederSubsystemIO;
import frc.robot.subsystems.Indexer.feeder.RealFeederSubsystemIO;
import frc.robot.subsystems.Indexer.feeder.SimulatedFeederSubsystemIO;
import frc.robot.subsystems.Indexer.rollerFloor.RealRollerFloorSubsystemIO;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorConstants;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.Indexer.rollerFloor.RollerFloorSubsystemIO;
import frc.robot.subsystems.Indexer.rollerFloor.SimulatedRollerFloorSubsystemIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotConstants.IntakeState;
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
    //  private final LightSubsystem light = createLight();
    private Vision vision = createVision();
    private HoodSubsystem hood = createHood();
    private IntakeRollerSubsystem roller = createRoller();
    private IntakePivotSubsystem pivot = createPivot();
    private FeederSubsystem feeder = createFeeder();
    private FlywheelSubsystem flywheel = createFlywheel();
    public DriveToPose alignmentCommand;
    public AlignToHub alignToHub;
    public RampUp rampUp;
    public Path path;
    private RollerFloorSubsystem floor = createFloor();
    private ShooterSubsystem shooter = createShooter();

    /*  private LightSubsystem createLight() {
            return switch (Constants.currentMode) {
                case REAL -> new LightSubsystem(new RealLightSubsystemIO());
                case SIM -> new LightSubsystem(new SimulatedLightSubsystemIO());
                default -> new LightSubsystem(new LightSubsystemIO() {});
            };
        }
    /* */
    private RollerFloorSubsystem createFloor() {
        return switch (Constants.currentMode) {
            case REAL -> new RollerFloorSubsystem(new RealRollerFloorSubsystemIO());
            case SIM -> new RollerFloorSubsystem(new SimulatedRollerFloorSubsystemIO());
            default -> new RollerFloorSubsystem(new RollerFloorSubsystemIO() {});
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
                        // Commenting out to prevent missing camera errors until cameras are
                        // wired
                        new RealPhotonVisionIO(
                                Constants.Vision.RIGHT_GLOBAL_POSE_CAMERA_NAME,
                                Constants.Vision.robotToCameraRight,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                true));
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
                                Constants.Vision.RIGHT_GLOBAL_POSE_CAMERA_NAME,
                                Constants.Vision.robotToCameraRight,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES,
                                true));
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

    private FeederSubsystem createFeeder() {
        return switch (Constants.currentMode) {
            case REAL -> new FeederSubsystem(new RealFeederSubsystemIO());
            case SIM -> new FeederSubsystem(new SimulatedFeederSubsystemIO());
            default -> new FeederSubsystem(new FeederSubsystemIO() {});
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

    private static SendableChooser<Command> t_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        alignmentCommand = new DriveToPose(drive);
        alignToHub = new AlignToHub(vision, drive);
        rampUp = new RampUp(pivot);
        SmartDashboard.putString("RunMode", String.valueOf(Constants.currentMode));
        SmartDashboard.putString("SysidMode", String.valueOf(Constants.SYSID_MODE));

        // Set up auto routines
        autoModeChooser();
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", m_chooser);

        // Set up SysId routines
        sysidChooser = new LoggedDashboardChooser<>("Sysid Choices", t_chooser);

        // Set up SysId routines

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
                "Sysid/Hood SysId (Dynamic Reverse)", hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Quasistatic Forward)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Quasistatic Reverse)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Dynamic Forward)", pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Pivot SysId (Dynamic Reverse)", pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Flywheel SysId (Quasistatic Forward)",
                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Flywheel SysId (Quasistatic Reverse)",
                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysidChooser.addOption(
                "Sysid/Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysidChooser.addOption(
                "Sysid/Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                        () -> Rotation2d.fromDegrees(180),
                        () -> floor.isShooting()));

        // light.setDefaultCommand(light.signal());
        roller.setDefaultCommand(roller.stop());
        pivot.setDefaultCommand(pivot.defaultCommand());
        floor.setDefaultCommand(floor.stop());
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
        controller
                .leftTrigger()
                .whileTrue(roller.setVelocity(-RollerConstants.ROLLER_ACTIVE_VELOCITY, pivot, controller.back()));
        controller
                .rightTrigger()
                .whileTrue(new ParallelCommandGroup(
                        roller.setVelocity(RollerConstants.ROLLER_ACTIVE_VELOCITY, pivot, controller.back()),
                        pivot.activePos(),
                        feeder.setVelocity(FeederConstants.FEEDER_INTAKING_VELOCITY),
                        floor.setVelocity(RollerFloorConstants.FLOOR_INTAKING_VELOCITY)));
        operatorController.a().whileTrue(pivot.moveDownVoltage());
        operatorController.a().and(operatorController.start()).onTrue(pivot.setEncoderDown());
        operatorController.b().onTrue(hood.zero());
        controller.leftBumper().whileTrue(pivot.stowedPos());
        controller.rightBumper().whileTrue(pivot.activePos());
        operatorController.rightBumper().whileTrue(new AutoUnjamFeeding(floor, feeder, flywheel, drive, vision));
        operatorController
                .rightBumper()
                .and(controller.rightTrigger().negate())
                .and(controller.x().negate())
                .whileTrue(new ParallelCommandGroup(
                        new AlignToHub(vision, drive).align(() -> -controller.getLeftY(), () -> -controller.getLeftX()),
                        roller.setVelocityOver(RollerConstants.ROLLER_SHOOT_VELOCITY, pivot),
                        pivot.shootingPos().beforeStarting(new WaitCommand(1))));

        operatorController
                .x()
                .whileTrue(new ParallelCommandGroup(flywheel.setFlywheelSpeedManual(), hood.setHoodAngleManual()));
        operatorController.rightTrigger().and(operatorController.rightBumper().negate()).whileTrue(shooter.spinUp(operatorController.back()));
        operatorController.rightTrigger().and(operatorController.rightBumper()).whileTrue(shooter.spinUpShot(operatorController.back()));
        controller.povDown().whileTrue(feeder.setPercent(-0.2));
        operatorController
                .leftBumper()
                .whileTrue(new ParallelCommandGroup(
                        feeder.setVelocity(-FeederConstants.FEEDER_FEEDING_VELOCITY),
                        floor.setVelocity(-RollerFloorConstants.FLOOR_VELOCITY)));
        operatorController
                .y()
                .whileTrue(new ParallelCommandGroup(
                        feeder.setVelocity(-FeederConstants.FEEDER_FEEDING_VELOCITY),
                        floor.setVelocity(RollerFloorConstants.FLOOR_VELOCITY * 0.7),
                        roller.setVelocity(-RollerConstants.ROLLER_ACTIVE_VELOCITY, pivot, controller.back())));
        towerSwipe swipe = new towerSwipe(drive, roller, pivot, alignmentCommand, path);
        controller.x().onTrue(swipe);
        controller
                .x()
                .onFalse(new InstantCommand(() -> swipe.cancel())); // controller.x().onFalse(new InstantCommand(()->
        // CommandScheduler.getInstance().cancel(towerSwipe)));
        controller.y().onTrue(AutoBuilder.resetOdom(new Pose2d(4.400, 0.547, new Rotation2d(0))));

        operatorController
                .leftTrigger()
                .whileTrue(new ParallelCommandGroup(
                        flywheel.setVelocity(FlywheelConstants.FLYWHEEL_FIXED_SHOT_VELOCITY),
                        hood.setPosition(
                                () -> HoodConstants.HOOD_FIXED_SHOT_ANGLE,
                                () -> shooter.inTrench(),
                                operatorController.back())));

        // controller.x().whileTrue(new towerSwipe(drive, roller, pivot, alignmentCommand));

        // controller.x().onFalse(new InstantComma  nd(()->
        // CommandScheduler.getInstance().cancel(towerSwipe)));
        /*** END TESTING CONTROLS ***/
    }

    private void autoModeChooser() {
        m_chooser.setDefaultOption("0 - Do Nothing", new SequentialCommandGroup());
        m_chooser.addOption(
                "1 - Middle", new Middle(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "2 - RightTrench",
                new RightTrench(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "3 - RightTrenchOutpost",
                new RightTrenchOutpost(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "4 - LeftTrench", new LeftTrench(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "5 - LeftTrenchDepot",
                new LeftTrenchDepot(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "6 - RightTrench2",
                new RightTrench2(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "7 - LeftTrench2",
                new LeftTrench2(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "8 - RightTrenchWin",
                new RightTrenchWin(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "9 - LeftTrenchWin",
                new LeftTrenchWin(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "10 - RightTrench2Win",
                new RightTrench2Win(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "11 - LeftTrench2Win",
                new LeftTrench2Win(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "12 - FullSwipeTrench",
                new FullSwipeTrench(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "13 - FullSwipeBump",
                new FullSwipeBump(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "14 - RightTrenchSnowBlow",
                new RightTrenchBlow(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "15 - LeftTrenchSnowBlow",
                new LeftTrenchBlow(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "16 - MiddleRightBump",
                new MiddleRightBump(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
        m_chooser.addOption(
                "17 - MiddleLeftBump",
                new MiddleLeftBump(drive, hood, flywheel, feeder, roller, pivot, floor, vision, shooter));
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
        hood.syncEncoder();
    }

    public void stopWhenDisabled() {}

    public void setDriveCoast() {
        // drive.setCoast();
    }

    public void setCurrentLimit() {
        drive.setCurrentLimit();
        roller.setCurrentLimit();
    }

    public void zeroMechanismsAuto() {
        pivot.setEncoders(IntakePivotConstants.PIVOT_ZEROED_UP_POSITION);
        pivot.setState(IntakeState.STOWED);
        hood.syncEncoder();
    }

    public void printThing() {
        alignToHub.log();
    }
}
