/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.auto.AutoSelector;
import frc.lib.input.CommandSwitchProController;
//import frc.lib.simulation.VisionEnvironmentSimulator;
import frc.lib.subsystems.drive.*;
//import frc.lib.subsystems.vision.RealPhotonVisionIO;
import frc.lib.subsystems.vision.ReplayVisionIO;
import frc.robot.autos.Autos;
//import frc.robot.commands.Eject;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.superstructure.SuperstructureIOSpark;

import java.lang.reflect.Field;
import java.util.*;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drivetrain;
    // private final Vision vision;
    // private final ExampleSubsystem exampleSubsystem;
    private final AutoSelector autoSelector;
    private final Superstructure superstructure;
    // Controller
    private final CommandXboxController controller = new CommandXboxController(1);
private final CommandXboxController operator = new CommandXboxController(0);

    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drivetrain = createDrive();
        superstructure = new Superstructure(new SuperstructureIOSpark());
        // Configure the triggergs
        configureBindings();
        autoSelector = new AutoSelector(Autos.class, getSubsystemList());

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in
     * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController
     * Xbox}/{@link CommandPS4Controller PS4}
     * controllers or {@link CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.joystickDrive(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                Constants.CONTROLLER_DEADBAND));

        controller.start().onTrue(drivetrain.resetRobotRotation());

        controller.rightTrigger().whileTrue(superstructure.intake());
        controller.leftTrigger().whileTrue(superstructure.eject());
        operator.rightBumper().whileTrue(superstructure.launch());
        operator.leftBumper().whileTrue(superstructure.spinup());
        operator.leftTrigger().whileTrue(superstructure.outtake());
        /*controller.rightTrigger().whileTrue(
            eject.set(0));*/

        // Add the process vision command as default command for vision
        /*
         * vision.setDefaultCommand(
         * vision.processVision(drivetrain.getRobotPoseSupplier(),
         * drivetrain.getVisionMeasurementConsumer())
         * .ignoringDisable(true));
         * 
         * exampleSubsystem.setDefaultCommand(exampleSubsystem.disable());
         * controller
         * .a()
         * .onTrue(exampleSubsystem
         * .goToHighSetpoint()
         * .until(exampleSubsystem.onPositionSetpoint().debounce(0.1)));
         * controller
         * .b()
         * .onTrue(exampleSubsystem
         * .goToLowSetpoint()
         * .until(exampleSubsystem.onPositionSetpoint().debounce(0.1)));
         * controller.x().whileTrue(exampleSubsystem.enableForward());
         * controller.y().whileTrue(exampleSubsystem.enableBackward());
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoSelector.getAutonomousCommand();
    }

    private Drive createDrive() {
        return switch (Constants.MODE) {
            case REAL -> new SwerveDrive(
                    WHEEL_BASE,
                    TRACK_WIDTH,
                    MAX_VELOCITY,
                    MAX_ACCELERATION,
                    Constants.LOOPER_DT,
                    ODOMETRY_FREQUENCY,
                    new Pigeon2IMUIO(CAN_ID_IMU),
                    new SparkSwerveModuleIO(
                            new SparkMax(CAN_ID_FRONT_LEFT_DRIVE, MotorType.kBrushless),
                            new SparkMax(CAN_ID_FRONT_LEFT_TURN, MotorType.kBrushless),
                            new CANcoder(CAN_ID_FRONT_LEFT_CANCODER),
                            DRIVE_CONFIG_FRONT_LEFT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF),
                    new SparkSwerveModuleIO(
                            new SparkMax(CAN_ID_FRONT_RIGHT_DRIVE, MotorType.kBrushless),
                            new SparkMax(CAN_ID_FRONT_RIGHT_TURN, MotorType.kBrushless),
                            new CANcoder(CAN_ID_FRONT_RIGHT_CANCODER),
                            DRIVE_CONFIG_FRONT_RIGHT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF),
                    new SparkSwerveModuleIO(
                            new SparkMax(CAN_ID_BACK_LEFT_DRIVE, MotorType.kBrushless),
                            new SparkMax(CAN_ID_BACK_LEFT_TURN, MotorType.kBrushless),
                            new CANcoder(CAN_ID_BACK_LEFT_CANCODER),
                            DRIVE_CONFIG_BACK_LEFT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF),
                    new SparkSwerveModuleIO(
                            new SparkMax(CAN_ID_BACK_RIGHT_DRIVE, MotorType.kBrushless),
                            new SparkMax(CAN_ID_BACK_RIGHT_TURN, MotorType.kBrushless),
                            new CANcoder(CAN_ID_BACK_RIGHT_CANCODER),
                            DRIVE_CONFIG_BACK_RIGHT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF));
            case SIM -> new SwerveDrive(
                    WHEEL_BASE,
                    TRACK_WIDTH,
                    MAX_VELOCITY,
                    MAX_ACCELERATION,
                    Constants.LOOPER_DT,
                    ODOMETRY_FREQUENCY,
                    new IMUIO() {
                    },
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO());
            case REPLAY -> new SwerveDrive(
                    WHEEL_BASE,
                    TRACK_WIDTH,
                    MAX_VELOCITY,
                    MAX_ACCELERATION,
                    Constants.LOOPER_DT,
                    ODOMETRY_FREQUENCY,
                    new IMUIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    },
                    new SwerveModuleIO() {
                    });
        };
    }

    /*
     * private Vision createVision() {
     * 
     * switch (Constants.MODE) {
     * case REAL:
     * return new Vision(
     * Constants.Vision.VISION_FILTER_PARAMETERS,
     * new RealPhotonVisionIO(
     * "camera1", Constants.Vision.robotToCamera1,
     * Constants.Field.APRIL_TAG_FIELD_LAYOUT));
     * case SIM:
     * VisionEnvironmentSimulator.getInstance()
     * .addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
     * VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(drivetrain.
     * getRobotPoseSupplier());
     * 
     * return new Vision(
     * Constants.Vision.VISION_FILTER_PARAMETERS,
     * new SimulatedPhotonVisionIO(
     * "camera1",
     * Constants.Vision.robotToCamera1,
     * Constants.Field.APRIL_TAG_FIELD_LAYOUT,
     * Constants.Vision.CAMERA_SIM_PROPERTIES));
     * default:
     * return new Vision(Constants.Vision.VISION_FILTER_PARAMETERS, new
     * ReplayVisionIO("camera1"));
     * }
     * }
     */

    /**
     * Detect which fields (variables) are declared in this class that extend
     * SubsystemBase. The detected types are
     * returned as a list
     *
     * @return List of subsystems (SubsystemBase objects)
     */
    private List<SubsystemBase> getSubsystemList() {
        List<SubsystemBase> subsystemList = new ArrayList<>();
        for (Field field : RobotContainer.class.getDeclaredFields()) {
            if (SubsystemBase.class.isAssignableFrom(field.getType())) {
                try {
                    subsystemList.add((SubsystemBase) field.get(this));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
        return subsystemList;
    }

}