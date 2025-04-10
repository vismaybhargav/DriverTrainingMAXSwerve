// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.gyro.GyroIOMapleSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOMapleSim;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonPoseEstimator;
import frc.robot.subsystems.vision.VisionIOPhotonPoseEstimatorSim;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Features;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import static frc.robot.Constants.VisionConstants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final Drive robotDrive;
	private final Vision vision;

	// The simulation
	private SwerveDriveSimulation simulation = null;

	// The driver's controller
	CommandPS4Controller driverController = new CommandPS4Controller(OIConstants.driverControllerPort);
	PS4Controller driveControllerHID = driverController.getHID();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		if (Robot.isReal()) {
			robotDrive = new Drive(
					new GyroIONavX(),
					new ModuleIOSpark(0),
					new ModuleIOSpark(1),
					new ModuleIOSpark(2),
					new ModuleIOSpark(3),
					(pose) -> {
					});

			if (Features.PHOTON_VISION_POSE_ESTIMATOR_ENABLED) {
				vision = new Vision(
						robotDrive::addVisionMeasurement,
						new VisionIOPhotonPoseEstimator(REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM),
						new VisionIOPhotonPoseEstimator(STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM));
			} else {
				vision = new Vision(
						robotDrive::addVisionMeasurement,
						new VisionIOPhotonVision(REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM),
						new VisionIOPhotonVision(STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM));
			}

		} else if (Robot.isSimulation()) {
			if (Features.MAPLE_SIM_ENABLED) {
				simulation = new SwerveDriveSimulation(
						Constants.SimConstants.mapleSimConfig,
						new Pose2d(3, 3, new Rotation2d()));

				SimulatedArena.getInstance().addDriveTrainSimulation(simulation);

				var modules = simulation.getModules();

				robotDrive = new Drive(
						new GyroIOMapleSim(simulation.getGyroSimulation()),
						new ModuleIOMapleSim(modules[0]),
						new ModuleIOMapleSim(modules[1]),
						new ModuleIOMapleSim(modules[2]),
						new ModuleIOMapleSim(modules[3]),
						simulation::setSimulationWorldPose);

			} else {
				robotDrive = new Drive(
						new GyroIO() {},
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						(pose) -> {});
			}

			if (Features.PHOTON_VISION_POSE_ESTIMATOR_ENABLED) {
				vision = new Vision(
					robotDrive::addVisionMeasurement, 
					new VisionIOPhotonPoseEstimatorSim(REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM,
						robotDrive::getPose),
					new VisionIOPhotonPoseEstimatorSim(STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM, 
						robotDrive::getPose));
			} else {
				vision = new Vision(
						robotDrive::addVisionMeasurement,
						new VisionIOPhotonVisionSim(REEF_CAMERA_NAME, ROBOT_TO_REEF_CAM,
								robotDrive::getPose),
						new VisionIOPhotonVisionSim(STATION_CAMERA_NAME, ROBOT_TO_STATION_CAM,
								robotDrive::getPose));
			}
		} else {
			robotDrive = new Drive(
					new GyroIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					(pose) -> {});

			vision = new Vision(robotDrive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
		}

		// TODO: Setup auto routines

		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Configure default commands
		robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				DriveCommands.joystickDrive(
						robotDrive,
						() -> -MathUtil.applyDeadband(driveControllerHID.getLeftY(),
								OIConstants.driveDeadband),
						() -> -MathUtil.applyDeadband(driveControllerHID.getLeftX(),
								OIConstants.driveDeadband),
						() -> -MathUtil.applyDeadband(driveControllerHID.getRightX(),
								OIConstants.driveDeadband)));

		final Runnable resetGyro = Robot.isSimulation()
				? () -> robotDrive.resetOdometry(
						simulation.getSimulatedDriveTrainPose())
				: () -> robotDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));

		driverController.share().onTrue(Commands.runOnce(resetGyro, robotDrive).ignoringDisable(true));

        // if (AutoBuilder.isConfigured()) {
        //     // Add a button to run pathfinding commands to SmartDashboard
        //     SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        //             new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        //             new PathConstraints(
        //                     4.0, 4.0,
        //                     Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //             0));
        //     SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
        //             new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
        //             new PathConstraints(
        //                     4.0, 4.0,
        //                     Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //             0));

        // }
    }

	public void resetSimulationField() {
		if (!Robot.isSimulation() || simulation == null)
			return;

		robotDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	public void updateSimulation() {
		if (!Robot.isSimulation() || simulation == null)
			return;

		SimulatedArena.getInstance().simulationPeriodic();
		Logger.recordOutput("Field Simulation/Robot Pose", simulation.getSimulatedDriveTrainPose());
		Logger.recordOutput("Field Simulation/Coral",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
		Logger.recordOutput("Field Simulation/Algae",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
	}
}
