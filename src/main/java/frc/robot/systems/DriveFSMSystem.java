package frc.robot.systems;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.input.TeleopInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.vision.RaspberryPi;

import jdk.jshell.spi.ExecutionControl;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;

import static edu.wpi.first.units.Units.Meter;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_TAG_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private final SwerveDrive swerveDrive;

	private final RaspberryPi rpi;

	private boolean tagPositionAligned = false;
	private Translation2d alignmentTranslation2d = null;
	private double rotationCache2d = 0;
	private Rotation2d rotationAlignmentPose = new Rotation2d();
	private int tagID = -1;

	// Auto PIDS
	private final PIDController xController = new PIDController(5, 0, 0);
	private final PIDController yController = new PIDController(5, 0, 0);
	private final PIDController headingController = new PIDController(0.75, 0, 0);

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem(File directory) {

		boolean blueAlliance = false;
		Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
				Meter.of(4)),
				Rotation2d.fromDegrees(0))
				: new Pose2d(new Translation2d(Meter.of(16),
				Meter.of(4)),
				Rotation2d.fromDegrees(180));

		// Perform hardware init
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
		try {
			// Hardware devices should be owned by one and only one system. They must
			// be private to their owner system and may not be used elsewhere.
			double maxSpeed = 5.41;
			swerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed, startingPose);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		swerveDrive.setHeadingCorrection(false);
		swerveDrive.setCosineCompensator(Robot.isReal());
		swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
		swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
		rpi = new RaspberryPi();

		headingController.enableContinuousInput(-Math.PI, Math.PI);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * 
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.TELEOP_STATE;
		zeroHeading();

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null)
			return;

		switch (currentState) {
			case TELEOP_STATE -> {
				// Reset all the auto logic when we go into the teleop state.
				if (alignmentTranslation2d != null) {
					alignmentTranslation2d = null;
					rotationCache2d = 0;
					tagPositionAligned = false;
				}
				handleTeleopState(input);
			}
			case ALIGN_TO_TAG_STATE -> handleAlignToTagState();
			default -> throw new IllegalStateException("Invalid state: " + currentState);
		}

		Logger.recordOutput("DriveFSM/Current State", currentState);
		Logger.recordOutput("DriveFSM/TeleOp/Swerve States", getModuleStates());

		swerveDrive.updateOdometry();

		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * 
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous() throws ExecutionControl.NotImplementedException {
		throw new ExecutionControl.NotImplementedException("TODO: Implement autonomous mode");
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		return switch (currentState) {
			case TELEOP_STATE -> {
				if (input.getDriveControllerAlignToTagPressed()) {
					yield FSMState.ALIGN_TO_TAG_STATE;
				} else {
					yield FSMState.TELEOP_STATE;
				}
			}
			case ALIGN_TO_TAG_STATE -> {
				if (!input.getDriveControllerAlignToTagPressed()) {
					yield FSMState.TELEOP_STATE;
				} else {
					yield FSMState.ALIGN_TO_TAG_STATE;
				}
			}
			default -> throw new IllegalStateException("Invalid state: " + currentState);
		};
	}

	/* ------------------------ FSM state handlers ------------------------ */
	private void handleTeleopState(TeleopInput input) {
		// Get joystick inputs
		var xInput = input.getDriveControllerLeftY(); // Up and down on the left stick
		var yInput = input.getDriveControllerLeftX(); // Left and right on the left stick
		var rotInput = input.getDriveControllerRightX(); // Left and right on the right stick

		Logger.recordOutput("DriveFSM/TeleOp/Inputs/X-Input", xInput);
		Logger.recordOutput("DriveFSM/TeleOp/Inputs/Y-Input", yInput);
		Logger.recordOutput("DriveFSM/TeleOp/Inputs/A-Input", rotInput);

		// Calculate speeds
		// Clamping the speeds here might be redundant as the kinematics should already
		// do this.
		var xSpeed = -MathUtil.applyDeadband(
				xInput, Constants.OIConstants.DRIVE_DEADBAND)
				* DriveConstants.MAX_SPEED_METERS_PER_SECOND / DriveConstants.SPEED_DAMP_FACTOR;

		var ySpeed = -MathUtil.applyDeadband(
				yInput, Constants.OIConstants.DRIVE_DEADBAND)
				* DriveConstants.MAX_SPEED_METERS_PER_SECOND / DriveConstants.SPEED_DAMP_FACTOR;

		// Rotational Speed
		var aSpeed = -MathUtil.applyDeadband(
				rotInput, Constants.OIConstants.DRIVE_DEADBAND)
				* DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC / DriveConstants.SPEED_DAMP_FACTOR;

		Logger.recordOutput("DriveFSM/TeleOp/Speeds/X-Speed", xSpeed);
		Logger.recordOutput("DriveFSM/TeleOp/Speeds/Y-Speed", ySpeed);
		Logger.recordOutput("DriveFSM/TeleOp/Speeds/A-Speed", aSpeed);

		// Calculate and send speeds
		drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				xSpeed, ySpeed, aSpeed, Rotation2d.fromDegrees(getHeading())));

		if (input.getDriveControllerZeroHeadingPressed()) {
			zeroHeading();
		}
	}

	private void handleAlignToTagState() {
		var tag = rpi.getAprilTagWithID(9);

		if (tag != null && !tagPositionAligned) {
			double rpiX = tag.getZ();
			double rpiY = tag.getX();
			double rpiTheta = tag.getPitch();

			double xSpeed = Math.abs(rpiX) < Constants.VisionConstants.X_MARGIN_TO_REEF
					? MAXSwerveModule.clamp(
							rpiX / Constants.VisionConstants.TRANSLATIONAL_ACCEL_CONSTANT,
							-Constants.VisionConstants.MAX_SPEED_METERS_PER_SECOND,
							Constants.VisionConstants.MAX_SPEED_METERS_PER_SECOND)
					: 0;

			double ySpeed = Math.abs(rpiY) < Constants.VisionConstants.Y_MARGIN_TO_REEF
					? MAXSwerveModule.clamp(
							rpiY / Constants.VisionConstants.TRANSLATIONAL_ACCEL_CONSTANT,
							-Constants.VisionConstants.MAX_SPEED_METERS_PER_SECOND,
							Constants.VisionConstants.MAX_SPEED_METERS_PER_SECOND)
					: 0;

			double aSpeed = Math.abs(rpiTheta) < Constants.VisionConstants.ROT_MARGIN_TO_REEF
					? MAXSwerveModule.clamp(
							rpiTheta / Constants.VisionConstants.ROTATIONAL_ACCEL_CONSTANT,
							-Constants.VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
							Constants.VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND)
					: 0;

			drive(ChassisSpeeds.fromFieldRelativeSpeeds(
					xSpeed, ySpeed, aSpeed, Rotation2d.fromDegrees(getHeading())));
		}
	}

	public void followTrajectory(SwerveSample sample) {
		var pose = getPose();

		var targetSpeeds = new ChassisSpeeds(
				sample.vx + xController.calculate(pose.getX(), sample.x),
				sample.vy + yController.calculate(pose.getY(), sample.y),
				sample.omega + headingController.calculate(pose.getRotation().getRadians(),
						sample.heading));

		swerveDrive.driveFieldOriented(targetSpeeds);
		// I assume that these speeds are field relative, the choreo doc says to have a
					// driveFieldRelative
	}

	private void setModuleStates(SwerveModuleState[] states) {
		swerveDrive.setModuleStates(states, true);
	}

	private SwerveModuleState[] getModuleStates() {
		return swerveDrive.getStates();
	}

	/**
	 * Get the current gyro heading in degrees.
	 * 
	 * @return Current gyro heading in degrees
	 */
	private double getHeading() {
		return swerveDrive.getYaw().getDegrees() * (DriveConstants.IS_GYRO_REVERSED ? -1 : 1);
	}

	/**
	 * Set the swerve modules to an x formation to prevent any movement
	 */
	private void setX() {
		setModuleStates(
				new SwerveModuleState[] {
						new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
						new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
						new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
						new SwerveModuleState(0, Rotation2d.fromDegrees(45))
				});
	}

	/**
	 * Zero the gyro heading.
	 */
	private void zeroHeading() {
		swerveDrive.setGyroOffset(new Rotation3d());
	}

	private SwerveModulePosition[] getModulePositions() {
        return swerveDrive.getModulePositions();
	}

	/**
	 * Get the current pose of the robot.
	 * 
	 * @return Current pose of the robot
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}
}
