package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Features;
import frc.robot.Robot;
import frc.robot.input.TeleopInput;
import frc.robot.systems.drive.SparkOdometryThread;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.systems.drive.module.ModuleIO;
import frc.robot.systems.drive.module.Module;
import frc.robot.Constants.DriveConstants;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveConstants.DRIVE_KINEMATICS;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions 
	public enum FSMState {
		TELEOP_STATE,
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private final Module[] modules = new Module[4]; // FL, FR, BL, BR

	public static final Lock odometryLock = new ReentrantLock();

	private final GyroIO gyro;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	
	private Rotation2d rawGyroRotation = new Rotation2d();
	private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

	private final SwerveDriveOdometry odometry;

	private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
		DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d());
	
	private final Consumer<Pose2d> resetSimPoseCallback;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem(
		GyroIO gyro,
		ModuleIO frontLeftIO,
		ModuleIO frontRightIO,
		ModuleIO rearLeftIO,
		ModuleIO rearRightIO,
		Consumer<Pose2d> resetSimPoseCallback
	) {
		// Perform hardware init
		modules[0] = new Module(frontLeftIO, 0);
		modules[1] = new Module(frontRightIO, 1);
		modules[2] = new Module(rearLeftIO, 2);
		modules[3] = new Module(rearRightIO, 3);

		this.resetSimPoseCallback = resetSimPoseCallback;

		this.gyro = gyro;

		odometry = new SwerveDriveOdometry(
			DriveConstants.DRIVE_KINEMATICS, 
			getHeading(), 
			getModulePositions()
		);

		SparkOdometryThread.getInstance().start();
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
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
				handleTeleopState(input);
			}
			default -> throw new IllegalStateException("Invalid state: " + currentState);
		}

		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 *
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous() {
        return true;
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
			case TELEOP_STATE -> FSMState.TELEOP_STATE;
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
				xSpeed, ySpeed, aSpeed, getHeading()));

		if (input.getDriveControllerZeroHeadingPressed()) {
			zeroHeading();
		}
	}

	public void updateModules() {
		gyro.updateInputs(gyroInputs);

		Logger.processInputs("DriveFSM/Gyro", gyroInputs);

		for (Module module : modules) {
			module.periodic();
		}

		double[] sampleTimestamps = modules[0].getOdometryTimestamps();

		for (int i = 0; i < sampleTimestamps.length; i++) {
			var modPos = new SwerveModulePosition[4];
			var modDelta = new SwerveModulePosition[4];
			for (int j = 0; j < 4; j++) {
				modPos[j] = modules[j].getOdometryPositions()[i];
				modDelta[j] = new SwerveModulePosition(
						modPos[j].distanceMeters - lastModulePositions[j].distanceMeters,
						modPos[j].angle);
				lastModulePositions[j] = modPos[j];
			}

			if (gyroInputs.connected) {
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				Twist2d twist = DRIVE_KINEMATICS.toTwist2d(modDelta);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modPos);
		}

		odometry.update(getHeading(), getModulePositions());

	}

	/**
	 * Drive the robot using the given chassis speeds. Robot relative or field
	 * relative depends on how
	 * you pass in the speeds, see
	 * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}.
	 * 
	 * @param speeds Chassis speeds
	 */
	private void drive(ChassisSpeeds speeds) {
		var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		Logger.recordOutput("DriveFSM/Setpoint States", states);
		setModuleStates(states);
	}

	private void setModuleStates(SwerveModuleState[] states) {
		for (int i = 0; i < modules.length; i++) {
			modules[i].runSetpoint(states[i]);
		}
	}

	/**
	 * Get the current DriveFSM state
	 *
	 * @return current FSM state.
	 */
	@AutoLogOutput(key = "DriveFSM/Current State")
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Get the current pose of the robot.
	 *
	 * @return Current pose of the robot
	 */
	@AutoLogOutput(key = "DriveFSM/Current Pose")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Get the current states of the swerve modules.
	 *
	 * @return Current states of the swerve modules
	 */
	@AutoLogOutput(key = "DriveFSM/Swerve States")
	public SwerveModuleState[] getModuleStates() {
		// Convert this into using streams
		return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
	}

	/**
	 * Get the current chassis speeds.
	 *
	 * @return Current chassis speeds
	 */
	@AutoLogOutput(key = "DriveFSM/Chassis Speeds")
	public ChassisSpeeds getChassisSpeeds() {
		return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Get the current gyro heading as a Rotation2D.
	 * 
	 * @return Current gyro heading as a Rotation2D
	 */
	@AutoLogOutput(key = "DriveFSM/Gyro Heading")
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * Get the module positions
	 *
	 * @return Array of module positions
	 */
	@AutoLogOutput(key = "DriveFSM/Module Positions")
	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
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
		//gyro.zeroYaw();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
				getHeading(),
				getModulePositions(),
				pose);
	}
}
