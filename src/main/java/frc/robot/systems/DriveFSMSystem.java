package frc.robot.systems;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.input.TeleopInput;
import frc.robot.vision.AprilTag;
import frc.robot.vision.rpi.RaspberryPi;
import frc.robot.vision.rpi.RaspberryPiPhoton;
import frc.robot.MAXSwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;
import jdk.jshell.spi.ExecutionControl;

import java.util.ArrayList;

import org.jcp.xml.dsig.internal.dom.Utils;
import org.littletonrobotics.junction.Logger;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_REEF_TAG_STATE,
		ALIGN_TO_STATION_TAG_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private final MAXSwerveModule frontLeft = new MAXSwerveModule(
		DriveConstants.kFrontLeftDrivingCanId, 
		DriveConstants.kFrontLeftTurningCanId, 
		DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule frontRight = new MAXSwerveModule(
		DriveConstants.kFrontRightDrivingCanId, 
		DriveConstants.kFrontRightTurningCanId, 
		DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule rearLeft = new MAXSwerveModule(
		DriveConstants.kRearLeftDrivingCanId, 
		DriveConstants.kRearLeftTurningCanId, 
		DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule rearRight = new MAXSwerveModule(
		DriveConstants.kRearRightDrivingCanId, 
		DriveConstants.kRearRightTurningCanId, 
		DriveConstants.kBackRightChassisAngularOffset);

	private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
	private final RaspberryPi rpi = new RaspberryPiPhoton(); 

	private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
			DriveConstants.kDriveKinematics,
			Rotation2d.fromDegrees(getHeading()),
			getModulePositions());

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
	public DriveFSMSystem() {
		// Perform hardware init
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
		if (input == null) {
			return;
		}

		switch (currentState) {
			case TELEOP_STATE:
				handleTeleOpState(input);
				break;
			case ALIGN_TO_REEF_TAG_STATE:
				handleReefTagAlignment(input);
				break;
			case ALIGN_TO_STATION_TAG_STATE:
				handleStationTagAlignment(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
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
	private void handleTeleOpState(TeleopInput input) {
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
		double xSpeed = -MathUtil.applyDeadband(
				xInput, Constants.OIConstants.kDriveDeadband)
				* DriveConstants.kMaxSpeedMetersPerSecond / 2;

		double ySpeed = -MathUtil.applyDeadband(
				yInput, Constants.OIConstants.kDriveDeadband)
				* DriveConstants.kMaxSpeedMetersPerSecond / 2;

		// Rotational Speed
		double aSpeed = -MathUtil.applyDeadband(
				rotInput, Constants.OIConstants.kDriveDeadband)
				* DriveConstants.kMaxSpeedMetersPerSecond / 2;

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

	/**
	 * Handles reef tag alignment by seeing the nearest reef tag.
	 * @param input
	 */
	public void handleReefTagAlignment(TeleopInput input) {

		if (input != null) {
			if (input.getAlignLeftOffsetButton()) {
				alignmentYOff = AutoConstants.REEF_Y_L_TAG_OFFSET;
			} else if (input.getAlignRightOffsetButton()) {
				alignmentYOff = AutoConstants.REEF_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = AutoConstants.REEF_Y_L_TAG_OFFSET;
			}
		}

		alignmentXOff = AutoConstants.REEF_X_TAG_OFFSET;

		ArrayList<AprilTag> sortedTagList = rpi.getReefAprilTags();

		if (DriverStation.getAlliance().get().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				if (tagID == -1) {
					for (int id: blueReefTagArray) {
						if (tag.getTagID() == id) {
							tagID = id;
							break;
						}
					}
				} else {
					break;
				}
			}

		} else if (DriverStation.getAlliance().get().equals(Alliance.Red) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				if (tagID == -1) {
					for (int id: redReefTagArray) {
						if (tag.getTagID() == id) {
							tagID = id;
							break;
						}
					}
				} else {
					break;
				}
			}

		}

		Logger.recordOutput("TagID", tagID);

		if (tagID != -1) {
			aligningToReef = true;
			handleTagAlignment(input, tagID, true);
		} else {
			drivetrain.setControl(brake);
		}
	}

	/**
	 * Handles station tag alignment by aligning with the nearest station tag.
	 * @param input
	 */
	public void handleStationTagAlignment(TeleopInput input) {

		if (input != null) {
			if (input.getAlignLeftOffsetButton()) {
				alignmentYOff = AutoConstants.STATION_Y_L_TAG_OFFSET;
			} else if (input.getAlignRightOffsetButton()) {
				alignmentYOff = AutoConstants.STATION_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = 0;
			}
		}

		alignmentXOff = -AutoConstants.SOURCE_X_OFFSET;

		ArrayList<AprilTag> sortedTagList = rpi.getStationAprilTags();

		if (DriverStation.getAlliance().get().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: blueStationTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		} else if (DriverStation.getAlliance().get().equals(Alliance.Red) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: redStationTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		}

		Logger.recordOutput("TagID", tagID);

		if (tagID != -1) {
			aligningToReef = false;
			handleTagAlignment(input, tagID, true);
		} else {
			drivetrain.setControl(brake);
		}
	}

	/**
	 * Handle tag alignment state.
	 * @param input
	 * @param id
	 * @param allianceFlip whether or not to invert the controls.
	 * allianceFlip should be true in TeleOp.
	 */
	private void handleTagAlignment(TeleopInput input, int id, boolean allianceFlip) {
		AprilTag tag = rpi.getAprilTagWithID(id);
		Pose2d currPose;

		if (Utils.isSimulation()) {
			currPose = getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose();
		} else {
			currPose = drivetrain.getState().Pose;
		}

		Transform3d robotToCamera;
		if (aligningToReef) {
		//TODO: make a reef and station alignment hepler function instead of just one.
			// robotToCamera =
			// new Transform2d(
			// 	SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().getX(),
			// 		// - if u use pose rotation.
			// 	SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().getY(),
			// 		// - if u use pose rotation.
			// 	SimConstants.ROBOT_TO_REEF_CAMERA.getRotation().toRotation2d()
			// );
			robotToCamera = SimConstants.ROBOT_TO_REEF_CAMERA;
		} else {
			robotToCamera =
			new Transform3d(
				SimConstants.ROBOT_TO_STATION_CAMERA.getTranslation(),
				new Rotation3d(0, SimConstants.STATION_CAMERA_RAD, 0)
				//.rotateBy(new Rotation3d(Rotation2d.k180deg))
			);
			//robotToCamera = SimConstants.ROBOT_TO_STATION_CAMERA;
		}

		System.out.println("TAG Reached here");

		if (tag != null) {
			alignmentPose2d =
				new Pose3d(
					currPose
				)
				.transformBy(robotToCamera)
				.plus(new Transform3d(
					tag.getZ(),
					(tag.getX()),
					-tag.getY(),
					new Rotation3d(
						new Rotation2d(-tag.getPitch())
					)
				))
				.toPose2d()
				.transformBy(
					new Transform2d(
						-alignmentXOff,
						-alignmentYOff,
						new Rotation2d()
					)
				);
		}

		if (alignmentPose2d != null) {
			if (tag != null) {
				Logger.recordOutput(
					"TransposedTag",
					new Pose3d(
						currPose
					)
					.transformBy(robotToCamera)
					.plus(new Transform3d(
						tag.getZ(),
						(tag.getX()),
						-tag.getY(),
						new Rotation3d(
							new Rotation2d(-tag.getPitch())
						)
					))
				);
			}
			driveToPose(alignmentPose2d, allianceFlip);
		}

		if (driveToPoseFinished || alignmentPose2d == null) {
			drivetrain.setControl(
				drive.withVelocityX(0)
				.withVelocityY(0)
				.withRotationalRate(0)
			);
			return;
		}

	}

	public void followTrajectory(SwerveSample sample) {
		var pose = getPose();

		var targetSpeeds = new ChassisSpeeds(
				sample.vx + xController.calculate(pose.getX(), sample.x),
				sample.vy + yController.calculate(pose.getY(), sample.y),
				sample.omega + headingController.calculate(pose.getRotation().getRadians(),
						sample.heading));

		drive(targetSpeeds); // I assume that these speeds are field relative, the choreo doc says to have a
					// driveFieldRelative
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
		var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
		setModuleStates(states);
	}

	private void setModuleStates(SwerveModuleState[] states) {
		frontLeft.setDesiredState(states[0]);
		frontRight.setDesiredState(states[1]);
		rearLeft.setDesiredState(states[2]);
		rearRight.setDesiredState(states[3]);
	}

	private SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				frontLeft.getState(),
				frontRight.getState(),
				rearLeft.getState(),
				rearRight.getState()
		};
	}

	/**
	 * Get the current gyro heading in degrees.
	 * 
	 * @return Current gyro heading in degrees
	 */
	private double getHeading() {
		return gyro.getAngle() * (DriveConstants.kGyroReversed ? -1 : 1);
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
		gyro.zeroYaw();
	}

	private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};
	}

	/**
	 * Get the current pose of the robot.
	 * 
	 * @return Current pose of the robot
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
				Rotation2d.fromDegrees(getHeading()),
				getModulePositions(),
				pose);
	}
}
