package frc.robot.systems;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.input.TeleopInput;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.vision.AprilTag;
import frc.robot.vision.RaspberryPi;

import jdk.jshell.spi.ExecutionControl;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.util.ArrayList;

import static edu.wpi.first.units.Units.Meter;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum DriveFSMState {
		TELEOP_STATE,
		ALIGN_TO_REEF_TAG_STATE,
		ALIGN_TO_STATION_TAG_STATE
	}

	/* ======================== Private variables ======================== */
	private DriveFSMState currentState;

	private final SwerveDrive swerveDrive;
	private Rotation2d rotationAlignmentPose;
	private	Pose2d alignmentPose2d = null;
	private boolean driveToPoseRunning = false;
	private boolean driveToPoseFinished = false;
	private boolean aligningToReef = false;

	private int[] blueReefTagArray = new int[] {
		AutoConstants.B_REEF_1_TAG_ID,
		AutoConstants.B_REEF_2_TAG_ID,
		AutoConstants.B_REEF_3_TAG_ID,
		AutoConstants.B_REEF_4_TAG_ID,
		AutoConstants.B_REEF_5_TAG_ID,
		AutoConstants.B_REEF_6_TAG_ID
	};
	private int[] redReefTagArray = new int[] {
		AutoConstants.R_REEF_1_TAG_ID,
		AutoConstants.R_REEF_2_TAG_ID,
		AutoConstants.R_REEF_3_TAG_ID,
		AutoConstants.R_REEF_4_TAG_ID,
		AutoConstants.R_REEF_5_TAG_ID,
		AutoConstants.R_REEF_6_TAG_ID
	};

	private int[] blueStationTagArray = new int[] {
		AutoConstants.BLUE_L_STATION_ID,
		AutoConstants.BLUE_R_STATION_ID
	};
	private int[] redStationTagArray = new int[] {
		AutoConstants.RED_L_STATION_ID,
		AutoConstants.RED_R_STATION_ID
	};

	/* -- cv constants -- */
	private RaspberryPi rpi = new RaspberryPi();
	private int tagID = -1;
	private double alignmentYOff;
	private double alignmentXOff;

	private ArrayList<Pose2d> aprilTagReefRefPoses = new ArrayList<Pose2d>();
	private ArrayList<Pose2d> aprilTagStationRefPoses = new ArrayList<Pose2d>();
	private ArrayList<Pose2d> aprilTagVisionPoses = new ArrayList<Pose2d>();
	private AprilTagFieldLayout aprilTagFieldLayout;
	private boolean hasLocalized = false;

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
	public DriveFSMState getCurrentState() {
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
		currentState = DriveFSMState.TELEOP_STATE;
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
	private DriveFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				if (input.getAlignReefButton()) {
					return DriveFSMState.ALIGN_TO_REEF_TAG_STATE;
				} else if (input.getDriveTriangleButton()) {
					return DriveFSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return DriveFSMState.TELEOP_STATE;
				}
			case ALIGN_TO_REEF_TAG_STATE:
				if (input.getAlignReefButton()) {
					return DriveFSMState.ALIGN_TO_REEF_TAG_STATE;
				} else if (input.getDriveTriangleButton()) {
					return DriveFSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return DriveFSMState.TELEOP_STATE;
				}
			case ALIGN_TO_STATION_TAG_STATE:
				if (input.getAlignReefButton()) {
					return DriveFSMState.ALIGN_TO_REEF_TAG_STATE;
				} else if (input.getDriveTriangleButton()) {
					return DriveFSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return DriveFSMState.TELEOP_STATE;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	private void handleTeleOpState(TeleopInput input) {
		/* --- cv alignment reset --- */
		tagID = -1;
		alignmentXOff = 0;
		alignmentYOff = 0;
		driveToPoseFinished = false;
		driveToPoseRunning = false;

		double constantDamp = 1;

		if (elevatorSystem != null) {
			constantDamp = (elevatorSystem.isElevatorAtL4() || input.getDriveCrossButton())
				? DriveConstants.SPEED_DAMP_FACTOR : DriveConstants.NORMAL_DAMP;
		}

		double xSpeed = MathUtil.applyDeadband(
			slewRateX.calculate(input.getDriveLeftJoystickY()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_SPEED / constantDamp;
			// Drive forward with negative Y (forward) ^

		double ySpeed = MathUtil.applyDeadband(
			slewRateY.calculate(input.getDriveLeftJoystickX()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_SPEED / constantDamp;
			// Drive left with negative X (left) ^

		double rotXComp = MathUtil.applyDeadband(
			input.getDriveRightJoystickX(), DriveConstants.JOYSTICK_DEADBAND)
			* MAX_ANGULAR_RATE / constantDamp;
			// Drive left with negative X (left) ^

		if (rotXComp != 0) {
			rotationAlignmentPose =
				(Utils.isSimulation())
					? getMapleSimDrivetrain().getDriveSimulation()
					.getSimulatedDriveTrainPose().getRotation()
					: drivetrain.getState().Pose.getRotation();
		}

		if (!input.getDriveCircleButton()) {
			drivetrain.setControl(
				driveFacingAngle.withVelocityX(xSpeed * allianceOriented.getAsInt())
				.withVelocityY(ySpeed * allianceOriented.getAsInt())
				.withTargetDirection(rotationAlignmentPose)
				.withTargetRateFeedforward(-rotXComp)
				.withHeadingPID(DriveConstants.HEADING_P, 0, 0)
			);
		} else {
			drivetrain.setControl(
				driveRobotCentric.withVelocityX(xSpeed * allianceOriented.getAsInt())
				.withVelocityY(ySpeed * allianceOriented.getAsInt())
				.withRotationalRate(-rotXComp)
			);
		}

		if (input.getSeedGyroButtonPressed()) {
			drivetrain.seedFieldCentric();
			rotationAlignmentPose = new Rotation2d();
			hasLocalized = false;
		}

		Logger.recordOutput("TeleOp/XSpeed", xSpeed);
		Logger.recordOutput("TeleOp/YSpeed", ySpeed);
		Logger.recordOutput("TeleOp/RotSpeed", rotXComp);	
	}

	private void handleReefTagAlignment(TeleopInput input) {

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

	private void handleStationTagAlignment(TeleopInput input) {
		
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
		if(Robot.isSimulation()) {
			return swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose();
		} else {
			return swerveDrive.getPose();
		}
	}

	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}
}
