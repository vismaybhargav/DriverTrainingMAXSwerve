package frc.robot.input;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/*
		TODO: Create a HAL for controllers to allow for easy swapping of controllers
	 */

	/* ======================== Constants ======================== */
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private final PS4Controller driveController;
	private final PS4Controller mechController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	public double getDriveControllerLeftX() {
		return driveController.getLeftX();
	}

	public double getDriveControllerLeftY() {
		return driveController.getLeftY();
	}

	public double getDriveControllerRightX() {
		return driveController.getRightX();
	}

	public boolean getDriveControllerZeroHeadingPressed() {
		return driveController.getShareButton();
	}

	public boolean getDriveControllerAlignToTagPressed() {
		return driveController.getSquareButton();
	}
}
