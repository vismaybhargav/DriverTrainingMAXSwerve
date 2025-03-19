// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
		public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
		public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * Math.PI; // radians per second

		// Chassis configuration
		public static final double kTrackWidth = Units.inchesToMeters(22.5);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(22.75);
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;

		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 24;
		public static final int kRearLeftDrivingCanId = 32;
		public static final int kFrontRightDrivingCanId = 26;
		public static final int kRearRightDrivingCanId = 28;

		public static final int kFrontLeftTurningCanId = 23;
		public static final int kRearLeftTurningCanId = 31;
		public static final int kFrontRightTurningCanId = 25;
		public static final int kRearRightTurningCanId = 27;

		//====== PIDS =====//

		// Driving
		public static final double kDrivingP = 0.04;
		public static final double kDrivingI = 0;
		public static final double kDrivingD = 0;

		//Turning
		public static final double kTurningP = 1;
		public static final double kTurningI = 0;
		public static final double kTurningD = 0;

		public static final int kDrivingCurrentLimitAmps = 40;
		public static final int kTurningCurrentLimitAmps = 30;

		public static final double SPEED_DAMP_FACTOR = 2;

		public static final boolean IS_GYRO_REVERSED = false;
	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
		// more teeth will result in a robot that drives faster).
		public static final int DRIVING_MOTOR_PINION_TEETH = 13;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
		public static final double WHEEL_DIAMETER_METERS = 0.0762;
		public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
		public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
				/ DRIVING_MOTOR_REDUCTION;
	}

	public static final class VisionConstants {
		public static final String REEF_CAM_NAME = "Reef_Camera";
		public static final String SOURCE_CAM_NAME = "Source_Camera";
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final double DRIVE_DEADBAND = 0.05;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPThetaController = 1;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double FREE_SPEED_RPM = 5676;
	}
}
