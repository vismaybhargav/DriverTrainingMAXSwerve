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
  public static final class RobotConstants {
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * Math.PI; // radians per second

    public static final boolean IS_FIELD_RELATIVE = true;
    public static final double SPEED_DAMP_FACTOR = 2;

    // Chassis configuration
    public static final double TRACK_WIDTH_IN = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE_IN = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_IN / 2, TRACK_WIDTH_IN / 2),
            new Translation2d(WHEEL_BASE_IN / 2, -TRACK_WIDTH_IN / 2),
            new Translation2d(-WHEEL_BASE_IN / 2, TRACK_WIDTH_IN / 2),
            new Translation2d(-WHEEL_BASE_IN / 2, -TRACK_WIDTH_IN / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 13;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 15;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 17;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 10;
    public static final int REAR_LEFT_TURNING_CAN_ID = 12;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 14;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 16;

    public static final boolean IS_GYRO_REVERSED = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_MOTOR_REDUCTION;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
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

  public static final class VisionConstants {
    public static final int UNABLE_TO_SEE_NOTE_CONSTANT = 5000;
    public static final int UNABLE_TO_SEE_TAG_CONSTANT = 4000;

    public static final double MAX_SPEED_METERS_PER_SECOND = 0.2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI / 20;

    public static final double TRANSLATIONAL_ACCEL_CONSTANT = 1.5;
    public static final double ROTATIONAL_ACCEL_CONSTANT = 1;
    //TODO: equate the margin to the probabilistic equation from HW.
    public static final double X_MARGIN_TO_REEF = 0.05;
    public static final double Y_MARGIN_TO_REEF = 0.05;
    public static final double ROT_MARGIN_TO_REEF = 0.09;
    public static final double REEF_TARGET_DISTANCE = 1.2;

    public static final double N_180 = 180;

    public static final int AT_ARR_INC = 10;
    public static final int AT_ARR_CAMERA_OFFSET = 1;
    public static final int AT_ARR_TRANSLATION_OFFSET = 4;
    public static final int AT_ARR_ROTATION_OFFSET = 7;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}