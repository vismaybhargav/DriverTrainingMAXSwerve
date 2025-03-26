// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

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

  public static final class SimConstants {
    public static final double kSimSteerP = 70;
    public static final double kSimSteerI = 0;
    public static final double kSimSteerD = 4.5;

    public static final double kSimDriveP = 0.1;
    public static final double kSimDriveI = 0;
    public static final double kSimDriveD = 0;

    public static final double DRIVE_FRICTION_VOLTS = 0.1;
    public static final double STEER_FRICTION_VOLTS = 0.15;

    public static final double STEER_INERTIA_KGMS2 = 0.05;

    public static final Pose2d BLUE_1_STARTING_POS_M = new Pose2d(
        7.5856494,
        6.4390466,
        new Rotation2d(Math.PI).minus(new Rotation2d(Math.PI / 3)));

    public static final Pose2d BLUE_2_STARTING_POS_M = new Pose2d(
        7.5856494,
        4.0468566,
        new Rotation2d(Math.PI));

    public static final Pose2d BLUE_3_STARTING_POS_M = new Pose2d(
        7.5856494,
        1.5596578,
        new Rotation2d(Math.PI).plus(new Rotation2d(Math.PI / 3)));

    public static final Pose2d RED_1_STARTING_POS_M = new Pose2d(
        9.972452163696289,
        1.5596578,
        new Rotation2d().plus(new Rotation2d(Math.PI / 3)));
    public static final Pose2d RED_2_STARTING_POS_M = new Pose2d(
        9.972452163696289,
        4.0468566,
        new Rotation2d());
    public static final Pose2d RED_3_STARTING_POS_M = new Pose2d(
        9.972452163696289,
        6.4390466,
        new Rotation2d().minus(new Rotation2d(Math.PI / 3)));

    // Estimated values for now, need to be calculated later
    public static final double MASS_WITH_BUMPER_LBS = 115;
    public static final double MOI = 6.99597;
    public static final double WHEEL_COF = 1.2;

    public static final Translation2d FL_TRANSLATION = new Translation2d(
        10.75,
        -10.75);

    public static final Translation2d FR_TRANSLATION = new Translation2d(
        10.75,
        10.75);

    public static final Translation2d BL_TRANSLATION = new Translation2d(
        -10.75,
        -10.75);

    public static final Translation2d BR_TRANSLATION = new Translation2d(
        -10.75,
        10.75);

    // mech pose logging constants
    public static final double ELEVATOR_WINCH_DIAMETER_METERS = 0.0463296;
    public static final double ELEVATOR_GEAR_RATIO = 10; // 25.0;

    // Camera names, must match names configured on coprocessor
    public static final String REEF_CAMERA_NAME = "Reef CV Camera";
    public static final String STATION_CAMERA_NAME = "Station CV Camera";

    public static final double STATION_CAMERA_RAD = Math.toRadians(19);

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static final Transform3d ROBOT_TO_REEF_CAMERA = new Transform3d(Units.inchesToMeters(7.129),
        -Units.inchesToMeters(4.306),
        Units.inchesToMeters(14.56), new Rotation3d(0.0, 0.0, 0.0));
    public static final Transform3d ROBOT_TO_STATION_CAMERA = new Transform3d(-Units.inchesToMeters(8.875),
        -Units.inchesToMeters(9.5),
        Units.inchesToMeters(37.596), new Rotation3d(0.0, -STATION_CAMERA_RAD, Math.PI));

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_MULTIPLIERS = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_MEGATAG_2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available

    // public static final double CAM_DISTANCE_READ = 2.5;
    public static final DriveTrainSimulationConfig kSimConfig = DriveTrainSimulationConfig.Default()
      .withCustomModuleTranslations(DriveConstants.kModuleTranslations)
      .withRobotMass(Kilograms.of(50))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
        DCMotor.getNEO(1),
        DCMotor.getNeo550(1),
        ModuleConstants.kDrivingMotorReduction,
        ModuleConstants.kTurningMotorReduction, 
        Volts.of(DRIVE_FRICTION_VOLTS),
        Volts.of(STEER_FRICTION_VOLTS), 
        Inches.of(1.5),
        KilogramSquareMeters.of(0.02), 
        WHEEL_COF
      )
    );
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.0);
    
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 26;
    public static final int kRearLeftDrivingCanId = 24;
    public static final int kFrontRightDrivingCanId = 28;
    public static final int kRearRightDrivingCanId = 32;

    public static final int kFrontLeftTurningCanId = 25;
    public static final int kRearLeftTurningCanId = 23;
    public static final int kFrontRightTurningCanId = 27;
    public static final int kRearRightTurningCanId = 31;

    // ====== PIDS =====//

    // Driving
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;

    // Turning
    public static final double kTurningP = 2;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    public static final int kDrivingCurrentLimitAmps = 40;
    public static final int kTurningCurrentLimitAmps = 30;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kTurningMotorReduction = 9424.0 / 203.0;
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

    /* -- ALL GENERAL CONSTANTS -- */
    public static final double DEG_180 = 180;
    public static final double DEG_360 = 360;

    /* -- ALL SOURCE SPECIFIC CONSTANTS -- */
    public static final double SOURCE_X_OFFSET = -Units.inchesToMeters(35.5 / 2 - 6 - 6)
        + SimConstants.ROBOT_TO_STATION_CAMERA.getX();
    public static final double SOURCE_Y_OFFSET = 0;

    public static final int BLUE_L_STATION_ID = 13;
    public static final int BLUE_R_STATION_ID = 12;
    public static final int RED_L_STATION_ID = 1;
    public static final int RED_R_STATION_ID = 2;

    public static final double REEF_X_TAG_OFFSET = Units.inchesToMeters(35.5 / 2 - 6)
        + SimConstants.ROBOT_TO_REEF_CAMERA.getX();
    public static final double REEF_Y_L_TAG_OFFSET = -Units.inchesToMeters(13) / 2
        + SimConstants.ROBOT_TO_REEF_CAMERA.getY();
    public static final double REEF_Y_R_TAG_OFFSET = Units.inchesToMeters(13) / 2
        + SimConstants.ROBOT_TO_REEF_CAMERA.getY();
    public static final double STATION_Y_L_TAG_OFFSET = -Units.inchesToMeters(15)
        + SimConstants.ROBOT_TO_REEF_CAMERA.getY();
    public static final double STATION_Y_R_TAG_OFFSET = Units.inchesToMeters(15)
        + SimConstants.ROBOT_TO_REEF_CAMERA.getY();

    public static final int R_REEF_1_TAG_ID = 10;
    public static final int R_REEF_2_TAG_ID = 11;
    public static final int R_REEF_3_TAG_ID = 6;
    public static final int R_REEF_4_TAG_ID = 7;
    public static final int R_REEF_5_TAG_ID = 8;
    public static final int R_REEF_6_TAG_ID = 9;

    public static final int B_REEF_1_TAG_ID = 21;
    public static final int B_REEF_2_TAG_ID = 20;
    public static final int B_REEF_3_TAG_ID = 19;
    public static final int B_REEF_4_TAG_ID = 18;
    public static final int B_REEF_5_TAG_ID = 17;
    public static final int B_REEF_6_TAG_ID = 22;

    public static final double ALIGN_DRIVE_P = 2.5; // 2.2; //4.0;
    public static final double ALIGN_THETA_P = 1.3; // 1.3; //2.0;
    public static final double ALIGN_MAX_T_SPEED = 0.0;
    public static final double ALIGN_MAX_T_ACCEL = 0.0;
    public static final double ALIGN_MAX_R_SPEED = Math.PI * 0.0;
    public static final double ALIGN_MAX_R_ACCEL = Math.PI * 0.0;
    public static final double FF_MIN_RADIUS = 0.2;
    public static final double FF_MAX_RADIUS = 0.8;
    public static final double CHOREO_PID_TOLLERANCE = 1e-3;
    public static final double DRIVE_TOLERANCE = 0.01;
    public static final double THETA_TOLERANCE = Math.toRadians(1);

    public static final int DRIVE_CURRENT_LIMIT_FRAMES = 2;

    /* -- ALL COMMAND NAME CONSTANTS -- */

    public enum AutoCommands {
      /* Red Align Reef Tag Commands */
      R_ALIGN_REEF1_L_TAG_CMD,
      R_ALIGN_REEF1_R_TAG_CMD,
      R_ALIGN_REEF2_L_TAG_CMD,
      R_ALIGN_REEF2_R_TAG_CMD,
      R_ALIGN_REEF3_L_TAG_CMD,
      R_ALIGN_REEF3_R_TAG_CMD,
      R_ALIGN_REEF4_L_TAG_CMD,
      R_ALIGN_REEF4_R_TAG_CMD,
      R_ALIGN_REEF5_L_TAG_CMD,
      R_ALIGN_REEF5_R_TAG_CMD,
      R_ALIGN_REEF6_L_TAG_CMD,
      R_ALIGN_REEF6_R_TAG_CMD,
      /* Blue Align Reef Tag Commands */
      B_ALIGN_REEF1_L_TAG_CMD,
      B_ALIGN_REEF1_R_TAG_CMD,
      B_ALIGN_REEF2_L_TAG_CMD,
      B_ALIGN_REEF2_R_TAG_CMD,
      B_ALIGN_REEF3_L_TAG_CMD,
      B_ALIGN_REEF3_R_TAG_CMD,
      B_ALIGN_REEF4_L_TAG_CMD,
      B_ALIGN_REEF4_R_TAG_CMD,
      B_ALIGN_REEF5_L_TAG_CMD,
      B_ALIGN_REEF5_R_TAG_CMD,
      B_ALIGN_REEF6_L_TAG_CMD,
      B_ALIGN_REEF6_R_TAG_CMD,
      /* Align Station Tag Commands */
      R_ALIGN_STATION_L_TAG_CMD,
      R_ALIGN_STATION_R_TAG_CMD,
      B_ALIGN_STATION_L_TAG_CMD,
      B_ALIGN_STATION_R_TAG_CMD,
      /* Drive Peripheral Commands */
      DRIVE_BRAKE_CMD,
      /* Elevator Commands */
      ELEVATOR_GROUND_CMD,
      ELEVATOR_L2_CMD,
      ELEVATOR_L3_CMD,
      ELEVATOR_L4_CMD,
      ELEVATOR_WAIT,
      /* Funnel Commands */
      INTAKE_CORAL_CMD,
      OUTTAKE_CORAL_CMD
    }

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final int UNABLE_TO_SEE_NOTE_CONSTANT = 5000;
    public static final int UNABLE_TO_SEE_TAG_CONSTANT = 4000;

    public static final double MAX_SPEED_METERS_PER_SECOND = 0.15;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 0.1;

    // TODO: equate the margin to the probabilistic equation from HW.
    public static final double X_MARGIN_TO_REEF = 0.02;
    public static final double Y_MARGIN_TO_REEF = 0.02;
    public static final double ROT_MARGIN_TO_REEF = Math.toRadians(2); // TODO: change rot margin.
    public static final double TAG_TARGET_DISTANCE = 0.2;

    public static final double N_180 = 180;

    public static final String APRIL_TAG_FIELD_LAYOUT_JSON =
		Filesystem.getDeployDirectory() + "/apriltag/welded/apriltag.json";

    public static final int AT_ARR_INC = 10;
    public static final int AT_ARR_CAMERA_OFFSET = 1;
    public static final int AT_ARR_TRANSLATION_OFFSET = 4;
    public static final int AT_ARR_ROTATION_OFFSET = 7;

    public static final String REEF_CAM_NAME = "Reef_Camera";
    public static final String SOURCE_CAM_NAME = "Source_Camera";

    public static final int LOCALIZATION_TAG_NUM = 1;
    public static final double LOCALIZATION_TRANSLATIONAL_THRESHOLD = 0.4;
    public static final double LOCALIZATION_ANGLE_TOLERANCE = Math.toRadians(5);

    // TODO: tune constants based on field testing.
    public static final double MIN_TAG_TARGET_DISTANCE_X = 0.2;
    public static final double MAX_TAG_TARGET_DISTANCE_X = 2.0;
    public static final double TAG_Y_SCALING_COEF = 0.6;
    public static final double TAG_TARGET_THETA_RAD = Math.PI / 5;
    public static final double TAG_Y_L_OFFSET_BASE = AutoConstants.REEF_Y_L_TAG_OFFSET * 2;
    public static final double TAG_Y_R_OFFSET_BASE = AutoConstants.REEF_Y_R_TAG_OFFSET * 2;
  }
}