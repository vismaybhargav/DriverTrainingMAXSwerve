// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.Module;

import java.lang.module.ModuleFinder;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.DriveConstants.driveKinematics;
import static frc.robot.Constants.DriveConstants.maxSpeedMetersPerSecond;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Gyro
  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Create MAXSwerveModules
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  static final Lock odometryLock = new ReentrantLock();

  private Consumer<Pose2d> resetSimulationPoseCallback;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.driveKinematics,
      Rotation2d.fromDegrees(getGyroAngle()),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(
      GyroIO gyro,
      ModuleIO flModule,
      ModuleIO frModule,
      ModuleIO blModule,
      ModuleIO brModule,
      Consumer<Pose2d> resetSimulationPoseCallback) {
    this.gyro = gyro;
    modules[0] = new Module(flModule, 0);
    modules[1] = new Module(frModule, 1);
    modules[2] = new Module(blModule, 2);
    modules[3] = new Module(brModule, 3);
    this.resetSimulationPoseCallback = resetSimulationPoseCallback;

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SparkOdometryThread.getInstance().start();

    updateSwerveModuleStates();
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;

    for (int i = 0; i < sampleCount; i++) {
      var modulePositions = new SwerveModulePosition[4];
      var moduleDeltas = new SwerveModulePosition[4];

      for (int j = 0; j < 4; j++) {
        modulePositions[j] = modules[j].getOdometryPositions()[i];
        moduleDeltas[j] = new SwerveModulePosition(
            modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters,
            modulePositions[j].angle);
        lastModulePositions[j] = modulePositions[j];
      }

      if (gyroInputs.connected) {
        // Actual gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Angle delta from the kinematics and module delta
        Twist2d twist = driveKinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
    }

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
    updateSwerveModuleStates();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);

  }

  /**
   * Runs the drive at the desired velocity
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Test to see if this fixes the drift
    var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var setpointStates = driveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSecond);

    // Unoptimized states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // These are the optimized states because runSetpoint mutates them.
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    modules[0].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    modules[1].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[2].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[3].runSetpoint(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Get the gyro angle in degrees
   */
  public double getGyroAngle() {
    return -m_gyro.getAngle();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getGyroAngle()).getDegrees();
  }

  /**
   * Resets the odometry pose
   * 
   * @param pose the pose to reset to
   */
  public void setPose(Pose2d pose) {
    m_odometry.resetPose(pose);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the module states (turn angles and drive vels) for all of the modules
   * @return module states as an array of SwerveModuleStates
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getSwerveStates() {
    return swerveModuleStates;
  }

  public void updateSwerveModuleStates() {
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = modules[i].getState();
    }
  }

  @AutoLogOutput(key = "SwerveStates/States")
  private ChassisSpeeds getChassisSpeeds() {
    return driveKinematics.toChassisSpeeds(getSwerveStates());
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSecond;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSecond / 10;
  }
}
