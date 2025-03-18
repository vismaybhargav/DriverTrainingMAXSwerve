// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;

import org.littletonrobotics.junction.Logger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final 

  // The driver's controller
  CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    m_swerve.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1
  )
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(Constants.OIConstants.kDriveDeadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(
      m_driverController::getRightX,
      m_driverController::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_swerve.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(
          2))
      .deadband(Constants.OIConstants.kDriveDeadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
        () -> Math.sin(
          m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
        () -> Math.cos(
          m_driverController.getRawAxis(2) * Math.PI) * (Math.PI *2)
      )
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    Command driveFieldOrientedDirectAngle      = m_swerve.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = m_swerve.driveFieldOriented(driveAngularVelocity);

    Command driveRobotOrientedAngularVelocity  = m_swerve.driveFieldOriented(driveRobotOriented);

    Command driveFieldOrientedDirectAngleKeyboard = m_swerve.driveFieldOriented(driveDirectAngleKeyboard);

    Command driveFieldOrientedAnglularVelocityKeyboard = m_swerve.driveFieldOriented(driveAngularVelocityKeyboard);

    m_swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  public void updateOutputs() {
    Logger.recordOutput("Pose", m_swerve.getSwerveDrive().getMapleSimDrive().get().getSimulatedDriveTrainPose());
  }
}
