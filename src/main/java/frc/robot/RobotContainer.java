// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive;

  // The simulation
  private SwerveDriveSimulation simulation = null;

  // The driver's controller
  CommandPS4Controller driverController = new CommandPS4Controller(OIConstants.driverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if(Robot.isReal()) {
      robotDrive = new DriveSubsystem(
              new GyroIONavX(),
              new ModuleIOSpark(0),
              new ModuleIOSpark(1),
              new ModuleIOSpark(2),
              new ModuleIOSpark(3),
              (pose) -> {});
    } else if(Robot.isSimulation()) {
      simulation = new SwerveDriveSimulation(
              Constants.SimConstants.mapleSimConfig,
              new Pose2d(3, 3, new Rotation2d())
      );
      SimulatedArena.getInstance().addDriveTrainSimulation(simulation);

      var modules = simulation.getModules();

      robotDrive = new DriveSubsystem(
              new GyroIOSim(simulation.getGyroSimulation()),
              new ModuleIOSim(modules[0]),
              new ModuleIOSim(modules[1]),
              new ModuleIOSim(modules[2]),
              new ModuleIOSim(modules[3]),
              simulation::setSimulationWorldPose
      );
    } else {
      robotDrive = new DriveSubsystem(
              new GyroIO()   {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              (pose) -> {});
    }

    // TODO: Setup auto routines

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
    System.out.println(robotDrive);
    // Configure default commands
    robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            DriveCommands.joystickDrive(
                    robotDrive,
                    () -> -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.driveDeadband),
                    () -> -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.driveDeadband),
                    () -> -MathUtil.applyDeadband(driverController.getRightX(),OIConstants.driveDeadband)
            )
    );

    final Runnable resetGyro = Robot.isSimulation()
            ? () -> robotDrive.resetOdometry(
                    simulation.getSimulatedDriveTrainPose())
            : () -> robotDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d())
    );
    driverController.share().onTrue(Commands.runOnce(resetGyro, robotDrive).ignoringDisable(true));
  }

  public void resetSimulationField() {
      if(!Robot.isSimulation() || simulation == null) return;

      robotDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if(!Robot.isSimulation() || simulation == null ) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("Field Simulation/Robot Pose", simulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("Field Simulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput("Field Simulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
