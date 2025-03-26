// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports

// Systems
import choreo.auto.AutoFactory;
import frc.robot.input.TeleopInput;
import frc.robot.systems.DriveFSMSystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveSystem;
	private AutoFactory autoFactory;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		Logger.recordMetadata("2473", "MaxSwerve Bot");

		// ==== Set Up Logging ==== //
		if(isReal()) {
			Logger.addDataReceiver(new NT4Publisher());
			//Logger.addDataReceiver(new WPILOGWriter());
		} else if(isSimulation()) {
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			setUseTiming(false); // Run as fast as possible

			var logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
		}
		// ========================= //

		Logger.start(); // Begin logging

		// Instantiate all systems here
		driveSystem = new DriveFSMSystem();

		//TODO: Are we using this or the new architecture?
		autoFactory = new AutoFactory(
				driveSystem::getPose,
				driveSystem::resetOdometry,
				driveSystem::followTrajectory,
				true,
				driveSystem
		);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveSystem.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() {
		Logger.recordOutput("DriveFSM/Current State", driveSystem.getCurrentState());
    Logger.recordOutput("DriveFSM/Swerve States", driveSystem.getModuleStates());
    Logger.recordOutput("DriveFSM/Chassis Speeds", driveSystem.getChassisSpeeds());
    Logger.recordOutput("DriveFSM/Current Pose", driveSystem.getPose());
	}
}
