package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;

public class MAXSwerveModuleSim extends MAXSwerveModule{
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController drivingMotor;
    private final SimulatedMotorController.GenericMotorController turningMotor;

    private final PIDController driveController = new PIDController(
        SimConstants.kSimDriveP,
        SimConstants.kSimDriveI, 
        SimConstants.kSimDriveD
    );

    private final PIDController turnController = new PIDController(
        SimConstants.kSimSteerP, 
        SimConstants.kSimSteerI, 
        SimConstants.kSimSteerD
    );

    private double chassisAngularOffset;

    public MAXSwerveModuleSim(
        int drivingCANId,
        int turningCANId,
        double chassisAngularOffset,
        SwerveModuleSimulation moduleSimulation
    ) {
        super(drivingCANId, turningCANId, chassisAngularOffset);
        this.chassisAngularOffset = chassisAngularOffset;

        this.moduleSimulation = moduleSimulation;
        drivingMotor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(DriveConstants.kDrivingCurrentLimitAmps));
        turningMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(DriveConstants.kTurningCurrentLimitAmps));
    }

    @Override
    public SwerveModuleState getState() {
        return moduleSimulation.getCurrentState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            moduleSimulation.getDriveEncoderUnGearedPosition().in(Rotations),
            new Rotation2d(
                moduleSimulation.getSteerRelativeEncoderPosition().in(Rotations) - chassisAngularOffset
            )
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(
            moduleSimulation.getSteerRelativeEncoderPosition().in(Rotations)
        ));
    }
}