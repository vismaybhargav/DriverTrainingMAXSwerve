import rev
from rev import SparkMax, SparkRelativeEncoder, SparkAbsoluteEncoder, SparkBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

import configs

class MAXSwerveModule:
    def __init__(
            self,
            driving_can_id: int,
            turning_can_id: int,
            chassis_angular_offset: float
    ) -> None:
        self.driving_spark_max: SparkMax = (
            SparkMax(driving_can_id, rev.SparkMax.MotorType.kBrushless))
        self.turning_spark_max: SparkMax = (
            SparkMax(turning_can_id, SparkMax.MotorType.kBrushless))

        # Get the encoders for the motors
        self.driving_encoder: SparkRelativeEncoder = self.driving_spark_max.getEncoder()
        self.turning_encoder: SparkAbsoluteEncoder = self.turning_spark_max.getAbsoluteEncoder()

        # Get the closed loop controllers for the motors, this is different from the WPILIB API
        self.driving_closed_loop = self.driving_spark_max.getClosedLoopController()
        self.turning_closed_loop = self.turning_spark_max.getClosedLoopController()

        # Configure the driving motor
        self.driving_spark_max.configure(
            configs.driving_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        # Configure the turning motor
        self.turning_spark_max.configure(
            configs.turning_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        self.chassis_angular_offset: float = chassis_angular_offset
        self.desired_state: SwerveModuleState = SwerveModuleState(0, 0, Rotation2d())
        self.desired_state.angle = Rotation2d(self.turning_encoder.getPosition())
        self.driving_encoder.setPosition(0)

    def get_state(self) -> SwerveModuleState:
        """
        Get the current state of the swerve module
        """

        return SwerveModuleState(
            self.driving_encoder.getVelocity(),
            # Apply the chassis angular offset relative to the chassis
            Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset),
        )

    def get_position(self) -> SwerveModulePosition:
        """
        Get the current position of the swerve module
        """
        return SwerveModulePosition(
            self.driving_encoder.getPosition(),
            # Apply the chassis angular offset relative to the chassis
            Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset),
        )

    def set_desired_state(self, state: SwerveModuleState) -> None:
        """
        Set the desired state of the swerve module
        """
        # Apply the chassis angular offset to the desired state
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = state.speed
        corrected_desired_state.angle = self.desired_state.angle + Rotation2d(self.chassis_angular_offset)

        # Optimize the reference state to avoid spinning the module further than 90 degrees
        corrected_desired_state.optimize(Rotation2d(self.turning_encoder.getPosition()))

        # Command the motors to the desired state
        self.driving_closed_loop.setReference(corrected_desired_state.speed, SparkMax.ControlType.kVelocity)
        self.turning_closed_loop.setReference(corrected_desired_state.angle.radians(), SparkMax.ControlType.kPosition)

        self.desired_state = state

    def reset_encoders(self) -> None:
        """
        Zero's all the Swerve Module encoders
        """
        self.driving_encoder.setPosition(0)
