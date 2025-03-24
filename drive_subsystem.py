import navx
import wpilib

import constants
from swerve_module import MAXSwerveModule

class DriveSubsystem():
    def __init__(self):
        self.front_left = MAXSwerveModule(
            constants.kFrontLeftDrivingCanId,
            constants.kFrontLeftTurningCanId,
            constants.kFrontLeftChassisAngularOffset
        )

        self.rear_left = MAXSwerveModule(
            constants.kRearLeftDrivingCanId,
            constants.kRearLeftTurningCanId,
            constants.kRearLeftChassisAngularOffset
        )

        self.front_right = MAXSwerveModule(
            constants.kFrontRightDrivingCanId,
            constants.kFrontRightTurningCanId,
            constants.kFrontRightChassisAngularOffset
        )

        self.rear_right = MAXSwerveModule(
            constants.kRearRightDrivingCanId,
            constants.kRearRightTurningCanId,
            constants.kRearRightChassisAngularOffset
        )

        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

    def periodic(self) -> None:




