# TODO: insert robot code here

import wpilib
import wpilib.drive

import constants


class MyRobot(wpilib.TimedRobot):
    def __init__(self, period: 0.02):
        super().__init__(period)
        self.drive_controller = None

    def robotInit(self) -> None:
        """
        This function is called on program startup.
        """

        self.drive_controller = wpilib.PS4Controller(constants.kDriverControllerPort)
        self.swerve =

