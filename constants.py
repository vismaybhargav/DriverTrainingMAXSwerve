import math
import rev
from wpimath import units
from wpimath.kinematics import SwerveDrive4Kinematics
import wpimath.trajectory

""" DRIVE CONSTANTS """
# Driving parameters - Note that these are not the maximum capable speeds of
# the robot, rather the allowed maximum speeds
kMaxSpeed = 4.8
kMaxAngularSpeed = 2 * math.pi

kDirectionSlewRate = 1.2 # radians per second
kMagnitudeSlewRate = 1.8 # percent per second (1 = 100%)
kRotationalSlewRate = 2.0 # percent per second (1 = 100%)

# Chassis configuration
kTrackWidth = units.inchesToMeters(28.0) # Distance between centers of right and left wheels on robot METERS
kWheelBase = units.inchesToMeters(28.0) # Distance between centers of front and back wheels on robot METERS

kKinematics = SwerveDrive4Kinematics

# Angular offsets of the modules relative to the chassis in radians
kFrontLeftChassisAngularOffset = -math.pi / 2
kFrontRightChassisAngularOffset = 0
kRearLeftChassisAngularOffset = math.pi
kRearRightChassisAngularOffset = math.pi / 2

# SPARK MAX CAN IDs
kFrontLeftDrivingCanId = 26
kRearLeftDrivingCanId = 24
kFrontRightDrivingCanId = 28
kRearRightDrivingCanId = 32

kFrontLeftTurningCanId = 25
kRearLeftTurningCanId = 23
kFrontRightTurningCanId = 27
kRearRightTurningCanId = 31

""" MODULE CONSTANTS """
# Invert the turning encoder, since the output shaft rotates in the opposite direction
# of the steering motor in the MAXSwerve Module
kTurningEncoderInverted = True

# The MaxSwerve module can be configured with one of the three pinion gears: 12T, 13T, or 14T.
# This changes the drive speed of the module ( a pinion gear with more teeth will result in a robot that drives faster)
kDrivingMotorPinionTeeth = 13

# Calculations required for driving motor conversion factors and feed forward
kDrivingMotorFreeSpeedRps = 5676.0 / 60
kWheelDiameter = 0.0762
kWheelCircumference = kWheelDiameter * math.pi

# 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction
kDrivingEncoderPositionFactor = (kWheelDiameter * math.pi) / kDrivingMotorReduction # Meters
kDrivingEncoderVelocityFactor = ((kWheelDiameter * math.pi) / kDrivingMotorReduction) / 60.0 # Meters per second

kTurningEncoderPositionFactor = (2 * math.pi) # radians
kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0 # meters per second

kTurningEncoderPositionPIDMinInput = 0
kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

kDrivingP = 0.04
kDrivingI = 0
kDrivingD = 0
kDrivingFF = (1/kDriveWheelFreeSpeedRps)
kDrivingMinOutput = -1
kDrivingMaxOutput = 1

kTurningP = 1
kTurningI = 0
kTurningD = 0
kTurningFF = 0
kTurningMinOutput = -1
kTurningMaxOutput = 1

kDrivingMotorIdleMode: rev.SparkMax.IdleMode = rev.SparkMax.IdleMode.kCoast
kTurningMotorIdleMode: rev.SparkMax.IdleMode = rev.SparkMax.IdleMode.kCoast

kDrivingMotorCurrentLimit = 40 # Amps
kTurningMotorCurrentLimit = 30 # Amps

""" Auto Constants """
kMaxSpeed = 3 # meters per second
kMaxAcceleration = 2 # meters per second squared
kMaxAngularSpeed = 3.142 # radians per second
kMaxAngularAcceleration = 3.142 # radians per second squared

kPXController = 0.5
kPYController = 0.5
kPThetaController = 0.5

kThetaControllerConstraints = wpimath.trajectory.TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAcceleration)

""" OI Constants """
kDriverControllerPort = 0
kDriveDeadband = 0.08

