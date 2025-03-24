import rev
from rev import SparkMaxConfig
import constants

# Driving motor configuration
driving_config: SparkMaxConfig = SparkMaxConfig()

# Set the idle mode to brake and the current limit to the driving motor
(driving_config
 .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
 .smartCurrentLimit(constants.kDrivingMotorCurrentLimit))

# Set the encoder conversion factors
(driving_config.encoder
 .positionConversionFactor(constants.kDrivingEncoderPositionFactor)
 .velocityConversionFactor(constants.kDrivingEncoderVelocityFactor))

# Set the PIDF constants for the driving motor
(driving_config.closedLoop
 .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
 .pid(constants.kDrivingP, constants.kDrivingI, constants.kDrivingD)
 .velocityFF(constants.kDrivingFF)
 .outputRange(-1, 1)
 )


# Turning motor configuration
turning_config: SparkMaxConfig = SparkMaxConfig()

# Set the idle mode to brake and the current limit to the turning motor
(turning_config
 .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
 .smartCurrentLimit(constants.kTurningMotorCurrentLimit))

# Set the encoder conversion factors
(turning_config.encoder
 .positionConversionFactor(constants.kTurningEncoderPositionFactor)
 .velocityConversionFactor(constants.kTurningEncoderVelocityFactor))

# Set the PIDF constants for the turning motor
(turning_config.closedLoop
 .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
 .pid(constants.kTurningP, constants.kTurningI, constants.kTurningD)
 .velocityFF(constants.kTurningFF)
 .outputRange(-1, 1)
 .positionWrappingEnabled(True)
 .positionWrappingInputRange(0, constants.kTurningEncoderPositionFactor)
 )

