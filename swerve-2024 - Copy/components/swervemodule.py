#gibutors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import rev
import math
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import phoenix6.hardware
from wpilib import PWMTalonSRX
from wpimath.units import inchesToMeters, rotationsToRadians



kWheelRadius = inchesToMeters(2)
kWheelCircumference = kWheelRadius * math.tau
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau
kDriveGearRatio = 6.75 # 6.75:1
kTurningGearRatio = 12.8 # 12.8:1

kDriveConversionFactor = kWheelCircumference / kDriveGearRatio
kTurningConversionFactor = math.tau / kTurningGearRatio


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        canCoderChannel: int,

    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      CAN output for the drive motor.
        :param turningMotorChannel:    CAN output for the turning motor.
        """
        self.driveMotor = rev.CANSparkMax(driveMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorChannel, rev.CANSparkMax.MotorType.kBrushless)


        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        self.canCoder = phoenix6.hardware.CANcoder(canCoderChannel)

        # Configure drive PID Controller
        self.drivePIDController = wpimath.controller.PIDController(0.01, 0, 0)

        # Configure turning PID Controller+
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0.01,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Convert rotations to meters for the drive motor.
        self.driveEncoder.setPositionConversionFactor(kDriveConversionFactor)
        self.driveEncoder.setVelocityConversionFactor(kDriveConversionFactor / 60)

        # Convert rotations to radians for the turning motor.
        self.turningEncoder.setPositionConversionFactor(kTurningConversionFactor)
        self.turningEncoder.setVelocityConversionFactor(kTurningConversionFactor / 60)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getRelativeAngle(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(self.turningEncoder.getPosition())

    def getAbsoluteAngle(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(rotationsToRadians(self.canCoder.get_absolute_position().value_as_double))
    
    def resetToAbsolute(self) -> None:
        newPosition = self.getRelativeAngle() - self.getAbsoluteAngle()

        self.turningEncoder.setPosition(newPosition.radians())

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            self.getRelativeAngle(),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getVelocity(),
            self.getRelativeAngle(),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = self.getRelativeAngle()

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.getRelativeAngle().radians(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)