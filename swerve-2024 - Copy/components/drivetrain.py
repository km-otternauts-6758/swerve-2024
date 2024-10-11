#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import navx
import math
import wpimath.geometry
import wpimath.kinematics
from components import swervemodule
from wpimath.units import inchesToMeters
import navx

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

kChassisX = inchesToMeters(30) # Length
kChassisY = inchesToMeters(30) # Width


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(kChassisX / 2, kChassisY / 2)
        self.frontRightLocation = wpimath.geometry.Translation2d(kChassisX /2, -kChassisY / 2)
        self.backLeftLocation = wpimath.geometry.Translation2d(-kChassisX / 2, kChassisY / 2)
        self.backRightLocation = wpimath.geometry.Translation2d(-kChassisX / 2, -kChassisY / 2)

        self.frontLeft = swervemodule.SwerveModule(1, 2, 3)
        self.frontRight = swervemodule.SwerveModule(3, 4, 1)
        self.backLeft = swervemodule.SwerveModule(5, 6, 2)
        self.backRight = swervemodule.SwerveModule(7, 8, 0)

        self.gyro = navx.AHRS.create_spi()

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )
        
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

        print(f"Absolute: {self.frontLeft.getAbsoluteAngle().radians()}")
        print(f"Relative: {self.frontLeft.getRelativeAngle().radians()}")


    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def resetToAbsolute(self) -> None:
        self.frontLeft.resetToAbsolute()
        self.frontRight.resetToAbsolute()
        self.backLeft.resetToAbsolute()
        self.backRight.resetToAbsolute