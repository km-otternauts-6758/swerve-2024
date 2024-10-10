#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#from msilib.schema import Component
from ntcore import NetworkTableInstance
from wpilib.cameraserver import CameraServer
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from components import arm
from components import Shooter
from components import intakemotor
from dataclasses import dataclass, fields
from wpilib import DutyCycle, SmartDashboard
from rev import CANSparkMax
from networktables import NetworkTables
from components.limelight import LimeLight
from components.drivetrain import Drivetrain
from components.motorgroup import MotorGroup
import math


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.joystick = wpilib.Joystick(0)
        self.intake = intakemotor.MotorIntake(8)
        self.shooter = Shooter.ReciprocalMotors(11, 7)
        self.shoulder = arm.ReciprocalMotors(10,9)
    #Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)
    #Hex Encoder and pdp
        self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))
        self.pdp = wpilib.PowerDistribution()
    #Networktables and Limelight Init
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getTable("limelight-kmrobot")

            #hex encoder stuff
        self.dutyCycleEncoder = wpilib.DutyCycleEncoder(0)
        self.dutyCycleEncoder.setDistancePerRotation(360)
        # self.dutyCycleEncoder.reset()
        
        self.driveStick = wpilib.XboxController(1)
        self.robotDrive = Drivetrain()
        self.autonTimer = wpilib.Timer()
        self.autonTimer.reset()
#Auton Chooser
        self.auton = 1
        self.autonSteps = 1
        wpilib.CameraServer.launch()
        
        # self.limelight = LimeLight()
        # self.limelight.robotInit()
        self.ty = self.limelight.getEntry("ty")
        self.limelight.putValue("priorityid", 4)

    def  setMotors(self, forward:float, turn: float):
        self.robotDrive.drive(forward, turn)    

    def autonomousInit(self) -> None:
        self.autonTimer.reset()
        self.autonTimer.start()
        self.limelight.setDefaultValue("priorityid", 4)
        # self.automodes.start()
        # self.dutyCycleEncoder.reset()

#Auton Periodic
    def autonomousPeriodic(self) -> None:
        # print(self.limelight.getNumber("tx", 0))
        print("Hex: ", self.dutyCycle.getOutput())
        print("AutoAimAngle: ", self.autoAim)

        if self.auton == 6758:
            print("Priority Id: ", self.limelight.getNumber("priorityid", 4))


#Auton
        if self.auton == 1:
                if self.autonSteps == 1:
                    self.shooter.set(0.65)
                    self.shoulder.set(-.5)
                    if self.dutyCycle.getOutput() >= .59:
                        self.shoulder.set(0)  

                    if self.autonTimer.get() >= 2:
                            self.intake.set(1)
                            self.autonSteps = 2
                if self.autonSteps == 2:
                    if self.autonTimer.get() >= 4:
                        self.shooter.set(-.25)
                        self.shoulder.set(-.45)
                    if self.autonTimer.get() >=11.5:
                        self.setMotors(.4,0)

                        if self.dutyCycle.getOutput() >= .69:
                            self.shoulder.set(0)
                            self.shooter.set(0)
                            self.intake.set(1)
                        if self.autonTimer.get() >= 14:
                            self.autonSteps = 3
                if self.autonSteps == 3:
                        if self.autonTimer.get() >=14:
                                self.setMotors(0,0)
                                self.shooter.set(0)
                                self.intake.set(0)
                                self.shoulder.set(0)
                                self.autonSteps =4
#                 if self.autonSteps == 4:
# #                   self.setMotors(-.4, 0)
#                     self.shoulder.set(.5)
#                     if self.dutyCycle.getOutput() <= self.autoAim:
#                         self.shoulder.set(0)
#                         self.shooter.set(.65)
#                         self.autonSteps = 5
#                 if self.autonSteps == 5:
#                     if self.autonTimer.get() >= 7.5:
#                         self.intake.set(1)
#                         if self.autonTimer.get() >= 13:
#                              self.intake.set(0)
#                              self.shooter.set(0)
                            
#End Auton 1

            # if self.limelight.getNumber("tv", 0):
            #     self.shooter.set(0.65)
            #     self.shoulder.set(-.5)

            #     if self.dutyCycle.getOutput() >= .595:
            #          self.shoulder.set(0)  

            #     if self.autonTimer.get() >= 2:
            #             self.intake.set(1)

            #     if self.autonTimer.get() >=2.5:
            #         self.setMotors(.4,0)
            #         self.shooter.set(-0.25)
            #         self.shoulder.set(-.75)

            #     if self.dutyCycle.getOutput() >= .695:
            #             self.shoulder.set(0)
            #             self.intake.set(0)
            #             self.intake.set(1)

            #     if self.autonTimer.get() >=6:
            #         self.setMotors(0,0)
            #         self.shooter.set(0)
            #         self.intake.set(0)
            
            #     if self.autonTimer.get() >=6.6:
            #         self.setMotors(-.7,0)

            #     if self.autonTimer.get() >=7:
            #         self.setMotors(0,0)
            #         self.shoulder.set(0)
 #Back up, turns left, mainly for red team on the opposite side of amp(the vertical input box/little thing).       
        elif self.auton == 2:
                if self.autonSteps == 1:
                    self.shooter.set(0.65)
                    self.shoulder.set(-.5)

                    if self.dutyCycle.getOutput() >= .593:
                        self.shoulder.set(0)  

                    if self.autonTimer.get() >= 2:
                        self.intake.set(1)

                    if self.autonTimer.get() >= 2.1:
                        self.setMotors(.4, 0)

                    if self.autonTimer.get() >=2.5:
                        self.setMotors(0, -0.2)
                        # self.shooter.set(-0.15)
                        # self.shoulder.set(-.6)
                        self.autonSteps = 2
                if self.autonSteps == 2:
                    if self.autonTimer.get() >= 2.7:
                        self.setMotors(0.4, 0)
                    
                    if self.autonTimer.get() >= 3:
                        self.setMotors(0, 0)
                    # if self.dutyCycle.getOutput() >= .695:
                    #         self.shoulder.set(0)
                    #         self.intake.set(0)
                    #         self.intake.set(1)

                    if self.autonTimer.get() >=6:
                        self.setMotors(0,0)
                        # self.intake.set(0)

                    # if self.autonTimer.get() >=6.6:
                    #     self.setMotors(-.7,0)

                    # if self.autonTimer.get() >=7:
                    #     self.setMotors(0,0)
                    #     self.shoulder.set(.8)

#Back up, turns right, mainly for blue team on the opposite side of amp(the vertical input box/little thing).
        elif self.auton == 3:            
                if self.autonSteps == 1:
                    self.shooter.set(0.65)
                    self.shoulder.set(-.5)

                    if self.dutyCycle.getOutput() >= .593:
                        self.shoulder.set(0)  

                    if self.autonTimer.get() >= 2:
                        self.intake.set(1)

                    if self.autonTimer.get() >= 2.1:
                        self.setMotors(.4, 0)

                    if self.autonTimer.get() >=2.5:
                        self.setMotors(0, 0.2)
                        # self.shooter.set(-0.15)
                        # self.shoulder.set(-.6)
                        self.autonSteps = 2
                if self.autonSteps == 2:
                    if self.autonTimer.get() >= 2.7:
                        self.setMotors(0.4, 0)
                    
                    if self.autonTimer.get() >= 3:
                        self.setMotors(0, 0)
                    # if self.dutyCycle.getOutput() >= .695:
                    #         self.shoulder.set(0)
                    #         self.intake.set(0)
                    #         self.intake.set(1)

                    if self.autonTimer.get() >=6:
                        self.setMotors(0,0)
                        # self.intake.set(0)

                    # if self.autonTimer.get() >=6.6:
                    #     self.setMotors(-.7,0)

                    # if self.autonTimer.get() >=7:
                    #     self.setMotors(0,0)
                    #     self.shoulder.set(.8)
#Red Side, Close to Amp
        elif self.auton == 4:
                if self.autonSteps == 1:
                    self.shooter.set(0.65)
                    self.shoulder.set(-.5)

                    if self.dutyCycle.getOutput() >= .5925:
                        self.shoulder.set(0)  

                    if self.autonTimer.get() >= 2:
                        self.intake.set(1)

                    if self.autonTimer.get() >= 4:
                        self.setMotors(0, 0.2) 
                    if self.autonTimer.get() >= 4.5:
                        self.autonSteps = 2

                    if self.autonSteps == 2:
                        self.shooter.set(0)
                        
                        if self.autonTimer.get() >= 5:
                            self.autonSteps = 3
                if self.autonSteps == 3:
                    if self.autonTimer.get() >= 5:
                        self.setMotors(0.4, 0)
                        self.intake.set(0)
                        self.autonSteps = 4
                if self.autonSteps == 4:
                    if self.autonTimer.get() >= 6.5:
                        self.setMotors(0, 0)
                    # if self.dutyCycle.getOutput() >= .695:
                    #         self.shoulder.set(0)
                    #         self.intake.set(0)
                    #         self.intake.set(1)
                        # self.intake.set(0)

                    # if self.autonTimer.get() >=6.6:
                    #     self.setMotors(-.7,0)

                    # if self.autonTimer.get() >=7:
                    #     self.setMotors(0,0)
                    #     self.shoulder.set(.8)
#             if self.autonTimer.get() >=7.2:
#                 self.shoulder.set(0)
#                 self.shooter.set(0.7)

#             if self.autonTimer.get() >= 13:
#                     self.intake.set(1)
                        
#Blue Team Amp Side
        elif self.auton == 5:
                if self.autonSteps == 1:
                    self.shooter.set(0.65)
                    self.shoulder.set(-.5)

                    if self.dutyCycle.getOutput() >= .5925:
                        self.shoulder.set(0)  

                    if self.autonTimer.get() >= 2:
                        self.intake.set(1)

                    if self.autonTimer.get() >= 2.1:
                        self.setMotors(.4, -.2) 
                    if self.autonTimer.get() >= 2.4:
                        self.autonSteps = 2

                    if self.autonSteps == 2:
                        self.shooter.set(0)
                        
                        if self.autonTimer.get() >= 2.6:
                            self.autonSteps = 3
                if self.autonSteps == 3:
                    if self.autonTimer.get() >= 2.6:
                        self.setMotors(0.4, 0)
                        self.intake.set(0)
                    
                    if self.autonTimer.get() >= 2.9:
                        self.setMotors(0, 0)
#Shoot and wait.
        elif self.auton == 6:
                if self.autonSteps == 1:
                    self.shooter.set(0.65)
                    self.shoulder.set(-.5)
                    if self.dutyCycle.getOutput() >= .59:
                        self.shoulder.set(0)  

                    if self.autonTimer.get() >= 2:
                            self.intake.set(1)
                            self.autonSteps = 2
                if self.autonSteps == 2:
                    self.intake.set(0)
                    self.shooter.set(0)
                    if self.autonTimer.get() >=11:
                        self.setMotors(.4,0)
                        if self.autonTimer.get() >= 13.7:
                             self.setMotors(0,0)
#Amp Auton
        elif self.auton == 7:
            if self.autonSteps == 1:
                self.setMotors(-.4, 0)
                self.shoulder.set(0.3)
                if self.dutyCycle.getOutput() <= 0.48:
                    self.shoulder.set(0)
                    self.autonSteps = 2
            if self.autonSteps == 2:
                if self.autonTimer.get() >= 3:
                    self.setMotors(0,0)
                    self.shooter.set(0.4)
                    self.intake.set(1)

    def robotPeriodic(self) -> None:
#Soft stop.
        # if self.dutyCycle.getOutput() >= .71 or self.dutyCycle.getOutput() <= .43:
        #     self.shoulder.set(0)
#Distance Estimator
        targetOffsetAngleVertical = self.ty.getDouble(0.0)
        limelightMountAngleDegrees = 30.0
        limelightLensHeightInches = 11.5
        goalHeightInches = 78.0

        self.angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngleVertical
        angleToGoalRadians = self.angleToGoalDegrees * (3.14159 / 180.0)

        self.distance = (goalHeightInches - limelightLensHeightInches) / math.tan(angleToGoalRadians)

#team 1716 gave us ethernet switch, thank you.
#Angle Estimator
        self.autoAim = (0.6401 *  pow(0.9995, self.distance))


    def teleopInit(self) -> None:
#Shoulder Limit
        pass


    def teleopPeriodic(self) -> None:
        # print("Normal Offset: ", self.limelight.getNumber("ty", 0.0))
        # print("Double Offset: ", self.ty.getDouble(0.0))
        # print("Hex: ", self.dutyCycle.getOutput())
        print("Distance: ", self.distance)
        print("Hex: ", self.dutyCycle.getOutput())
        print("AutoAimAngle: ", self.autoAim)
        # print(self.distance)
        # print (self.limelight.getEntry("ty"))
        # print (self.distance)
    
#Shoulder Limit
        # if self.dutyCycle.getOutput() <= .39 >= 0:
        #     self.shoulder.set(0)

#Camera Server
        wpilib.CameraServer.launch('vision.py:main')
        CameraServer.is_alive()

        self.robotDrive.drive(-self.driveStick.getRawAxis(1), -self.driveStick.getRawAxis(0))
       
#Intake
        if self.joystick.getRawButton(2):
            self.intake.set(2)
        else:
            self.intake.set(0)
#Shooter speed.
        if self.joystick.getRawButton(1):
            self.driveStick.setRumble(self.driveStick.RumbleType.kBothRumble, 1)
            self.shooter.set(self.joystick.getRawButton(1) * .65)  
            # print("Hex: ", self.dutyCycle.getOutput())
            # print("AutoAimAngle: ", self.autoAim)  
        elif self.joystick.getRawButton(3):
            self.driveStick.setRumble(self.driveStick.RumbleType.kBothRumble, 1)
            self.shooter.set(-self.joystick.getRawButton(3))
        else:
            self.shooter.set(0)
            self.driveStick.setRumble(self.driveStick.RumbleType.kBothRumble, 0)
        
#Shoulder Speed
        self.shoulder.set(self.joystick.getY())

        
#Shooter Angle 1
        if self.joystick.getRawButton(6):
            self.shoulder.set(0.2)
            if self.dutyCycle.getOutput() <= .59:
                 self.shoulder.set(0)
                
#Amp Angle
        if self.joystick.getRawButton(10):
            self.shoulder.set(0.3)
            if self.dutyCycle.getOutput() <= 0.48:
                self.shoulder.set(0)

#Align w/ Apriltag
        if self.driveStick.getAButton():  
            if self.limelight.getNumber("tx", 0) < -4.5:
                self.setMotors(.1, .4)

            elif self.limelight.getNumber("tx", 0) > 4.5:
                self.setMotors(.1, -.4)

            elif self.limelight.getNumber("tx", 0) >= -1 and self.limelight.getNumber("tx", 0) <= 1:
                self.setMotors(0,0)
                
# Aim w/ Apriltag    
        if self.joystick.getRawButton(7):
            self.shoulder.set(0.2)
            if self.dutyCycle.getOutput() <= self.autoAim:
                 self.shoulder.set(0)