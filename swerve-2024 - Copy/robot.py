import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from components import drivetrain
import navx

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        self.joystick = wpilib.Joystick(0)
        self.swerve = drivetrain.Drivetrain()
        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        def robotPeriodic(self):
            pass

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getY(), 0.5)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getX(), 0.5)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            self.rotLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getZ(), 0.5)
            )
            * drivetrain.kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def teleopInit(self) -> None:
        # self.swerve.resetToAbsolute()
        pass
    
    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True) 

    def robotPeriodic(self):
        pass