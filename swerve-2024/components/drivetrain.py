import wpilib.drive
from components.motorgroup import MotorGroup


class Drivetrain:
    def __init__(self):
        self.rightGroup = MotorGroup(1, 2)
        self.leftGroup = MotorGroup(4, 3)

        self.rightGroup.setInverted(True)

        self.robotDrive = wpilib.drive.DifferentialDrive(self.leftGroup.getMotorControllerGroup(), self.rightGroup.getMotorControllerGroup()) 
    
    def drive(self, xSpeed: float, zRotation: float, squareInputs: bool = True): 
        self.robotDrive.arcadeDrive(xSpeed, zRotation, squareInputs)
    