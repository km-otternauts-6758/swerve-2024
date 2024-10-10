from rev import CANSparkMax

class ReciprocalMotors:
    def __init__(self, leftMotorChannel: int, rightMotorChannel: int) -> None:
        self.leftMotor = CANSparkMax(leftMotorChannel, CANSparkMax.MotorType.kBrushless)
        self.rightMotor = CANSparkMax(rightMotorChannel, CANSparkMax.MotorType.kBrushless)
        self.rightMotor.follow(self.leftMotor, True)

    def set(self, speed: float) -> None:
        self.leftMotor.set(speed)
    