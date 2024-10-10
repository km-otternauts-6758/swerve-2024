import rev

class ReciprocalMotors:
    def __init__(self, leftMotorChannel: int, rightMotorChannel: int) -> None:
        self.leftMotor = rev.CANSparkMax(leftMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rightMotor = rev.CANSparkMax(rightMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rightMotor.follow(self.leftMotor, False)

    def set(self, speed: float) -> None:
        self.leftMotor.set(speed)
    