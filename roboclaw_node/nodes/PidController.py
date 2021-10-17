class PidController:
    def __init__(self, kP, kI, kD, kF, kIZone, kMaxOutput):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIZone = kIZone
        self.iState = 0
        self.prevErr = 0
        self.kMaxOutput = kMaxOutput

    def setPIDValues(self, kP, kI, kD, kF, kIZone, kMaxOutput):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIZone = kIZone
        self.kMaxOutput = kMaxOutput

    def calculate(self, setpoint, feedback, arbFF):
        # Error
        error = setpoint - feedback

        # P term
        p = error * self.kP

        # I term
        if (abs(error) <= self.IZone or self.IZone == 0.0):
            self.iState = self.iState + (error * self.kI)
        else:
            self.iState = 0

        # D term    
        d = (error - self.prevErr)
        self.prevErr = error
        d *= self.kD

        # F term
        f = setpoint * self.kF

        # Sum of terms
        output = p + self.iState + d + f + arbFF

        # Limit output to max output
        if (output > 0):
            output = min(output, self.kMaxOutput)
        else:
            output = max(output, -self.kMaxOutput)

        return output



    def calculate(self, setpoint, feedback):
        return self.calculate(setpoint, feedback, 0)

