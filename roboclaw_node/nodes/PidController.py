
# Math bits got from Rev robotics and ported into python
# https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control

class PidController:
    def __init__(self, kInvertOutput, kP, kI, kD, kF, kIZone, kMaxOutput):
        self.kInvertOutput = kInvertOutput
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIZone = kIZone
        self.iState = 0
        self.prevErr = 0
        self.kMaxOutput = kMaxOutput
        
        self.softLimitEnabled = False
        self.softLimitHigh = 0
        self.softLimitLow = 0

    # Max Output should be raw motor output units (voltage usually)
    def setPIDValues(self, kInvertOutput:bool, kP, kI, kD, kF, kIZone, kMaxOutput):
        self.kInvertOutput = kInvertOutput
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIZone = kIZone
        self.kMaxOutput = kMaxOutput

    # Soft limit is meant for position PID only
    # Will prevent movement beyond a limit when the limit is exceeded.
    def enableSoftLimit(self, enabled:bool):
        self.softLimitEnabled = enabled

    # Limits should be PID units
    def setSoftLimits(self, limitLow, limitHigh):
        self.softLimitLow = limitLow
        self.softLimitHigh = limitHigh
        self.softLimitEnabled = True

    def getSoftLimits(self):
        return self.softLimitLow, self.softLimitHigh

    # Setpoint and feedback should be PID units
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

        # Clamp output to given max value
        output = self.clampValue(output, -self.kMaxOutput, +self.kMaxOutput)

        # Invert the direction if needed
        if (self.kInvertOutput):
            output *= -1

        # Do soft limits if it's enabled
        if (self.softLimitEnabled):

            ## Check if a soft limit is exceeded and prevent movement towards if it is
            # If: past endstop in the positive direction and going in the positive direction
            if (feedback > self.softLimitHigh and output > 0):
                output = 0
                return output
            
            # If: past endstop in the negative direction and going in the negative direction
            if (feedback < self.softLimitLow and output < 0):
                output = 0
                return output

        return output


    
    def calculate(self, setpoint, feedback):
        return self.calculate(setpoint, feedback, 0)



    ##
    ## Helper methods
    ##

    # Clamp the voltage so it doesn't exceed a given max value
    def clampValue(self, value, minValue, maxValue):
        if (value > maxValue):
            return maxValue
        elif (value < minValue):
            return minValue

        return value

