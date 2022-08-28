#!/usr/bin/env python
from json import encoder
from math import pi, cos, sin
import threading
from functools import partial

import diagnostic_msgs
import diagnostic_updater
from nodes.PidController import PidController
from roboclaw_driver.roboclaw_driver import Roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from numbers import Number

from std_msgs.msg import Int16 # IDK IF THIS WORKS. Might need to just make my own custom message

from ArmKinematics import inverseKinematics
from GravityCompensation import gravityCompensation

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"



# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?
class Node:
    def __init__(self):

        self.ERRORS = {0x000000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x000001: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "E-Stop"),
                       0x000002: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x000004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x000008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main Voltage High"),
                       0x000010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic Voltage High"),
                       0x000020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic Voltage Low"),
                       0x000040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Driver Fault"),
                       0x000080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Driver Fault"),
                       0x000100: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Speed"),
                       0x000200: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Speed"),
                       0x000400: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Position"),
                       0x000800: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Position"),
                       0x001000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Current"),
                       0x002000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Current"),
                       0x010000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 Over Current"),
                       0x020000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 Over Current"),
                       0x040000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main Voltage High"),
                       0x080000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main Voltage Low"),
                       0x100000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x200000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x400000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "S4 Signal Triggered"),
                       0x800000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "S5 Signal Triggered"),
                       0x01000000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Speed Error Limit"),
                       0x02000000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Position Error Limit")}

    

        ## 
        ## Defining stuff
        ##

        # Core variables to run this node
        self.loopCalculatePID = False
        self.isHoming = False
        self.HOME_SETPOINT_MOTORANGLES_RAD = [0,0,0,0,0,0]
        self.setpointMotorAngles = self.HOME_SETPOINT_MOTORANGLES_RAD
        self.REACHED_SETPOINT_TOLERANCE_RAD = 0.05236
        self.massOnEndEffector = 0

        # Tunable constants of PID, should be live tunable
        MAX_VOLTAGE = 12
        self.pidControllers = [ PidController(False, 0, 0, 0, 0, 0, MAX_VOLTAGE),
                                PidController(False, 0, 0, 0, 0, 0, MAX_VOLTAGE),
                                PidController(False, 0, 0, 0, 0, 0, MAX_VOLTAGE),
                                PidController(False, 0, 0, 0, 0, 0, MAX_VOLTAGE),
                                PidController(False, 0, 0, 0, 0, 0, MAX_VOLTAGE),
                                PidController(False, 0, 0, 0, 0, 0, MAX_VOLTAGE)]

        self.pidControllers[0].setSoftLimits(0, 0)
        self.pidControllers[1].setSoftLimits(0, 0)
        self.pidControllers[2].setSoftLimits(0, 0)
        self.pidControllers[3].setSoftLimits(0, 0)
        self.pidControllers[4].setSoftLimits(0, 0)
        self.pidControllers[5].setSoftLimits(0, 0)

        self.encoderTicksPerRotation = [0,0,0,0,0,0]
        self.gearReduction = [0,0,0,0,0,0] # on encoders
        self.invertEncoderDirection = [1,1,1,1,1,1]
        self.encoderOffset = [0,0,0,0,0,0]


        # Effects the influence of gravity comp on the arms, should be live tunable
        self.gravityCompNewtonMetersToVoltage = 0

        self.timeSinceCommandRecieved = rospy.get_rostime()   #rospy.Time.now()

        ## 
        ## Setup roboclaw 
        ##
        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "19200"))

        self.addresses = [int(addr) for addr in rospy.get_param("~addresses", "131,132,133").split(",")]

        # ERIK: Unsure what this is supposed to do. A better check would be to make sure 131, 132, 133 are there?
        if any((addr > 0x87 or addr < 0x80 for addr in self.addresses)):
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        self.roboclaw = Roboclaw(dev_name, baud_rate)
        # TODO need someway to check if address is correct
        try:
            self.roboclaw.Open()
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")


        # This is no longer needed. But maybe a diagostic updater would be helpful still??
        """
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        
        for address in self.addresses:
            self.updater.add(diagnostic_updater.
                            FunctionDiagnosticTask(f"[{address}] Vitals", partial(self.check_vitals, address=address)))
        """

        versions = []
        for address in self.addresses:
            try:
                versions.append(self.roboclaw.ReadVersion(address))
            except Exception as e:
                rospy.logwarn(f"[{address}] Problem getting roboclaw version")
                rospy.logdebug(e)
                pass

        for version in versions:
            if not version[0]:
                rospy.logwarn("Could not get version from roboclaw")
            else:
                rospy.logdebug(repr(version[1]))

        self.stopMotors()

        """ Keeping this temporarily to reference for the get_param
        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "4342.2"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))
        """

        rospy.Subscriber("cmd_arm_state", Int16, self.arm_state_callback) # ERIK: Idk if this Int16 works
        rospy.Subscriber("cmd_arm", Pose, self.cmd_arm_callback)
        rospy.Subscriber("arm_setVoltage", Pose, self.cmd_setVoltage_callback) # ERIK: Change Pose msg to 6 voltage values
        rospy.Subscriber("arm_liveTune", LiveTune, self.callback_liveTune) # ERIK: Change Pose msg.

        self.angles_pub = rospy.Publisher('/armAngles', Odometry, queue_size=10) # ERIK: Need to figure out the message here. 6 radian values
        # self.pose_pub = rospy.Publisher('/armPose', Pose, queue_size=10)

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("addresses %d", ",".join(self.addresses))




    # Subscriber to change the state of the arm. 
    # state should be an int
    # state == 0 -> Disable (emergency disable)
    # state == 1 -> Home then disable (safe disable)
    # state == 2 -> Enable
    def arm_state_callback(self, state):
        if (state == 0):
            self.stopMotors()
            self.loopCalculatePID = False
            self.isHoming = False

        if (state == 1):
            if (self.loopCalculatePID == False): 
                self.startLooping()
            self.loopCalculatePID = True
            self.isHoming = True
            self.setpointMotorAngles = self.HOME_SETPOINT_MOTORANGLES_RAD

        if (state == 2):
            if (self.loopCalculatePID == False):
                 self.startLooping()
            self.loopCalculatePID = True
            self.isHoming = False


    # Change the setpoint to a new value. The arm has to be enabled to go to this setpoint
    def cmd_arm_callback(self, pose):    
        if (self.isHoming == False):
            self.setpointMotorAngles = inverseKinematics(pose)

            # Ensure values calculated do not go beyond mechanical endstops
            i = 0
            for angle in self.setpointMotorAngles:
                lowLimit, highLimit = self.pidControllers[i].getSoftLimits()
                angle = self.clampValue(angle, lowLimit, highLimit)
                i += 1

    # Change the addiitonal mass that is on the End Effector
    def additionalMassOnEndEffector_callback(self, mass):
        self.massOnEndEffector = mass

    # This runs a heartbeat monitor so that if PID has not recalculated within given period of time, it stops the motors from moving
    def run(self): 
        r_time = rospy.Rate(5) # Every 0.2s, do a check
        while not rospy.is_shutdown():
            # If 1 second has pasted with no recalculation, stop the motors from running
            if (rospy.get_rostime() - self.timeSinceCommandRecieved).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                self.stopMotors()

            r_time.sleep()



    # This updates the heartbeat monitor so it knows it has received a command recently
    def feedMonitor(self):
        self.timeSinceCommandRecieved = rospy.get_rostime()

    def startLooping(self):
        while self.loopCalculatePID == True:

            # self.setpointMotorAngles from cmd_arm_callback(pose)
            motorAngles = self.setpointMotorAngles

            # Calculate gravity compensation
            gravityCompVolts = gravityCompensation(motorAngles, self.massOnEndEffector, self.gravityCompNewtonMetersToVoltage)
            
            #rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

            ##
            ## Loop through each roboclaw
            ##

            # Prepare to store all encoder positions
            encoderRadians = [0, 0, 0, 0, 0, 0]
            encoderRaw = [0, 0, 0, 0, 0, 0]

            i = 0
            for address in self.addresses:
                ##
                ## Check vitals. A basic 'motor controller is ok' check
                ##

                # Get error reported from roboclaw and log it
                statusMessage = ""
                try:
                    errorCode = self.roboclaw.ReadError(address)[1]
                except OSError as e:
                    rospy.logwarn(f"[{address}] Diagnostics OSError: {e.errno}")
                    rospy.logdebug(e)

                state, message = self.ERRORS[errorCode]
                statusMessage.summary(state, f"[{address}] {message}")

                # Store info about the voltage input to the roboclaw and board temperature 1 and 2
                try:
                    # MainBatteryVoltage/10 to get volts
                    mainBatteryVoltage = float(self.roboclaw.ReadMainBatteryVoltage(address)[1] / 10)

                    statusMessage.add("Main Batt V:", mainBatteryVoltage)
                    statusMessage.add("Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(address)[1] / 10))
                    statusMessage.add("Temp1 C:", float(self.roboclaw.ReadTemp(address)[1] / 10))
                    statusMessage.add("Temp2 C:", float(self.roboclaw.ReadTemp2(address)[1] / 10))
                except OSError as e:
                    rospy.logwarn(f"[{address}] Diagnostics OSError: {e.errno}")
                    rospy.logdebug(e)
                
                # ERIK: statusMessage contains a ton of information about the current status of the arm. Idk how to make that available to read.

                ##
                ## Store all encoder positions
                ##
                try:
                    encoderCount1 = float(self.roboclaw.ReadEncM1(address)[1]) * self.invertEncoderDirection[i]
                except OSError as e:
                    rospy.logwarn(f"[{address}] ReadEncM1 OSError: {e.errno}")
                    rospy.logdebug(e) 
                
                try:
                    encoderCount2 = float(self.roboclaw.ReadEncM2(address)[1]) * self.invertEncoderDirection[i+1]
                except OSError as e:
                    rospy.logwarn(f"[{address}] ReadEncM2 OSError: {e.errno}")
                    rospy.logdebug(e) 

                encoderRadians1 = encoderCount1 / self.encoderTicksPerRotation[i] * 2*pi / self.gearReduction[i]
                encoderRadians2 = encoderCount2 / self.encoderTicksPerRotation[i+1] * 2*pi / self.gearReduction[i+1]

                encoderRadians1 = encoderRadians1 + self.encoderOffset[i]
                encoderRadians2 = encoderRadians2 + self.encoderOffset[i+1]

                encoderRadians[i] = encoderRadians1
                encoderRadians[i+1] = encoderRadians2

                encoderRaw[i] = encoderCount1
                encoderRaw[i+1] = encoderCount2

                ##
                ## Calculate PID + gravity compensation
                ## Convert voltage to duty cycle and send command to roboclaw
                ##

                # Calculate grav comp and PID for motor 1
                setpoint1 = motorAngles[i]
                feedback1 = encoderRadians1
                voltage1 = self.pidControllers[i].calculate(setpoint1, feedback1, gravityCompVolts[i])

                # Calculate grav comp and PID for motor 2
                setpoint2 = motorAngles[i+1]
                feedback2 = encoderRadians2
                voltage2 = self.pidControllers[i+1].calculate(setpoint2, feedback2, gravityCompVolts[i+1])

                # 32767 is 100% duty cycle (15 bytes)
                dutyCycle1 = voltage1 / mainBatteryVoltage * 32767
                dutyCycle2 = voltage2 / mainBatteryVoltage * 32767

                # Send the command to the roboclaw
                try:
                    self.roboclaw.DutyM1M2(address, dutyCycle1, dutyCycle2)
                except OSError as e:
                    rospy.logwarn(f"[{address}] DutyM1M2 OSError: {e.errno}")
                    rospy.logdebug(e) 

                # Feed so the heartbeat monitor knows it got a pulse 
                self.feedMonitor()

                # Iterate the loop
                i += 2

            ## 
            ## Forward kinematics
            ## ERIK: This needs work... and math...
            ##
            
            # Publish motor angles
            self.angles_pub.publish(motorAngles)

            # Publish Forward Kinematics
            # self.pose_pub.publish() # Pose here


            if (self.isHoming): 
                if (self.reachedSetpoint(self.HOME_SETPOINT_MOTORANGLES_RAD, encoderRadians)):
                    self.arm_state_callback(0)
                    break

        # Loop ended means arm got disabled
        self.stopMotors()

    def cmd_setVoltage_callback(self, Pose): # CHANGE POSE
        voltages = [0, 0, 0, 0, 0 ,0]

        i = 0
        for address in self.addresses:

            try:
                # MainBatteryVoltage/10 to get volts
                mainBatteryVoltage = self.roboclaw.ReadMainBatteryVoltage(address)[1] / 10
            except OSError as e:
                rospy.logwarn(f"[{address}] Diagnostics OSError: {e.errno}")
                rospy.logdebug(e)

            # 32767 is 100% duty cycle (15 bytes)
            dutyCycle1 = voltages[i] / mainBatteryVoltage * 32767
            dutyCycle2 = voltages[i+1] / mainBatteryVoltage * 32767

            # Send the command to the roboclaw
            try:
                self.roboclaw.DutyM1M2(address, dutyCycle1, dutyCycle2)
            except OSError as e:
                rospy.logwarn(f"[{address}] DutyM1M2 OSError: {e.errno}")
                rospy.logdebug(e) 

            # Feed so the heartbeat monitor knows it got a pulse 
            self.feedMonitor()

            i += 2

    
    def callback_liveTune(self, message: LiveTune):
        #P, I, D, IZone, InvertOutput, InvertEncoder, EncoderOffset -> for a specific motor (address and M1 or M2)
        """ LiveTune message
        
        int armMotorNumber   (0 to 5)
        string command
        float value

        Types of commands:

        "s" or "e" or ""   stops the motors from moving

        "p", "i", "d" or "iZ"    changes the values of the constants for PID + IZone
        
        "mI", "eI" or "eO"   sets the values for motorInvert, encoderInvert and encoderOffset     

        "sH", "sL"   sets the values for softLimits, high and low

        "gc"  sets gravity compensation constant

        "v" sets a voltage of a given motor

        "rad" sets a radian setpoint for the PID of a given motor
        """
        message.command = message.command.lower()

        if (message.command == "s" or message.command == "e" or message.command == ""):
            self.stopMotors

        elif (message.command == "p"):
            self.pidControllers[message.armMotorNumber].setP = message.value
        elif (message.command == "i"):
            self.pidControllers[message.armMotorNumber].setI = message.value
        elif (message.command == "d"):
            self.pidControllers[message.armMotorNumber].setD = message.value
        elif (message.command == "iZ"):
            self.pidControllers[message.armMotorNumber].setIZone = message.value

        elif (message.command == "mI"):
            self.pidControllers[message.armMotorNumber].setInvertOutput = (message.value == 1) # 0 = false, 1 = true
        elif(message.command == "eI"):
            self.invertEncoderDirection[message.armMotorNumber] = int(message.value)
        elif(message.command == "eO"):
            self.encoderOffset[message.armMotorNumber] = message.value
        
        elif (message.command == "sH"):
            self.pidControllers[message.armMotorNumber].setSoftLimitHigh(message.value)
        elif (message.command == "sL"):
            self.pidControllers[message.armMotorNumber].setSoftLimitLow(message.value)

        elif(message.command == "gc"):
            self.gravityCompNewtonMetersToVoltage = message.value

        elif(message.command == "v"):
            voltages = [0] * 6
            voltages[message.armMotorNumber] = message.value
            # CONVERT VOLTAGE LIST TO 6 RAD VALUE ROS MSG
            self.cmd_setVoltage_callback(voltages)

        elif(message.command == "rad"):
            self.setpointMotorAngles = self.HOME_SETPOINT_MOTORANGLES_RAD

            lowLimit, highLimit = self.pidControllers[message.armMotorNumber].getSoftLimits()
            angle = self.clampValue(message.value, lowLimit, highLimit)

            self.setpointMotorAngles[message.armMotorNumber] = angle




    # This will stop the motors until the topic receives a new value
    def stopMotors(self, address):
        try:
            self.roboclaw.DutyM1M2(address, 0, 0)
        except OSError as e:
            rospy.logwarn(f"[{address}] stopMotors DutyM1M2 OSError: {e.errno}")
            rospy.logdebug(e)

    # Stops all motors until the topic receives a new value
    def stopMotors(self):
        for address in self.addresses:
            self.stopMotors(address)

    # ERIK: I want to write a shutdown system that's different than a temporary stop (ie prevents further action until told otherwise)
    def shutdown(self):
        rospy.loginfo("Shutting down")
        for address in self.addresses:
            try:
                self.roboclaw.ForwardM1(address, 0)
                self.roboclaw.ForwardM2(address, 0)
            except OSError:
                rospy.logerr(f"[{address}] Shutdown did not work trying again")
                try:
                    self.roboclaw.ForwardM1(address, 0)
                    self.roboclaw.ForwardM2(address, 0)
                except OSError as e:
                    rospy.logerr(f"[{address}] Could not shutdown motors!!!!")
                    rospy.logdebug(e)


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

    # Check if each motor has reached its setpoint within a tolarable error
    def reachedSetpoint(self, setpoint, motorAngles):
        i = 0
        for angle in motorAngles:
            if (abs(setpoint[i] - motorAngles[i]) > self.REACHED_SETPOINT_TOLERANCE_RAD):
                return False
            i+=1

        return True

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")


    




#LeftOvers from drivetrain roboclaw node that may still be useful

"""
@staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle



rospy.Time.now()


"""


"""
IDEA/TODO

1. ~~DONE~~Subscribe to a topic to update PID values and gravity comp value
2. ~~DONE~~Write gravity comp
3. ~~DONE~~Organize constants
4. ~~DONE~~Create a spreadsheet of all constants

5. ***Define the PidControllers with values in rospy.param (and other constants)

6. ~~DONE~~ Replace drivetrain odometry with arm forward kinematics
7. ~~DONE~~ Add a way to cap the output of the pid controller (currently could ask the roboclaw for >100% duty cycle)
8. ~~DONE~~Add a way to send a voltage value to a motor (for testing and tuning)
9. ~~DONE~~Create a control logic for the arm. Disabled, Home, SetVoltages, SetPose 
                ~~DONE~~Home: Go to a nice position to turn off at
10. ~~DONE~~ Add a system to check that the values from inverse kinematics are within the range of the hardware
11. ~~DONE~~Subscribe to a topic to send voltage values to the roboclaws

***Still need to change the MSG

12. ~~DONE~~ Create soft limits. If the motor is outside a soft limit and moving in a direction farther away then prevent the motor
                from moving (set duty cycle to 0).
13. ~~DONE~~Create a safety feature so that if a the cmd_arm topic or cmd_arm_voltage topic are not given a value within 0.5s, 
                stop the motor from moving (duty cycle to 0)
14. ~~DONE~~Modify the end effector angles based on the turret angle
15. ~~DONE~~Make sure encoders are setup correctly (do they need to be reset? I hope not. They should be set once or they I need an offset to redefine what 0 to the encoder means)
                ~~DONE~~What's really important here is that what the encoder defines as 0 matches what the kinematics defines as 0. Likely horizontal, front facing is 0.



"""
