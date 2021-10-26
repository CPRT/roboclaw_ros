#!/usr/bin/env python
from math import pi, cos, sin
import threading
from functools import partial

import diagnostic_msgs
import diagnostic_updater
from nodes.PidController import PidController
from roboclaw_driver.roboclaw_driver import Roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from numbers import Number

from ArmKinematics import inverseKinematics
from GravityCompensation import gravityCompensation

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


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

        # 
        # Defining stuff
        #
        
        # Tunable constants of PID, should be live tunable
        self.pidControllers = [PidController(),
                            PidController(),
                            PidController(),
                            PidController(),
                            PidController(),
                            PidController()]

        # Effects the influence of gravity comp on the arms, should be live tunable
        self.gravityCompNewtonMetersToVoltage = 0



        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "19200"))

        self.addresses = [int(addr) for addr in rospy.get_param("~addresses", "131,132,133").split(",")]

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

        # We have a single 'roboclaw' object handling serial communications.
        # We're about to launch different threads that each want to talk.
        # 1 - Diagnostics thread calling into our self.check_vitals
        # 2 - '/cmd_vel' thread calling our self.cmd_vel_callback
        # 3 - our self.run that publishes to '/odom'
        # To prevent thread collision in the middle of serial communication
        # (which causes sync errors) all access to roboclaw from now
        # must be synchronized using this mutually exclusive lock object.
        self.mutex = threading.Lock()

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")

        for address in self.addresses:
            self.updater.add(diagnostic_updater.
                            FunctionDiagnosticTask(f"[{address}] Vitals", partial(self.check_vitals, address=address)))

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

        for address in self.addresses:
            with self.mutex:
                self.roboclaw.SpeedM1M2(address, 0, 0)
                self.roboclaw.ResetEncoders(address)

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "4342.2"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("addresses %d", ",".join(self.addresses))
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                for address in self.addresses:
                    with self.mutex:
                        try:
                            self.roboclaw.ForwardM1(address, 0)
                            self.roboclaw.ForwardM2(address, 0)
                        except OSError as e:
                            rospy.logerr(f"[{address}] Could not stop")
                            rospy.logdebug(e)

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                with self.mutex:
                    status1, enc1, crc1 = self.roboclaw.ReadEncM1(self.addresses[0])
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                with self.mutex:
                    status2, enc2, crc2 = self.roboclaw.ReadEncM2(self.addresses[0])
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            if (isinstance(enc1,Number) and isinstance(enc2,Number)):
                rospy.logdebug("Encoders %d %d" % (enc1, enc2))
                self.encodm.update_publish(enc2, enc1) # update_publish(enc_left, enc_right)

                self.updater.update()
            r_time.sleep()

    def cmd_arm_callback(self, pose):
        self.last_set_speed_time = rospy.get_rostime()

        encoderTicksPerRotation = 0 # CONSTANT DEFINE SOMEWHERE
        gearReduction = [0,0,0,0,0,0] # CONSTANT DEFINE SOMEWHERE
        invertMotorDirection = [1,1,1,1,1,1] # CONSTANT DEFINE SOMEWHERE
        invertEncoderDirection = [1,1,1,1,1,1] # CONSTANT DEFINE SOMEWHERE
        maxVoltage = [0,0,0,0,0,0] # CONSTANT DEFINE SOMEWHERE
        maxMotorAngle = [0,0,0,0,0,0] # CONSTANT DEFINE SOMEWHERE
        minMotorAngle = [0,0,0,0,0,0] # CONSTANT DEFINE SOMEWHERE

        additionalMassOnEndEffector = 0 # Need to figure out how to make this an input

        motorAngles = inverseKinematics(pose)

        # Ensure values calculated do not go beyond mechanical endstops
        i = 0
        for angle in motorAngles:
            angle = self.clampValue(angle, minMotorAngle[i], maxMotorAngle[i])
            i += 1


        gravityCompVolts = gravityCompensation(motorAngles, additionalMassOnEndEffector, self.gravityCompNewtonMetersToVoltage)
        

        # rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)
        i = 0
        for address in self.addresses:
            try:
                with self.mutex:
                    encoderCount1 = self.roboclaw.ReadEncM1(address) * invertEncoderDirection[i]
                    encoderCount2 = self.roboclaw.ReadEncM2(address) * invertEncoderDirection[i+1]

                    setpoint1 = motorAngles[i]
                    feedback1 = encoderCount1 / encoderTicksPerRotation * 2*pi / gearReduction[i]  # GET ENCODER VALUE FROM ROBOCLAW & CONVERT TO RADIANS
                    voltage1 = (self.pidControllers[i].calculate(setpoint1, feedback1, gravityCompVolts[i])
                                * invertMotorDirection[i])

                    setpoint2 = motorAngles[i+1]
                    feedback2 = encoderCount2 / encoderTicksPerRotation * 2*pi / gearReduction[i+1]   # GET ENCODER VALUE FROM ROBOCLAW & CONVERT TO RADIANS
                    voltage2 = (self.pidControllers[i+1].calculate(setpoint2, feedback2, gravityCompVolts[i+1])
                                * invertMotorDirection[i+1])

                    voltage1 = self.clampVoltage(voltage1, -maxVoltage[i], maxVoltage[i])
                    voltage2 = self.clampVoltage(voltage2, -maxVoltage[i+1], maxVoltage[i+1])

                    self.setRoboclawVoltage(self.roboclaw, address, voltage1, voltage2)
                    
            except OSError as e:
                rospy.logwarn(f"[{address}] SpeedM1M2 OSError: {e.errno}")
                rospy.logdebug(e) 
                #TODO: add more error tracking for each roboclaw call

            i += 1

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat, address):
        try:
            status = self.roboclaw.ReadError(address)[1]
        except OSError as e:
            rospy.logwarn(f"[{address}] Diagnostics OSError: {e.errno}")
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, f"[{address}] {message}")
        try:
            stat.add("Main Batt V:", float(self.roboclaw.ReadMainBatteryVoltage(address)[1] / 10))
            stat.add("Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(address)[1] / 10))
            stat.add("Temp1 C:", float(self.roboclaw.ReadTemp(address)[1] / 10))
            stat.add("Temp2 C:", float(self.roboclaw.ReadTemp2(address)[1] / 10))
        except OSError as e:
            rospy.logwarn(f"[{address}] Diagnostics OSError: {e.errno}")
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        for address in self.addresses:
            try:
                with self.mutex:
                    self.roboclaw.ForwardM1(address, 0)
                    self.roboclaw.ForwardM2(address, 0)
            except OSError:
                rospy.logerr(f"[{address}] Shutdown did not work trying again")
                try:
                    with self.mutex:
                        self.roboclaw.ForwardM1(address, 0)
                        self.roboclaw.ForwardM2(address, 0)
                except OSError as e:
                    rospy.logerr(f"[{address}] Could not shutdown motors!!!!")
                    rospy.logdebug(e)

    # 
    # Helper methods
    # 

    # Clamp the voltage so it doesn't exceed a given max value
    def clampValue(self, value, minValue, maxValue):
        if (value > maxValue):
            return maxValue
        elif (value < minValue):
            return minValue

        return value

    # Convert voltage to dutycycle and pass on to both motors attached to the roboclaw
    def setRoboclawVoltage(self, roboclaw:Roboclaw, address:int, voltage1, voltage2):

        # MainBatteryVoltage/10 to get volts. 32767 is 100% duty cycle
        mainBatteryVoltage = roboclaw.ReadMainBatteryVoltage(address)
        dutyCycle1 = voltage1 / (mainBatteryVoltage / 10) * 32767
        dutyCycle2 = voltage2 / (mainBatteryVoltage / 10) * 32767

        roboclaw.DutyM1M2(address, dutyCycle1, dutyCycle2)



if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")











"""
IDEA/TODO

1. Subscribe to a topic to update PID values and gravity comp value
2. ~~DONE~~Write gravity comp
3. Organize constants
4. Create a spreadsheet of all constants
5. Define the PidControllers with values in rospy.param
6. Replace drivetrain odometry with arm forward kinematics
7. ~~DONE~~ Add a way to cap the output of the pid controller (currently could ask the roboclaw for >100% duty cycle)
8. Add a way to send a voltage value to a motor (for testing and tuning)
                ~~DONE~~Move the voltage math from the cmd_arm_callback into a method so both can use it
9. Create a control logic for the arm. Disabled, Home, SetVoltages, SetPose 
                Home: Go to a nice position to turn off at
10. Add a system to check that the values from inverse kinematics are within the range of the hardware
11. Subscribe to a topic to send voltage values to the roboclaws
12. Create soft limits. If the motor is outside a soft limit and moving in a direction farther away then prevent the motor
                from moving (set duty cycle to 0).
13. Create a safety feature so that if a the cmd_arm topic or cmd_arm_voltage topic are not given a value within 0.5s, 
                stop the motor from moving (duty cycle to 0)
14. Modify the end effector angles based on the turret angle



"""
