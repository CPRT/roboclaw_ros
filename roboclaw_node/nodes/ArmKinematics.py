from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

ARM_BASE_LENGTH = 0.30  # m, pivot to pivot
ARM_UPPER_LENGTH = 0.30  # m, pivot to end-effector axis


def validate_point(pose: Point) -> bool:
    return math.hypot(pose.x, pose.y, pose.z) < ARM_UPPER_LENGTH + ARM_BASE_LENGTH


def calculate_joint_state(pose: Point):
    if not validate_point(pose):
        raise ValueError("Invalid target point")
    print("Target Point:", pose.x, pose.y, pose.z)
    # Identify base/turrent rotation:
    base_angle = math.atan2(pose.y, pose.x)
    print("Base Angle:", math.degrees(base_angle))

    # Identify elevation angle of lower arm
    vector_length = math.hypot(pose.x, pose.y, pose.z)  # length of pose vector
    shoulder_angle_pt1 = math.acos((ARM_UPPER_LENGTH**2 - vector_length**2 -
                                   ARM_BASE_LENGTH**2)/(-2.0 * ARM_BASE_LENGTH * vector_length))  # cosine law
    shoulder_angle_pt2 = math.atan2(pose.z, math.sqrt(pose.x**2+pose.y**2))
    shoulder_angle = shoulder_angle_pt1 + shoulder_angle_pt2
    print("Shoulder Angle:", math.degrees(shoulder_angle))

    # Identify elevation angle of upper arm
    elbow_angle = math.acos((vector_length**2 - ARM_UPPER_LENGTH**2 -
                               ARM_BASE_LENGTH**2)/(-2.0 * ARM_BASE_LENGTH * ARM_UPPER_LENGTH))  # cosine law
    print("Elbow Angle:", math.degrees(elbow_angle))

    # Calculate arm vectors
    # length of lower arm vector projected onto ground plane
    xy_length = (ARM_BASE_LENGTH*math.sin(math.pi/2 -
                 shoulder_angle))/math.sin(math.pi/2)

    # Lower arm:
    lower_z_comp = (ARM_BASE_LENGTH*math.sin(shoulder_angle)) / \
        math.sin(math.pi/2)
    lower_y_comp = (xy_length*math.sin(base_angle))/math.sin(math.pi/2)
    lower_x_comp = (xy_length*math.sin(math.pi/2-base_angle)) / \
        math.sin(math.pi/2)
    print("Lower Arm Vector:", lower_x_comp, lower_y_comp, lower_z_comp)

    # Upper arm:
    upper_z_comp = pose.z - lower_z_comp
    upper_y_comp = pose.y - lower_y_comp
    upper_x_comp = pose.x - lower_x_comp
    print("Upper arm Vector:", upper_x_comp, upper_y_comp, upper_z_comp)

    return [base_angle, shoulder_angle, elbow_angle]


def inverseKinematics(pose:Pose):
    motorAngles = [0,0,0,0,0,0]

    # Calculate the angles of the turret, joint 1 and joint 2
    bottomAngles = calculate_joint_state(pose.position)

    motorAngles[0] = bottomAngles[0]
    motorAngles[1] = bottomAngles[1]
    motorAngles[2] = bottomAngles[2]


    # A pose object has a quaternion as part of it. 
    # Gonna borrow 3 of the 4 float64 in the quaternion as the motor angles
    # x = motor 3 (carbon fibre tube), y = motor 4 (wrist), z = motor 5 (spinny end effector)
    motorAngles[3] = pose.orientation.x
    motorAngles[4] = pose.orientation.y
    motorAngles[5] = pose.orientation.z





