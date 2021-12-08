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



def calculateEndEffectorAngles(pose: Pose, bottomAngles):
    endEffectorAngles = euler_from_quaternion(pose.quaternion)
    pitch = 0
    roll = 0
    yaw = 0

    turret = bottomAngles[0]
    shoulder = bottomAngles[1]
    elbow = bottomAngles[2]

    point = [pose.point.x, pose.point.y, pose.point.z]

    ## Wrist angle. Angle between endeffector and carbon fiber tube
    arm1Vector = vectorFromAngles(shoulder, turret, 0.3251)
    arm12OffsetVector = vectorFromAngles(elbow-shoulder+81.534213, turret, 0.07199)

    arm2Vector = vectorSubtraction(vectorAddition(arm1Vector, arm12OffsetVector), point)

    arm2UnitVector = unitVector(arm2Vector)

    endEffectorUnitVector = [math.cos(pitch)*math.cos(yaw), 
                             math.cos(pitch)*math.sin(yaw),
                             math.sin(pitch)]

    wrist = angleBetweenUnitVectors(arm2UnitVector, endEffectorUnitVector)


    ## Carbon fiber tube motor angle. Rotation along axis of the carbon fiber tube
    # Point projected onto a plane. (point - dotproduct(point, normal) * normal)
    projectedPoint = vectorSubtraction(endEffectorUnitVector, vectorScalarMultiplication(arm2UnitVector, dotProduct(endEffectorUnitVector, arm2UnitVector)))

    offsetPoint = [pose.x, pose.y, pose.z+1]

    projectedOffsetPoint = vectorSubtraction(offsetPoint, vectorScalarMultiplication(arm2UnitVector, dotProduct(offsetPoint, arm2UnitVector)))

    tubeTwistAngle = angleBetweenUnitVectors(unitVector(projectedOffsetPoint), unitVector(projectedPoint))

    return [tubeTwistAngle, wrist, roll]

def vectorFromAngles(angleXY, angleZ, length):
    return [length*math.cos(angleZ)*math.cos(angleXY), 
            length*math.cos(angleZ)*math.sin(angleXY),
            length*math.sin(angleZ)]

def vectorAddition(vector1, vector2):
    return [vector1[0]+vector2[0], vector1[1]+vector2[1], vector1[2]+vector2[2]]

def vectorSubtraction(vector1, vector2):
    return [vector1[0]-vector2[0], vector1[1]-vector2[1], vector1[2]-vector2[2]]

def vectorScalarMultiplication(vector1, scalar):
    return [vector1[0]*scalar, vector1[1]*scalar, vector1[2]*scalar]

def unitVector(vector):
    magnitude = math.hypot(vector[0], vector[1], vector[2])

    return [vector[0]/magnitude,
            vector[1]/magnitude,
            vector[2]/magnitude]

def angleBetweenUnitVectors(vector1, vector2):
    # Cross product between unit vectors

    crossProductVector = [vector1[1]*vector2[2]-vector1[2]*vector2[1],
                          vector1[2]*vector2[0]-vector1[0]*vector2[2],
                          vector1[0]*vector2[1]-vector1[1]*vector2[0]]

    crossProductMagnitude = math.hypot(crossProductVector[0], crossProductVector[1], crossProductVector[2])

    return math.sin(crossProductMagnitude)

def dotProduct(vector1, vector2):
    return vector1[0]*vector2[0] + vector1[1]*vector2[1] + vector1[2]*vector2[2]

def inverseKinematics(pose:Pose):
    motorAngles = [0,0,0,0,0,0]

    # Calculate the angles of the turret, joint 1 and joint 2
    bottomAngles = calculate_joint_state(pose.position)

    motorAngles[0] = bottomAngles[0]
    motorAngles[1] = bottomAngles[1]
    motorAngles[2] = bottomAngles[2]


    endEffectorAngles = calculateEndEffectorAngles(pose, bottomAngles)

    # x = motor 3 (carbon fibre tube), y = motor 4 (wrist), z = motor 5 (spinny end effector)
    motorAngles[3] = endEffectorAngles[0]
    motorAngles[4] = endEffectorAngles[1]
    motorAngles[5] = endEffectorAngles[2]




