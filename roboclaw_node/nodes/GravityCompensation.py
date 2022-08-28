from math import cos

# 
# Constants
#

# Constant of gravity in m/s/s
gravityConstant = 9.81

# Length along the first arm to the centroid of that arm
lengthCentroid1 = 0.344

# Length along the second arm to the centroid of that arm
lengthCentroid2 = 0.254

# Length along the second arm to the centroid the end effector
# Assumed that the mass of the end effector is along the axis of the second arm
lengthCentroid3 = 0.55

# Length between the first and second joint
lengthArm1 = 0.325

# Mass of the first arm, between joint 1 and joint 2
massArm1 = 1.5

# Mass of the second arm, between joint 2 and joint 3 including between inside the C channel
# This is purposely 0 for now. 
massArm2 = 0

# Mass of the end effector
massEndEffector = 1.95



# 
# Gravity Compensation
#
# Calculates and provides a voltage to counteract the influence of gravity on 2 joints
#
# PARAM: motorAngles -> Only the angle of the first and second joint are used.
# PARAM: additionalMassOnEndEffector -> Any extra mass that is on the end effector and should be compensated for
# PARAM: newtonMetersToVoltage -> Newton meters of torque converted to voltage, 
#       also can control how much influence gravity compensation has over the system
def gravityCompensation(motorAngles: list[6], additionalMassOnEndEffector: float, newtonMetersToVoltage: float):
    result = [0, 0, 0, 0, 0, 0]

    # 
    # Turret not influenced by gravity
    #
    result[0] = 0

    # 
    # Setup for first and second joint
    # 

    # x coordinate of centroid 1
    xCentroid1 = lengthCentroid1 * cos(motorAngles[1])

    # x coordinate of the position vector to the second joint
    xJoint2 = lengthArm1 * cos(motorAngles[1])

    # Angle between horizontal (along x-axis) and the second arm
    angleJoint2Horizontal = motorAngles[1] + motorAngles[2] + 180

    # x coordinate of centroid 2
    xCentroid2 = xJoint2 + lengthCentroid2 * cos(angleJoint2Horizontal)

    # x coordinate of centroid 3
    xCentroid3 = xJoint2 + lengthCentroid3 * cos(angleJoint2Horizontal)

    # Total mass of end effector
    totalMassEndEffector = massEndEffector + additionalMassOnEndEffector


    # 
    # First joint
    # 
    
    # x coordinate of the summed centroid of all 3 parts
    xCentroidTotal = (((xCentroid1 * massArm1) + (xCentroid2 * massArm2) + (xCentroid3 * totalMassEndEffector)) 
                        / (massArm1 + massArm2 + totalMassEndEffector))

    # Force of gravity on joint 1
    forceGravity = (massArm1 + massArm2 + totalMassEndEffector) * gravityConstant

    # Final calculation for the influence of gravity on the first joint
    # Moment = F * distance * conversionToVoltage
    result[1] = forceGravity * xCentroidTotal * newtonMetersToVoltage


    #
    # Second joint
    #

    # Move the origin to the second joint
    xCentroid2 -= xCentroid1
    xCentroid3 -= xCentroid1

    # x coordinate of the summed centroid of all 3 parts
    xCentroidTotal = (((xCentroid2 * massArm2) + (xCentroid3 * totalMassEndEffector)) 
                        / (massArm2 + totalMassEndEffector))

    # Force of gravity on joint 1
    forceGravity = (massArm2 + totalMassEndEffector) * gravityConstant

    # Final calculation for the influence of gravity on the first joint
    # Moment = F * distance * conversionToVoltage
    result[2] = forceGravity * xCentroidTotal * newtonMetersToVoltage
    

    return result  # unit of voltages
