import numpy as np 
import math as m
import constants
import math



def Rx(theta):
  return np.array([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.array([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.array([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]]) 



# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=-1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


def computeDK(a, b, c, use_rads=True):
    return computeDKDetailed(a, b, c, use_rads)[-1]

def rotaton_2D(x, y, z, leg_angle):
    def rotation2D(angle):
        return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    xy = np.array([[x], [y]])
    xy = np.dot(rotation2D(leg_angle), xy)
    return [xy[0], xy[1], z]


def computeDKDetailed(theta1, theta2, theta3, use_rads=True):
    theta1 = theta1 * constants.THETA1_MOTOR_SIGN
    theta2 = theta2 * constants.THETA2_MOTOR_SIGN - constants.theta2Correction
    theta3 = theta3 * constants.THETA3_MOTOR_SIGN - constants.theta3Correction
   
    A1 = np.array([constants.constL1 * m.cos(theta1), constants.constL1 * m.sin(theta1), 0]).reshape((3, 1))
   
    A2 = np.dot(Rz(theta1), np.array([constants.constL2 * np.cos(-theta2), 0, constants.constL2 * np.sin(-theta2)]).reshape((3, 1))) + A1

    r0 = np.array([constants.constL3 * np.cos(np.pi - theta3), 0, constants.constL3 * np.sin(np.pi - theta3)]).reshape((3, 1))
    r1 = np.dot(Ry(theta2 + np.pi), r0)
    
    M = np.dot(Rz(theta1), r1) + A2

    
    O = list(np.array([0, 0, 0]))
    A1  = list(A1.reshape((3,)))
    A2  = list(A2.reshape((3,)))
    M   = list(M.reshape((3,)))

    
    return [O, A1, A2, M]



def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)
  
# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle
      


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes
# separated by the distances (l1, l2, l3), returns the angles to apply to the 3 axes

def computeIK(
    x,
    y,
    z,
    l1=constants.constL1,
    l2=constants.constL2,
    l3=constants.constL3,
    verbose=False,
    use_rads=constants.USE_RADS_OUTPUT,
    sign=-1,
    use_mm=constants.USE_MM_INPUT,
):

  dist_unit = 1
  if use_mm:
    dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit
  
    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
  if y == 0 and x == 0:
     # Taking care of this singularity (leg right on top of the first rotational axis)
    theta1 = 0
  else:
    theta1 = math.atan2(y, x)

      
      
    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - constants.Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
          angleRestrict(constants.THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
          angleRestrict(
            constants.THETA2_MOTOR_SIGN * (theta2 + constants.theta2Correction), use_rads=use_rads
          ),
          angleRestrict(
            constants.THETA3_MOTOR_SIGN * (theta3 + constants.theta3Correction), use_rads=use_rads
          ),
        ]
        
    else:
      result = [
        angleRestrict(constants.THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
        angleRestrict(
          constants.THETA2_MOTOR_SIGN * (math.degrees(theta2) + constants.theta2Correction),
          use_rads=use_rads,
        ),
        angleRestrict(
          constants.THETA3_MOTOR_SIGN * (math.degrees(theta3) + constants.theta3Correction),
          use_rads=use_rads,
        ),
      ]
    if verbose:
      print(
        "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
          x,
          y,
          z,
          result[0],
          result[1],
          result[2],
        )
      )

    print("result :", result)
    return result
  
  

def computeIKOriented(x, y, z, leg_id, params, verbose=True):

  target = np.array([x, y, z])  # le points cible dans repère du bout de la patte

  #le points cible dans le repère d'avant
  toret = np.dot(Rz(constants.LEG_ANGLES[leg_id-1]), target) + np.array(params.initLeg[leg_id-1] + [params.z])
   
  
  return computeIK(toret[0], toret[1] , toret[2])
  

  
  


  

   
if __name__ == "__main__":
    print("bonjour.\n")

    print(computeIKOriented(0, -90, -90, 1, ))


    

