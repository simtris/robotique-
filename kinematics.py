 # -- coding: utf-8 --

import numpy as np
from math import *
import matplotlib.pyplot as pyplo

# -----------------------------------------------------------------------

l1 = 0.045
l2 = 0.065
l3 = 0.087

def rotationX(teta):
    return [[1,0,0],[0,cos(teta),-sin(teta)],[0,sin(teta),cos(teta)]]

def rotationY(teta):
    return [[cos(teta),0,sin(teta)],[0,1,0],[-sin(teta),0,cos(teta)]]

def rotationZ(teta):
    return [[cos(teta),-sin(teta),0],[sin(teta),cos(teta),0],[0,0,1]]

def matrixMul(X,Y):
    result =  [[0 for i in range(len(Y[0]))] for j in range(len(X))]
    for i in range(len(X)):
        for j in range(len(Y[0])):
            for k in range(len(Y)):
                result[i][j] += X[i][k] * Y[k][j]
    return result

def matrixPlus(X,Y):
    result =  [[0 for i in range(len(X[0]))] for j in range(len(X))]
    for i in range(len(X)):
        for j in range(len(X[0])):
            result[i][j] = X[i][j] + Y[i][j]
    return result

def alKashi(a,b,c):
    return acos((a**2 + b**2 - c**2)/(2*a*b))

def direct(a, b, c):
    coord1 = [[l1], [0], [0]]
    coord2 = [[l2], [0], [0]]
    coord3 = [[l3], [0], [0]]
    result = matrixPlus(matrixMul(rotationY(c), coord3), coord2)
    result = matrixPlus(matrixMul(rotationY(-b), result), coord1)
    result = matrixMul(rotationZ(a), result)
    return [result[0][0], result[1][0], result[2][0]]

def inverse(x, y, z):
    alpha = atan2(y,x)

    xa = cos(alpha) * l1
    ya = sin(alpha) * l1
    za = 0
    AM = sqrt((x - xa)**2 + (y - ya)**2 + (z - za)**2)
    gamma = pi - alKashi(l2,l3,AM)

    beta = alKashi(l2,AM,l3) + asin(z/AM)

    return [alpha,beta,gamma]

#--------------------------------------------------------------------------------------------------

def computeDK(theta1, theta2, theta3):
    theta1 = THETA1_MOTOR_SIGN * theta1
    theta2 = THETA2_MOTOR_SIGN - theta2Correction
    theta3 = THETA3_MOTOR_SIGN - theta3Correction
    #todo

'''
x,y + angle theta --> point qui est rotation de (x,y) avec theta
'''
def rotaton_2D:
    #todo

if __name__ == "__main__":
    printf("bonjour.\n")