#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 14:35:05 2023

@author: sadikozangorgu
"""

import numpy as np
import math
import time


def Inverse_Kinematics(x_pos, y_pos, angle): 
    global theta1_guess_g, theta2_guess_g, theta3_guess_g
    
    a1 = 160
    a2 = 165.01
    a3 = 140
    a4 = 140
    a5 = 135.8
    a6 = 1 #335

    a1 = 165.01
    a2 = 140
    a3 = 275.8

    def F1(theta1, theta2, theta3, xe):
        return a1 * math.cos(theta1) + a2 * math.cos(theta1+ theta2) + a3 * math.cos(theta1+ theta2+ theta3) - xe

    def F2(theta1, theta2, theta3, ye):
        return a1 * math.sin(theta1) + a2 * math.sin(theta1+ theta2) + a3 * math.sin(theta1+ theta2+ theta3) - ye

    def F3(theta1, theta2, theta3, phi):
        return theta1 + theta2 +  theta3 - phi

    #def Newton_Rhapson(guess_theta1, guess_theta2):
       # continue

    def vector_x(theta1_n,theta2_n, theta3_n):
        mat =  [
                [theta1_n],
                [theta2_n],
                [theta3_n]
               ]
        #mat = np.matrix(mat)
        return mat

    def vector_F(theta1_n, theta2_n, theta3_n, xe, ye, phi):
        mat =      [
                   [F1(theta1_n, theta2_n, theta3_n, xe)],
                   [F2(theta1_n, theta2_n, theta3_n, ye)],
                   [F3(theta1_n, theta2_n, theta3_n, phi)]
                   ]
        #mat = np.matrix(mat)
        return mat

    def jacobian_mat(theta1_n, theta2_n, theta3_n):
        mat = [
            [-a2 * math.sin(theta1_n) - a2*math.sin(theta1_n + theta2_n) - a3 * math.sin(theta1_n + theta2_n+ theta3_n), -a2 * math.sin(theta1_n + theta2_n) - a3 * math.sin(theta1_n + theta2_n+ theta3_n), -a3 * math.sin(theta1_n + theta2_n+ theta3_n) ],
            [ a1 * math.cos(theta1_n) + a2*math.cos(theta1_n + theta2_n) + a3 * math.cos(theta1_n + theta2_n+ theta3_n),  a2 * math.cos(theta1_n + theta2_n) + a3 * math.cos(theta1_n + theta2_n+ theta3_n),  a3 * math.cos(theta1_n + theta2_n+ theta3_n)],
            [1,1,1]
        ]
        #mat = np.matrix(mat)
        mat = np.linalg.inv(mat)
        return mat

    theta1_guess_g = 4
    theta2_guess_g = 2
    theta3_guess_g = 23

    theta1_guess_g = np.deg2rad(theta1_guess_g)
    theta2_guess_g = np.deg2rad(theta2_guess_g)
    theta3_guess_g = np.deg2rad(theta3_guess_g)

    max_interation = 80
    vector_x_n = vector_x(theta1_guess_g, theta2_guess_g, theta3_guess_g)

    def IK_2_arm(xe,ye, theta1_guess, theta2_guess, theta3_guess, angle):
        global theta1_guess_g, theta2_guess_g, theta3_guess_g
        vector_x_n1 = vector_x(theta1_guess, theta2_guess, theta3_guess)
        for i in range(max_interation):
            vector_x_n1 = vector_x_n1 - np.dot(jacobian_mat(theta1_guess, theta2_guess, theta3_guess), vector_F(theta1_guess, theta2_guess, theta3_guess, xe, ye, angle) )
            theta1_guess = vector_x_n1[0][0]
            theta2_guess = vector_x_n1[1][0]
            theta3_guess = vector_x_n1[2][0]
            theta1_guess_g = theta1_guess
            theta2_guess_g = theta2_guess
            theta3_guess_g = theta3_guess

    xe = x_pos
    ye = y_pos

    ye = ye - 160
    phi = angle

    IK_2_arm(xe,ye, theta1_guess_g, theta2_guess_g, theta3_guess_g, phi)

    theta1_guess_g = theta1_guess_g % (2* math.pi)
    theta2_guess_g = theta2_guess_g % (2* math.pi)
    theta3_guess_g = theta3_guess_g % (2* math.pi)
    
    if theta1_guess_g > math.pi:
        theta1_guess_g = theta1_guess_g - 2* math.pi
    if theta2_guess_g > math.pi:
        theta2_guess_g = theta2_guess_g - 2* math.pi
    if theta3_guess_g > math.pi:
        theta3_guess_g = theta3_guess_g - 2* math.pi
            
    return [theta1_guess_g, theta2_guess_g, theta3_guess_g]
