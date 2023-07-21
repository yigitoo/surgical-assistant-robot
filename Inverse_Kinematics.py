#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 19 19:13:32 2023

@author: sadikozangorgu
"""

import numpy as np
import time
import math

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
        #print(vector_x_n1)
        theta1_guess = vector_x_n1[0][0]
        theta2_guess = vector_x_n1[1][0]
        theta3_guess = vector_x_n1[2][0]
        theta1_guess_g = theta1_guess
        theta2_guess_g = theta2_guess
        theta3_guess_g = theta3_guess
        '''print(theta1_guess)
        print(theta2_guess)
        print(theta3_guess)
        print("------------------")'''
        #print()
        #time.sleep(2)



xe = 300
ye = 500

ye = ye - 160
phi = math.pi/3

IK_2_arm(xe,ye, theta1_guess_g, theta2_guess_g, theta3_guess_g, phi)


theta1_guess_g = theta1_guess_g % (2* math.pi)
theta2_guess_g = theta2_guess_g % (2* math.pi)
theta3_guess_g = theta3_guess_g % (2* math.pi)

print(theta1_guess_g)
print(theta2_guess_g)
print(theta3_guess_g)





'''
while -math.pi/4<theta2_guess_g and theta2_guess_g<math.pi:
    theta1_guess_g = theta1_guess_g % (2* math.pi)
    theta2_guess_g = theta2_guess_g % (2* math.pi)
    theta3_guess_g = theta3_guess_g % (2* math.pi)
    IK_2_arm(xe, ye, theta1_guess_g, theta2_guess_g, theta3_guess_g, angle)    
    
    print(theta1_guess_g)
    print(theta2_guess_g)
    print(theta3_guess_g)
'''
#print (F1(theta1_guess_g, theta2_guess_g, xe) + xe)
#print (F2(theta1_guess_g, theta2_guess_g, ye) + ye)



# %%


R = 68
x_c = 340
y = 190
z_h = 200

z_h = z_h - 160

x_min = x_c - R
x_max = x_c + R


x_list = list(np.linspace(x_min, x_max, 200))
#print(x_list)

theta1_guess_g = 4
theta2_guess_g = 2
theta3_guess_g = 23
phi = math.pi/4


value_list = list()

for i in range(len(x_list)):
    xe = x_list[i]
    IK_2_arm(xe,z_h, theta1_guess_g, theta2_guess_g, theta3_guess_g, phi)
    theta1_guess_g = theta1_guess_g % (2* math.pi)
    theta2_guess_g = theta2_guess_g % (2* math.pi)
    theta3_guess_g = theta3_guess_g % (2* math.pi)
    temp = [theta1_guess_g, theta2_guess_g,theta3_guess_g]
    value_list.append(temp)
    print(temp)


x_min
x_max
for i in range(20):
    t1 = value_list[i][0]
    t12 = t1 + value_list[i][1] 
    t123 = t12 + value_list[i][2] 
    a1 = 165.01
    a1_xy = [a1 * math.cos(t1), a1 * math.sin(t1)+ 160]
    a2_xy = [a2 * math.cos(t12) + a1_xy[0], a2 * math.sin(t12) + a1_xy[1]]
    a3_xy = [a3 * math.cos(t123) + a2_xy[0], a3 * math.sin(t123) + a2_xy[1]]
    
    print(a3_xy)




'''
import matplotlib.pyplot as plt
import seaborn as sns

import matplotlib.animation as ani
from sklearn.linear_model import LinearRegression

animator = ani.FuncAnimation(fig, func, interval = 100)

x_i = 200 
x_f = -250
y_i = 500

x_list = list(np.linspace(x_i, x_f, num = 2000))
print(x_list)
x_list[1999]



for i in range(2000):
    IK_2_arm(xe,ye, theta1_guess_g, theta2_guess_g, theta3_guess_g, phi)



fig = plt.figure(figsize = (12,5))
ax1 = plt.subplot(1,2,2)
ax2 = plt.subplot(1,2,2)
'''










#%%

# Finding the Homogeneous Transformation Matrix Using the Denavit Hartenberg Method



a1 = 160
a2 = 165.01
a3 = 140
a4 = 140
a5 = 135.8
a6 = 1 


theta1 = 0
theta2 = theta1_guess_g
theta3 = theta2_guess_g
theta4 = 0
theta5 = theta3_guess_g


'''
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0
'''

table = [[theta1,              math.pi/2,   0,       a1],
         [theta2,              0,           a2,      0],
         [(math.pi/2+theta3),  math.pi/2,   0,       0],
         [(math.pi+theta4),    math.pi/2,   0,       (a3+a4)],
         [(math.pi/2+theta5),  0,           (a5+a6), 0]]


def Denavit_Hartenberg_Mat(row):
    a0_0 =  -math.sin(table[row-1][0]) * math.cos(table[row-1][1])
    a0_1 =   math.sin(table[row-1][0]) * math.sin(table[row-1][1])
    a0_2 =   table[row-1][2] * math.cos(table[row-1][0])

    a1_0 =   math.cos(table[row-1][0]) * math.cos(table[row-1][1])
    a1_1 =  -math.cos(table[row-1][0]) * math.sin(table[row-1][1])
    a1_2 =   table[row-1][2] * math.sin(table[row-1][0])
    
    mat = [[math.cos(table[row-1][0]), a0_0,  a0_1, a0_2],
           [math.sin(table[row-1][0]), a1_0,  a1_1, a1_2],
           [0, math.sin(table[row-1][1]), math.cos(table[row-1][1]), table[row-1][3]],
           [0,0,0,1]
           ]
    mat = np.matrix(mat)
    return mat

DH0_1 = Denavit_Hartenberg_Mat(1).round(decimals = 8)
DH1_2 = Denavit_Hartenberg_Mat(2).round(decimals = 8)
DH2_3 = Denavit_Hartenberg_Mat(3).round(decimals = 8)
DH3_4 = Denavit_Hartenberg_Mat(4).round(decimals = 8)
DH4_5 = Denavit_Hartenberg_Mat(5).round(decimals = 8)


DH0_2 = np.dot(DH0_1, DH1_2)
DH2_4 = np.dot(DH2_3, DH3_4)
DH0_5 = np.dot(np.dot(DH0_2, DH2_4 ), DH4_5)

print()
print(DH0_5)


