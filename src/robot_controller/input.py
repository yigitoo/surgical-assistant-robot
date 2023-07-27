import numpy as np
import matplotlib.pyplot as plt
from servo import DynamixelServo
import time
import pypot.dynamixel
import json
current_values = [0,0,0,0]
#dxl_io = pypot.dynamixel.Dxl320IO('/dev/ttyUSB0', baudrate=1000000)

def setangles(m1, m2, m3, m4):
    dxl_io.set_goal_position({1: m1, 2: m2, 3: m3, 4: m4})
    return True

angles = json.loads(open('/home/salih/gits/surgical-assistant-robot/src/robot_controller/servo_angles.json').read())

d = DynamixelServo()
def davinci(id, val):
    current_values[id-1] = current_values[id-1] + val
    d.set_angles(current_values)

def forward():
    global xv
    xv+=1

def back():
    global xv
    xv-=1

def right():
    zv+=1

def left():
    zv-=1

def down():
    global yv
    yv-=1

def up():
    global yv
    yv+=1

def press(event):
    print('press', event.key)
    if event.key == 'up':
        davinci(3,6)
        davinci(4,6)
    if event.key == 'down':
        davinci(3,-6)
        davinci(4,-6)
    if event.key == "right":
        davinci(3,-6)
        davinci(4,6)
    if event.key == "left":
        davinci(3,6)
        davinci(4,-6)
    if event.key == "shift+up":
        davinci(1,6)
    if event.key == "shift+down":
        davinci(1,-6)
    if event.key == "shift+right":
        davinci(2,6)
    if event.key == "shift+left":
        davinci(2,-6)
    if event.key == "space":
        up()
    if event.key == "shift":
        down()
    if event.key == "d":
        right()
    if event.key == "a":
        left()
    if event.key == "w":
        forward()
    if event.key == "s":
        back()
    

plt.rcParams['keymap.save'] = ["ctrl+s"]
fig, ax = plt.subplots()
fig.canvas.mpl_connect('key_press_event', press)
#plt.show()
#setangles(1.61, 87.24, 150.0, -1.03)
print(d.set_angles(angles["close"]))
time.sleep(2)
print(d.set_angles(angles["open"]))
print(d.read_angles())
