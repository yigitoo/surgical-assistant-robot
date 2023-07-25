import numpy as np
import matplotlib.pyplot as plt
from servo import DynamixelServo

def davinci_open():
    DynamixelServo(3).set_angles(DynamixelServo.read_angles[2]+6)
    DynamixelServo(4).set_angles(DynamixelServo.read_angles[3]-6)


def davinci_close():
    DynamixelServo(3).set_angles(DynamixelServo.read_angles[2]-6)
    DynamixelServo(4).set_angles(DynamixelServo.read_angles[3]+6)

def davinci_right():    
    DynamixelServo(3).set_angles(DynamixelServo.read_angles[2]+6)
    DynamixelServo(4).set_angles(DynamixelServo.read_angles[3]+6)


def davinci_left():    
    DynamixelServo(3).set_angles(DynamixelServo.read_angles[2]-6)
    DynamixelServo(4).set_angles(DynamixelServo.read_angles[3]-6)


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
    if event.key == 'enter':
        cnt.append(1)
    if event.key == 'up':
        davinci_open()
    if event.key == 'down':
        davinci_close()
    if event.key == "right":
        davinci_right()
    if event.key == "left":
        davinci_left()
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
    
cnt=[]
plt.rcParams['keymap.save'] = ["ctrl+s"]
fig, ax = plt.subplots()
fig.canvas.mpl_connect('key_press_event', press)
ax.plot(np.random.rand(12), np.random.rand(12), 'go')
plt.show()
