"""
{
		angles: [0, 0, 0, 0, 0, 0]
		cartesian: {
			x: 0,
			y: 0,
			z: 0,
			thetaX: 0,
			thetaY: 0,
			thetaZ: 0,
		},
		temperatures: [0, 0, 0, 0, 0, 0],
		currents: [0, 0, 0, 0, 0, 0],
		torques: [0, 0, 0, 0, 0, 0]
	}
"""                                                          
from flask import (
    Flask,
    render_template,
    request,
    jsonify,
    redirect,
)
import json
import os
import requests


app = Flask(__name__, static_folder='public', template_folder='templates')
jdata = json.loads(open('param.json','r').read())

''' GO server'dan dönenleri al '''

@app.get('/get_sensor_values')
def get_all_sensor_values():

    r = requests.post('https://f220-178-237-49-127.ngrok-free.app/', json={"cmd_name": "get_motor_by_id","cmd_val": "1"})

    rdata = r.text
    return jsonify(rdata)
''' f/i kinematics'''
#jdata["angles"] = json.loads(rdata)

@app.get('/')
def index():
    return render_template('index.html')

@app.get('/api')
def api_doc():
    return render_template('api.html')

@app.get('/commander')
def commander():
    return render_template('commander.html')

import numpy as np

def ik(X,Y): #Inverse Kinematics Function
    T2 = np.arctan2(Y,X)

    R0_6 = [[-1.0,0.0,0.0],
            [0.0,-1.0,0.0],
            [0.0,0.0,1.0]]

    R0_3 = [[-np.sin(T2),0.0,np.cos(T2)],
            [np.cos(T2),0.0,np.sin(T2)],
            [0.0,1.0,0.0]]

    invR0_3=np.linalg.inv(R0_3)

    R3_6=np.dot(invR0_3, R0_6)

    T5 = np.arccos(R3_6[2][2])
    T6 = np.arccos(R3_6[2][0]/-np.sin(T5))
    T4 = np.arccos(R3_6[1][2]/np.sin(T5))
    return T4,T5,T6    


''' kontrol döngüsü '''

def update(m1=0, m2=0, m3=0, m4=0, m5=0, m6=0, xv=0, xy=0):
   
    jdata["angles"][0] = m1
    jdata["angles"][1] = m2
    jdata["angles"][2] = m3
    jdata["angles"][3] = m4
    jdata["angles"][4] = m5
    jdata["angles"][5] = m6
    jdata["cartesian"]["x"] = xv
    jdata["cartesian"]["y"] = yv

xv=0.0
yv=0.0

''' gui '''
from functools import lru_cache

@lru_cache
def start_gui():

    def forward():
        global xv
        xv+=0.1

    def back():
        global xv
        xv-=0.1

    def right():
        pass

    def left():
        pass

    def down():
        global yv
        yv-=0.1

    def up():
        global yv
        yv+=0.1
        


    import tkinter
    def onKeyPress(event):
        global xv, yv
        if event.char == "w":
            forward()
        if event.char == "s":
            back()
        if event.char == "d":
            right()
        if event.char == "a":
            left()
        if event.char == "q":
            up()
        if event.char == "e":
            down()
        l = np.degrees(list(ik(xv,yv)))
        update(m4=l[0], m5=l[1], m6=l[2])
        print(yv,xv)
        print(l, jdata)
        open('param.json','w+').write(json.dumps(jdata))

    root = tkinter.Tk()
    root.bind('<KeyPress>', onKeyPress)
    rB = tkinter.Button(root, text ="￪", command = up, width=15, height=6,font=("Arial",15))
    rB.pack(side="top")
    rB = tkinter.Button(root, text ="￬", command = down, width=15, height=6,font=("Arial",15))
    rB.pack(side="bottom")

    fB = tkinter.Button(root, text ="▲", command = forward, width=20, height=5,font=("Arial",15))
    fB.place(height=100, width=100,)
    fB.pack(side="top")
    bB = tkinter.Button(root, text ="▼", command = back, width=20, height=5,font=("Arial",15))
    bB.pack(side="bottom")
    #bB.place(height=100, width=100,)
    rB = tkinter.Button(root, text ="▶", command = right, width=20, height=5,font=("Arial",15))
    rB.pack(side="right")
    #rB.place(height=100, width=100,)
    lB = tkinter.Button(root, text ="◀", command = left, width=20, height=5,font=("Arial",15))
    lB.pack(side="left")
    #lB.place(height=100, width=100,)

    root.mainloop()

import sys

if len(sys.argv)==2:
    start_gui()

if __name__ == "__main__":    
    import webview
    window = webview.create_window('Webview', app)
    webview.start()