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
import json, requests, functools, webview, tkinter


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

tracemod = -1

jargs = list(jdata.keys())
jargs += [jargs.pop(1)]

''' kontrol döngüsü '''

def arr2string(ar):
    s=""
    for i in range(len(ar)-1):
        s+=str(ar[i])+","
    s+=ar[-1]
    return s
def applydata(variables, li):
    jdata[variables][0] = li[0]
    jdata[variables][1] = li[1]
    jdata[variables][2] = li[2]
    jdata[variables][3] = li[3]
    jdata[variables][4] = li[4]
    jdata[variables][5] = li[5]

def update(variables="angles", l=[0,0,0,0,0,0], xv=0, xy=0):

    applydata(variables="angles",li=l)
    if tracemod == 1:
        r = requests.post('http://localhost:5632/', json={"cmd_name": "set_angles","cmd_val": f"{arr2string(jdata['angles'])}"})
        data = f"""
            {str(jdata[jargs[0]]) + str(jdata[jargs[0]])}
            {str(jdata[jargs[1]]) + str(jdata[jargs[1]])}
            {str(jdata[jargs[2]]) + str(jdata[jargs[2]])}
            {str(jdata[jargs[3]]) + str(jdata[jargs[3]])}
            {str(jdata[jargs[4]]) + str(jdata[jargs[4]])}
            """
    else:
        r = requests.post('http://localhost:5632/', json={"cmd_name": "get_all","cmd_val":""})
        s = r.text.split(":")
        #jdata[jargs[2]] = s[0].split(',')
        #jdata[jargs[1]] = s[1].split(',')
        #jdata[jargs[2]] = s[2].split(';')[0] + s[2].split(';')[1] 
        cartesian = s[3].split(',')
        jdata[jargs[4]] = {
            "x": cartesian[0],
            "y": cartesian[1],
            "z": cartesian[2],
            "thetaX": cartesian[3],
            "thetaY": cartesian[4],
            "thetaZ": cartesian[5],
        }
        jdata[jargs[3]] = s[0].split(';')
        for i in jargs[1:len(jargs)-1]:
            applydata(variables=i,li=s[map[str(jargs.index(i))]])    
        print(data)

xv=0.0
yv=0.0

''' gui '''

@functools.lru_cache
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
    root.mainloop()

def title_changer(window):
    from time import sleep
    for i in range(1, 100):
        window.set_title('SURGICAL ASSISTANT ROBOT GUI')
        sleep(3)
        window.set_title('Written by bengi')
        sleep(3)


if __name__ == "__main__":    
    import threading 
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == "-t":
            start_gui()       
        elif sys.argv[1] == "-w":
            window = webview.create_window('SURGICAL ASSISTANT ROBOT GUI', app)
            webview.start(title_changer, window)
    else:
        window = webview.create_window('SURGICAL ASSISTANT ROBOT GUI', app,
            width=800, height=800, resizable=True,
        )
        webview.start(title_changer, window) #'debug=True')


    