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
import time
import json, requests, functools, tkinter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import webview


initial_pos = (100,160)

app = Flask(__name__, static_folder='../public', template_folder='../templates')
jdata = json.loads(open('param.json','r').read())
with open('param.json','w+') as f:
    json.dump(jdata, f, indent=4)
''' GO server'dan dönenleri al '''

@app.get('/get_sensor_values')
def get_all_sensor_values():

    r = requests.post('https://localhost:5632/', json={"cmd_name": "get_motor_by_id","cmd_val": "1"})

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

@app.get('/commander/')
def commander():
    return render_template('commander.html')

@app.get('/inc_degree/<int:motor_id>')
def inc_degree(motor_id):

    cmd_val = inc_cmd_list_creator(motor_id, False)

    requests.post('http://localhost:5632/', json={
        "cmd_name": "inc_angle",
        "cmd_val": cmd_val,
    })

    return jsonify({
        "messaage": "success"
    })
@app.get('/dec_degree/<int:motor_id>')
def dec_degree(motor_id: int):
    
    cmd_val = inc_cmd_list_creator(motor_id, True)

    requests.post('http://localhost:5632/', json={
        "cmd_name": "inc_angle",
        "cmd_val": cmd_val,
    })
    return jsonify({
        "messaage": "success"
    })

def inc_cmd_list_creator(act_id: int, is_neg: bool) -> str:
    result = ""
    l_result = []
    for i in range(6):
        if i != (act_id - 1):
            l_result.append(0)
        else:
            if is_neg:
                l_result.append(-5)
            else:
                l_result.append(5)

    for index in range(len(l_result) - 1):
        result += str(l_result[index]) + ','

    result += str(l_result[-1])

    return result

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
        with open('param.json','w+') as f:
            json.dump(jdata, f, indent=4)

    root = tkinter.Tk()
    root.bind('<KeyPress>', onKeyPress)
    root.mainloop()

def title_changer(window):
    from time import sleep
    while True:
        window.set_title('SURGICAL ASSISTANT ROBOT GUI')
        sleep(3)
        window.set_title('Written by Surgical Assistan Robot Interns\'2023')
        sleep(3)

class animator():

    def __init__(self):
        plt.ion()
        plt.title(r'$\Sigma(x) = \gamma x^2 \sin(\theta)$', pad = 20)
        plt.ylim([0, 800])
        plt.xlim([0, 800])
        plt.axis([0, 800, 0, 800])
        self.x = np.linspace(0, 10*np.pi, 100)
        self.y = np.sin(self.x)

        self.line1, = plt.plot(self.x, self.y, 'b-')
        
        plt.margins(2,2) # Default margin is 0.05, value

    def main(self, a1,a2,a3,t1,t2,t3):
        global plt
        def calc(m1, t1):
            x = np.cos(np.radians(t1))*m1
            y = np.sin(np.radians(t1))*m1
            return x,y

        initial_pos = (0,160)

        vals1 = calc(a1,(t1))
        vals2 = calc(a2,(t1+t2))
        vals3 = calc(a3,(t1+t2+t3))
        #vals4 = calc(a4,t4)
        #vals5 = calc(a5,t5)

    
        x0 = 0
        y0 = 160
        
        x1 = a1 * np.cos(t1)
        y1 = y0 + a1*np.sin(t1)
        
        x2 = x1 + a2 * np.cos(t1+t2 )
        y2 = y1 + a2 * np.sin(t1+t2)
        
        x3 = x2 + a3*np.cos(t1+t2+t3)
        y3 = y2 + a3*np.sin(t1+t2+t3)
        
        print("x3:", x1, "   y3: ", y1)
        
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("ROBOT_MOVING")
            
            
        plt.plot(0,0, color = 'red', linestyle = "--", linewidth = 4, marker = 'o', markersize=4)
        plt.plot(x0,y0, color = 'red', linestyle = "--", linewidth = 4, marker = 'o', markersize=4)
        plt.plot(x1,y1, color = 'red', linestyle = "--", linewidth = 4, marker = 'o', markersize=4)
        plt.plot(x2,y2, color = 'red', linestyle = "--", linewidth = 4, marker = 'o', markersize=4)
        plt.plot(x3,y3, color = 'red', linestyle = "--", linewidth = 4, marker = 'o', markersize=4)
            
        '''
        plt.plot(0+vals1[0]+vals2[0]+vals3[0],160+vals1[1]+vals2[1]+vals3[1],vals1[0]+vals2[0]+vals3[0]+vals4[0],
                160+vals1[1]+vals2[1]+vals3[1]+vals4[1],marker = 'o')

        plt.plot(0+vals1[0]+vals2[0]+vals3[0]+vals4[0],160+vals1[1]+vals2[1]+vals3[1]+vals4[1],vals1[0]+vals2[0]+vals3[0]+vals4[0]+vals5[0],
                160+vals1[1]+vals2[1]+vals3[1]+vals4[1]+vals5[1],marker = 'o')
        '''

        return {"xvals":[vals1[0],vals2[0],vals3[0]],"yvals":[vals1[1],vals2[1],vals3[1]]}

        '''
        return (0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
                160+vals1[1]+vals2[1],
                0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
                160+vals1[1]+vals2[1],
                0+vals1[0]+vals2[0],160+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
                160+vals1[1]+vals2[1]+vals3[1]),
        '''

    def main2(self, a1,a2,a3,t1,t2,t3):
        
        global plt
        
        def calc(m1, t1):
            x = np.cos(np.radians(t1))*m1
            y = np.sin(np.radians(t1))*m1
            return x,y


        vals1 = calc(a1,t1)
        vals2 = calc(a2,t2)
        vals3 = calc(a3,t3)
        #vals4 = calc(a4,t4)
        #vals5 = calc(a5,t5)

        plt.plot(initial_pos[0],self.inital_pos[1],initial_pos[0]+vals1[0],initial_pos[1]+vals1[1],marker = 'o')

        plt.plot(initial_pos[0]+vals1[0],initial_pos[1]+vals1[1],vals1[0]+vals2[0],
                initial_pos[1]+vals1[1]+vals2[1],marker = 'o')

        plt.plot(initial_pos[0]+vals1[0],initial_pos[1]+vals1[1],vals1[0]+vals2[0],
                initial_pos[1]+vals1[1]+vals2[1],marker = 'o')

        plt.plot(initial_pos[0]+vals1[0]+vals2[0],initial_pos[1]+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
                initial_pos[1]+vals1[1]+vals2[1]+vals3[1],marker = 'o')
        '''    
        plt.plot(0+vals1[0]+vals2[0]+vals3[0],160+vals1[1]+vals2[1]+vals3[1],vals1[0]+vals2[0]+vals3[0]+vals4[0],
                160+vals1[1]+vals2[1]+vals3[1]+vals4[1],marker = 'o')

        plt.plot(0+vals1[0]+vals2[0]+vals3[0]+vals4[0],160+vals1[1]+vals2[1]+vals3[1]+vals4[1],vals1[0]+vals2[0]+vals3[0]+vals4[0]+vals5[0],
                160+vals1[1]+vals2[1]+vals3[1]+vals4[1]+vals5[1],marker = 'o')
        '''
        

        return {"xvals":[vals1[0],vals2[0],vals3[0]],"yvals":[vals1[1],vals2[1],vals3[1]]}
        '''
        return (0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
                160+vals1[1]+vals2[1],
                0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
                160+vals1[1]+vals2[1],
                0+vals1[0]+vals2[0],160+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
                160+vals1[1]+vals2[1]+vals3[1]),
        '''


def use_matrix():
    import csv
    import pandas as pd

    df = pd.read_csv('kosag.csv')
    
    d1 = df['EKSEN1']
    d2 = df['EKSEN2']
    d3 = df['EKSEN3']
    d4 = df['EKSEN4']
    d5 = df['EKSEN5']
    
    for i in range(0, len(d1)):
        r = requests.post('http://localhost:5632/', json={"cmd_name": "set_angle","cmd_val": f"{d1[i]},{d2[i]},{d3[i]},{d4[i]},{d5[i]},0"})
        if i == 0:
            time.sleep(3)

if __name__ == "__main__":    
    import threading 
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == "-a":
            import time
            animating = animator()
            for i in range(100):
                animating.main(a1=165.01, a2=140, a3=275.8+335,t1=10,t2=20,t3=i)
                plt.xlim(0,800)
                plt.ylim(0,800)
                plt.show()
                plt.pause(0.1)
                plt.clf()
        if sys.argv[1] == "-t":
            app.run(host='0.0.0.0', port=1453, debug=True)
        if sys.argv[1] == "-w":
            window = webview.create_window('SURGICAL ASSISTANT ROBOT GUI', app)
            webview.start(title_changer, window)
        if sys.argv[1] == "-m":
            use_matrix()
    else:
        window = webview.create_window('SURGICAL ASSISTANT ROBOT GUI', app,
            width=800, height=800, resizable=True,
        )
        webview.start(title_changer, window) #'debug=True')
