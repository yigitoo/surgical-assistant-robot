import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

plt.ion()
fig = plt.figure()
x = np.linspace(0, 10*np.pi, 100)
y = np.sin(x)
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')

plt.xlim(0, 600)
plt.ylim(0, 600)

def main(a1,a2,a3,t1,t2,t3):
    global plt
    def calc(m1, t1):
        x = np.cos(np.radians(t1))*m1
        y = np.sin(np.radians(t1))*m1
        return x,y

    initial_pos = (0,160)

    vals1 = calc(a1,t1)
    vals2 = calc(a2,t2)
    vals3 = calc(a3,t3)
    #vals4 = calc(a4,t4)
    #vals5 = calc(a5,t5)

    plt.plot(0,160,0+vals1[0],160+vals1[1],marker = 'o')

    plt.plot(0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
            160+vals1[1]+vals2[1],marker = 'o')

    plt.plot(0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
            160+vals1[1]+vals2[1],marker = 'o')

    plt.plot(0+vals1[0]+vals2[0],160+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
            160+vals1[1]+vals2[1]+vals3[1],marker = 'o')
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

def main2(a1,a2,a3,t1,t2,t3):
    
    global plt
    
    def calc(m1, t1):
        x = np.cos(np.radians(t1))*m1
        y = np.sin(np.radians(t1))*m1
        return x,y

    initial_pos = (0,160)

    vals1 = calc(a1,t1)
    vals2 = calc(a2,t2)
    vals3 = calc(a3,t3)
    #vals4 = calc(a4,t4)
    #vals5 = calc(a5,t5)

    plt.plot(0,160,0+vals1[0],160+vals1[1],marker = 'o')

    plt.plot(0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
            160+vals1[1]+vals2[1],marker = 'o')

    plt.plot(0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
            160+vals1[1]+vals2[1],marker = 'o')

    plt.plot(0+vals1[0]+vals2[0],160+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
            160+vals1[1]+vals2[1]+vals3[1],marker = 'o')
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

if __name__=="__main__":
    for i in range(10):
        main(a1=165.01, a2=140, a3=275.8,t1=10,t2=20,t3=i)
        plt.show()
        time.sleep(1)
        for phase in np.linspace(0, 10*np.pi, 100):
            line1.set_ydata(np.sin(0.5 * x + phase))
            fig.canvas.draw()
            fig.canvas.flush_events()

        plt.clf()
        fig.canvas.draw()