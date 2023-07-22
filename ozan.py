import matplotlib.pyplot as plt
import numpy as np
import pickle
from matplotlib.animation import FuncAnimation
with open('data_new','rb') as f:
    dataset = pickle.load(f)
dataset = np.array(dataset)
print(dataset)

a1 = 165.01
a2 = 140
a3 = 610.8


for k in range(len(dataset[0])):

    #print(np.rad2deg(angles[2][k]), " : ", k)
    plt.xlim(0,1000)
    plt.ylim(0,1000)
        
    t1 = dataset[0][k]
    t2 = dataset[1][k]
    t3 = dataset[2][k]
    
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
        
    plt.grid()
    plt.pause(0.000001)

ani = FuncAnimation(plt.gcf(), update, interval = 200)
plt.show()