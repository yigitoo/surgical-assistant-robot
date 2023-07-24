import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time
from main import animator
plt.ion()

# def main(a1,a2,a3,t1,t2,t3):
#     global plt
#     def calc(m1, t1):
#         x = np.cos(np.radians(t1))*m1
#         y = np.sin(np.radians(t1))*m1
#         return x,y

#     initial_pos = (0,160)

#     vals1 = calc(a1,t1)
#     vals2 = calc(a2,t2)
#     vals3 = calc(a3,t3)
#     #vals4 = calc(a4,t4)
#     #vals5 = calc(a5,t5)

    

#     plt.plot(0,160,0+vals1[0],160+vals1[1], marker='o')

#     plt.plot(0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
#         160+vals1[1]+vals2[1],marker = 'o')

#     plt.plot(0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
#         160+vals1[1]+vals2[1],marker = 'o')

#     plt.plot(0+vals1[0]+vals2[0],160+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
#         160+vals1[1]+vals2[1]+vals3[1],marker = 'o',color='r')
#     '''    
#     plt.plot(0+vals1[0]+vals2[0]+vals3[0],160+vals1[1]+vals2[1]+vals3[1],vals1[0]+vals2[0]+vals3[0]+vals4[0],
#             160+vals1[1]+vals2[1]+vals3[1]+vals4[1],marker = 'o')

#     plt.plot(0+vals1[0]+vals2[0]+vals3[0]+vals4[0],160+vals1[1]+vals2[1]+vals3[1]+vals4[1],vals1[0]+vals2[0]+vals3[0]+vals4[0]+vals5[0],
#             160+vals1[1]+vals2[1]+vals3[1]+vals4[1]+vals5[1],marker = 'o')
#     '''
#     return {"xvals":[vals1[0],vals2[0],vals3[0]],"yvals":[vals1[1],vals2[1],vals3[1]]}
#     '''
#     return (0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
#             160+vals1[1]+vals2[1],
#             0+vals1[0],160+vals1[1],vals1[0]+vals2[0],
#             160+vals1[1]+vals2[1],
#             0+vals1[0]+vals2[0],160+vals1[1]+vals2[1],vals1[0]+vals2[0]+vals3[0],
#             160+vals1[1]+vals2[1]+vals3[1]),
#     '''

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

import pickle
with open('data_new','rb') as f:
    dataset = pickle.load(f)
dataset = np.array(dataset)
print(dataset)
for i in dataset:
    print(len(i))

if __name__=="__main__":
    anim = animator()
    for i,j,k in zip(dataset[0],dataset[1],dataset[2]):
        print(i,j,k)
        anim.main(a1=165.01, a2=140, a3=275.8+335,t1=i,t2=j,t3=k)
        plt.xlim(-1000,1000)
        plt.ylim(-1000,1000)
        plt.show()
        plt.pause(0.1)
        plt.clf()
