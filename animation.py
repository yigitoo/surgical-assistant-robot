import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import test

# This needs to be changed for your code

# This is the final example I showed in the code - notice I have 2 "cursor marks" not shown in the video
fig = plt.figure()
l, = plt.plot([], [], 'k-')
p1, = plt.plot([], [], 'ko')

plt.xlabel('xlabel')
plt.ylabel('ylabel')
plt.title('title')

plt.xlim(0, 300)
plt.ylim(0, 200)

def func(x):
    return np.sin(x)/2
    
lengths=[165.01,140,275.8]

testdata = [
   [10, 10, 10],
   [10, 20, 30],
   [30, 40, 20]
   ]
lst = []
for i in testdata:
    a = test.main(a1=165.01, a2=140, a3=275.8,t1=i[0],t2=i[1],t3=i[2])
    lst.append(a)
print(lst)
metadata = dict(title='.', artist='.')
writer = PillowWriter(fps=15, metadata=metadata)
f = open('lst.json','w+')
f.write(str(lst))

xlist = []
ylist = []

with writer.saving(fig, "result.gif", 100):

    for i in lst:
        for xval, yval in zip(i["xvals"],i["yvals"]):
            xlist.append(xval)
            ylist.append(yval)

        l.set_data(xlist,ylist)
        p1.set_data(xval,func(xval))

        writer.grab_frame()
    print(f"X: {xlist}\nY: {ylist}")
    for x, y in zip(xlist,ylist):
        print(x,y)
 