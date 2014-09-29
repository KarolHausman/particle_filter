import matplotlib.pyplot as plt
from matplotlib.dates import date2num
import datetime
import numpy as np
import matplotlib as mpl

spread = 4.0
barWidth = 0.15

#mpl.rcParams['text.usetex']=True
#mpl.rcParams['text.latex.unicode']=True

x = np.array([1.0])#, 1.0 + spread, 1.0 + spread*2])

rot = [49]#, 51.2, 77]
prism=[41]#,48.8,23]
free=[10]#,0,0]
rigid=[0]

#rot = [49, 51.2, 77]
#prism=[41,48.8,23]
#free=[10,0,0]
#rigid=[0]

#rot = [37.6, 46.6, 53.2]
#prism=[42.2,53.4,46.8]
#free=[20.2,0,0]


#particles = [49,  41, 10, 0]
#particles = (0, 51.2, 48.8, 0)
#particles = (0, 23, 77, 0)

fig = plt.figure()#figsize=(4.5, 2.5), dpi=200)
barWidth = 0.4
ax = plt.subplot(111)
ax.bar(x-barWidth, rot,width=barWidth,color=[0,0,1],align='center',label="Rotational")
ax.bar(x, prism,width=barWidth,color=[1,0,1],align='center', label="Prismatic")
ax.bar(x+barWidth,free, width=barWidth,color=[0,1,0],align='center',label="Free-body")
ax.bar(x+barWidth*2,rigid, width=barWidth,color=[1,0,0],align='center',label="Rigid")

ax.legend(loc='upper right', numpoints=1)

plt.xticks([])
plt.ylabel(r'Fraction of particles [\%]')
plt.ylim(0,100)

#plt.show()
plt.savefig('before.png')
#plt.savefig('articulation_all.svg')
