import matplotlib.pyplot as plt
from matplotlib.dates import date2num
import datetime
import numpy as np
import matplotlib as mpl

spread = 4.0
barWidth = 0.4

mpl.rcParams['text.usetex']=True
mpl.rcParams['text.latex.unicode']=True

x = np.array([1.0, 1.0 + spread])
#rot = [49, 51.2, 77]
#prism=[41,48.8,23]
#free=[10,0,0]



rot = [1.6, 0.4]
prism=[94.9,0.0]
free=[3.5,99.6]

err_rot = [0.65192024, 0.54772256]
err_prism = [0.65192024, 0.0]
err_free = [0.0,0.54772256]

#particles = [49,  41, 10, 0]
#particles = (0, 51.2, 48.8, 0)
#particles = (0, 23, 77, 0)

fig = plt.figure(figsize=(5.5, 2.5), dpi=200)
barWidth = 0.4
ax = plt.subplot(111)
ax.bar(x-barWidth, rot,width=barWidth, color=[0,0,1],align='center')
ax.bar(x, prism,width=barWidth, color=[1,0,1],align='center')
ax.bar(x+barWidth,free, width=barWidth, color=[0,1,0],align='center')

plt.xticks([])
plt.ylabel(r'Fraction of particles [\%]')
plt.ylim(0,100)

#plt.show()

plt.savefig('erasers_prob.svg')
