"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt
from pylab import *


n_groups = 4

particles = [49,  41, 10, 0]
#particles = (0, 51.2, 48.8, 0)
#particles = (0, 23, 77, 0)
colors =[[0,0,1],[1,0,1],[0,1,0],[1,0,0]]

#fig, ax = plt.subplots()

fig = plt.figure(figsize=(2.5, 5), dpi=200)


index = np.arange(n_groups)
bar_width = 0.1

opacity = 0.9
error_config = {'ecolor': '0.1'}


print (index+1)*bar_width/2
rects1 = plt.bar(index*bar_width, particles, bar_width,
                 color=colors,
                 alpha=opacity,
                 error_kw=error_config,
                 label='Models')

#plt.figure(num=None, figsize=(8, 6), dpi=80, facecolor='w', edgecolor='k')
#fig.set_size_inches(5,1)
plt.xlim(0,0.3)
plt.ylim(0,100)

plt.xticks([])
plt.yticks([])
#plt.xlabel('Articulation Model')
#plt.ylabel('Percentage of particles [%]')
#plt.title('Articulation Models Statistics')
#plt.xticks(index + bar_width, ('Free', 'Prismatic', 'Revolute', 'Rigid'))
#plt.legend()
#print plt.margins(0.7,1)
#fig = plt.figure(figsize=(18, 18))
#rcParams['figure.figsize'] = 5, 1

#plt.tight_layout()
plt.show()

#plt.savefig('articulation3.png')
