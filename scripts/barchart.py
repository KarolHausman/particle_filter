"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt


n_groups = 4

#particles = (10, 49, 41, 0)
#particles = (0, 51.2, 48.8, 0)
particles = (0, 23, 77, 0)

fig, ax = plt.subplots()

index = np.arange(n_groups)
bar_width = 0.25

opacity = 0.4
error_config = {'ecolor': '0.1'}

rects1 = plt.bar(index+bar_width/2, particles, bar_width,
                 alpha=opacity,
                 color='b',
                 error_kw=error_config,
                 label='Models')


plt.ylim(0,100)
plt.xlabel('Articulation Model')
plt.ylabel('Percentage of particles [%]')
plt.title('Articulation Models Statistics')
plt.xticks(index + bar_width, ('Free', 'Prismatic', 'Revolute', 'Rigid'))
#plt.legend()

plt.tight_layout()
#plt.show()

plt.savefig('articulation3.png')
