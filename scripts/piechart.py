

from pylab import *
import matplotlib as mpl
mpl.rcParams['font.size'] = 27.0

# make a square figure and axes
figure(1, figsize=(4,4))
#ax = axes([0.1, 0.1, 0.8, 0.8])

# The slices will be ordered and plotted counter-clockwise.
#labels = 'Frogs', 'Hogs', 'Dogs', 'Logs'



#entrpy
#particles = [37.6, 42.2, 20.2]
#colors =[[0,0,1],[1,0,1],[0,1,0]]
#particles = [46.6, 53.4]
particles = [53.2,46.8]
colors =[[0,0,1],[1,0,1]]

#kld
#particles = [49,  41, 10]
#colors =[[0,0,1],[1,0,1],[0,1,0]]
#particles = [51.2, 48.8]
#particles = [77, 23]
#colors =[[0,0,1],[1,0,1]]
#explode=(0, 0.05, 0, 0)


patches, texts, autotexts = plt.pie(particles, colors = colors,  autopct='%1.1f%%')
                # The default startangle is 0, which would start
                # the Frogs slice on the x-axis.  With startangle=90,
                # everything is rotated counter-clockwise by 90 degrees,
                # so the plotting starts on the positive y-axis.

#texts[0].set_fontsize(40)

#title('Raining Hogs and Dogs', bbox={'facecolor':'0.8', 'pad':5})

#plt.show()
plt.savefig('pie3.png')