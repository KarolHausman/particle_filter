import pylab 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

def plot1D(txt_file):

  datalist = pylab.loadtxt(txt_file)
  pylab.plot(datalist[:,0],datalist[:,1])
  pylab.show()

def plot2D(txt_file):
  datalist = pylab.loadtxt(txt_file)
  fig = plt.figure()
  ax = plt.axes(projection='3d')
  s1 = np.array(sorted(set(datalist[:,0])))
  s2 = np.array(sorted(set(datalist[:,1])))
  x, y = np.meshgrid(s1, s2)
  Z = datalist[:,2].reshape(len(s1), len(s2))
    
  #x = np.outer(np.linspace(-2, 2, 30), np.ones(30))
  #y = x.copy().T
  #x, y = np.meshgrid(datalist[:,0],datalist[:,1])
  p = ax.plot_surface(x,y,Z, rstride=2, cstride=2, cmap=plt.cm.jet, linewidth=0) 
  #p = ax.plot_surface(datalist[:,0],datalist[:,1], datalist[:,2], rstride=1, cstride=1, cmap=plt.cm.jet, linewidth=0) 
  #z = np.cos(x ** 2 + y ** 2)  
  #ax.plot_surface(x, y, z, cmap=plt.cm.jet, rstride=1, cstride=1, linewidth=0)

  plt.show()

if __name__ == '__main__':
  plot_dim = 0  
  txt_file =""
  if (len(sys.argv) >= 2):
    plot_dim = int(sys.argv[1])
    txt_file = sys.argv[2]
  else:
    print "\nAborting! Wrong number of command line args"
    print "Usage: python plot_kernel.py <number of dimensions 1 or 2> <file>"
    sys.exit(0) 
  if plot_dim == 1:
    plot1D(txt_file)
  elif plot_dim == 2: 
    plot2D(txt_file)  
  else:
    print "\n Wrong number of dimensions, it needs to be 1 or 2"
    sys.exit(0)
