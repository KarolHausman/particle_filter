import pylab 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot1D():

  datalist = pylab.loadtxt("/home/hak2pal/.ros/kernel_test.txt")
  pylab.plot(datalist[:,0],datalist[:,1])
  pylab.show()

def plot2D():
  datalist = pylab.loadtxt("/home/hak2pal/.ros/kernel_test2D.txt")
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

plot2D()  
