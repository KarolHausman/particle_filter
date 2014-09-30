import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams['text.usetex']=True
mpl.rcParams['text.latex.unicode']=True

# example data
x = np.arange(0, 8, 1)
y = np.exp(-x)

kld = [0.96098, 0.799508, 0.9565, 0.6863, 0.4334, 0.23076, 0.1332184, 0.0568]
kld_err = [0.039092608, 0.0983427339, 0.0332644705, 0.1558388751, 0.0986271514, 0.0554197889, 0.0513893695, 0.0422892421]


entr = [0.98866, 0.92154, 0.95548, 0.92014, 0.87406, 0.80958, 0.73134, 0.67754]
entr_err = [0.0298626523, 0.0909966922, 0.0621077048, 0.0484818316, 0.0849954881, 0.1320568514, 0.1789143315, 0.21536984]

ran = [0.96168, 0.81646, 0.87878, 0.90936, 0.89154, 0.88142, 0.81532, 0.72678]
ran_err = [0.0443342644, 0.1390013777, 0.1290776394, 0.0629977619, 0.0285690042, 0.1077853515, 0.1763482124, 0.2504651812]



# First illustrate basic pyplot interface, using defaults where possible.
plt.figure()
plt.errorbar(x, kld, yerr=kld_err, fmt='-o', capthick=2, ls="solid", label = "KLD")
plt.errorbar(x, entr, yerr=entr_err, fmt='-o', capthick=2, ls="dotted", label = "Min. Entropy")
plt.errorbar(x, ran, yerr=ran_err, fmt='-o', capthick=2, ls="dashed", label = "Random")
plt.xlim(-0.5,7.5)
plt.ylabel(r'Entropy')
plt.xlabel(r'Push number')
plt.legend(loc='lower left', numpoints=1)

#plt.title("Simplest errorbars, 0.2 in x, 0.4 in y")

plt.show()

#plt.savefig('entropy_cabinet.svg')
