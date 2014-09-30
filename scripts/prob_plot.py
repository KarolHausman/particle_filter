import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams['text.usetex']=True
mpl.rcParams['text.latex.unicode']=True

# example data
x = np.arange(0, 8, 1)
y = np.exp(-x)

kld = [0.173, 0.25, 0.551, 0.742, 0.9088, 0.962, 0.981,0.993]
kld_err = [0.0340220517, 0.0615426681, 0.1237638881, 0.2243490584, 0.030938649, 0.0120415946, 0.0089442719, 0.0057008771]


entr = [0.2, 0.367, 0.481, 0.569, 0.633, 0.689, 0.737, 0.773]
entr_err = [0.0365718471, 0.1025060974, 0.135342159, 0.1664857351, 0.1738749551, 0.1803260935, 0.1896575862, 0.1779255462]


ran = [0.172, 0.275, 0.356, 0.482, 0.546, 0.657, 0.711, 0.753]
ran_err = [0.0385032466, 0.1055935604, 0.1540048701, 0.1941841909, 0.207557221, 0.1343316791, 0.1342758355, 0.1580585335]



# First illustrate basic pyplot interface, using defaults where possible.
plt.figure()
plt.errorbar(x, kld, yerr=kld_err, fmt='-o', capthick=2,ls="solid", label = "KLD")
plt.errorbar(x, entr, yerr=entr_err, fmt='-o', capthick=2,ls="dotted", label = "Min. Entropy")
plt.errorbar(x, ran, yerr=ran_err, fmt='-o', capthick=2, ls="dashed", label = "Random")
plt.xlim(-0.5,7.5)
plt.ylim(0,1.05)
plt.ylabel(r'Probability of Rotational Model')
plt.xlabel(r'Push number')
plt.legend(loc='upper left', numpoints=1)

#plt.title("Simplest errorbars, 0.2 in x, 0.4 in y")

#plt.show()

plt.savefig('prob_cabinet.svg')
