import scipy.io
import numpy as np

data = scipy.io.loadmat("jumping-A1Robot-SHINE-d100.mat")
data = {k:v for k, v in data.items() if k[0] != '_'}

parameter = data.keys()

for i in parameter:
	np.savetxt(("data_{}.csv".format(i)), data.get(i), delimiter=",")
