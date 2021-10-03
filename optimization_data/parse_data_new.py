import scipy.io
import numpy as np

data = scipy.io.loadmat("jumping_A1Robot_SHINE_d100.mat")

data = {k:v for k, v in data.items() if k[0] != '_'}

parameter = data.keys()

for i in parameter:
	if(i == 'taus'):
		np.savetxt(("data_tau.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'Qs'):
		np.savetxt(("data_Q.csv".format(i)), data.get(i), delimiter=",")
