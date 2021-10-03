import matplotlib.pyplot as plt
import numpy as np

def plot_data(file_name, title):
    FootForce= np.genfromtxt(file_name, delimiter=' ')
    f_0 = FootForce[:,][:,0]
    f_1 = FootForce[:,][:,1]
    f_2 = FootForce[:,][:,2]
    f_3 = FootForce[:,][:,3]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()
    

plot_data('Hipposition.txt', 'Hip_Position')
plot_data('Hipvelocity.txt', 'Hip_Velocity')
plot_data('Hiptorque.txt', 'Hip_Torque')
plot_data('Thighposition.txt', 'Thigh_Position')
plot_data('Thighvelocity.txt', 'Thigh_Velocity')
plot_data('Thightorque.txt', 'Thigh_Torque')
plot_data('Calfposition.txt', 'Calf_Position')
plot_data('Calfvelocity.txt', 'Calf_Velocity')
plot_data('Calftorque.txt', 'Calf_Torque')

plt.show()