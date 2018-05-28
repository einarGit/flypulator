import numpy as np
import matplotlib.pyplot as plt


data1 = np.genfromtxt('rotor_data.csv', delimiter=',', skip_header=1,names=['time','rotor1','vel1', 'f_x_1', 'f_y_1','f_z_1','tau_x_1','tau_y_1','tau_z_1',\
                                                                              'rotor2','vel2', 'f_x_2', 'f_y_2','f_z_2','tau_x_2','tau_y_2','tau_z_2',\
                                                                              'rotor3','vel3', 'f_x_3', 'f_y_3','f_z_3','tau_x_3','tau_y_3','tau_z_3',\
                                                                              'rotor4','vel4', 'f_x_4', 'f_y_4','f_z_4','tau_x_4','tau_y_4','tau_z_4',\
                                                                              'rotor5','vel5', 'f_x_5', 'f_y_5','f_z_5','tau_x_5','tau_y_5','tau_z_5',\
                                                                              'rotor6','vel6', 'f_x_6', 'f_y_6','f_z_6','tau_x_6','tau_y_6','tau_z_6'])

# data1['vel1'] = data1['vel1']
# data1['vel2'] = data1['vel2']
# data1['vel3'] = data1['vel3']
# data1['vel4'] = data1['vel4']
# data1['vel5'] = data1['vel5']
# data1['vel6'] = data1['vel6']

data1['vel1'] = data1['vel1']
data1['vel2'] = data1['vel2']
data1['vel3'] = data1['vel3']
data1['vel4'] = data1['vel4']
data1['vel5'] = data1['vel5']
data1['vel6'] = data1['vel6']


# data1['vel1'] = 1.0/data1['vel1']
# data1['vel2'] = 1.0/data1['vel2']
# data1['vel3'] = 1.0/data1['vel3']
# data1['vel4'] = 1.0/data1['vel4']
# data1['vel5'] = 1.0/data1['vel5']
# data1['vel6'] = 1.0/data1['vel6']

# data1['vel1'] = 1.0/data1['vel1']**2
# data1['vel2'] = 1.0/data1['vel2']**2
# data1['vel3'] = 1.0/data1['vel3']**2
# data1['vel4'] = 1.0/data1['vel4']**2
# data1['vel5'] = 1.0/data1['vel5']**2
# data1['vel6'] = 1.0/data1['vel6']**2

# show path
plt.figure(1)
plt.scatter(data1['vel1']**2,data1['f_z_1'], marker='x', label='motor1')
plt.scatter(data1['vel2']**2,data1['f_z_2'], marker='+', label='motor2')
plt.scatter(data1['vel3']**2,data1['f_z_3'], marker='x', label='motor3')
plt.scatter(data1['vel4']**2,data1['f_z_4'], marker='+', label='motor4')
plt.scatter(data1['vel5']**2,data1['f_z_5'], marker='x', label='motor5')
plt.scatter(data1['vel6']**2,data1['f_z_6'], marker='+', label='motor6')
plt.xlabel('vel_sq')
plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('force')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(2)
plt.scatter(data1['vel1']**2,data1['tau_z_1'], marker='x', label='motor1')
plt.scatter(data1['vel2']**2,data1['tau_z_2'], marker='+', label='motor2')
plt.scatter(data1['vel3']**2,data1['tau_z_3'], marker='x', label='motor3')
plt.scatter(data1['vel4']**2,data1['tau_z_4'], marker='+', label='motor4')
plt.scatter(data1['vel5']**2,data1['tau_z_5'], marker='x', label='motor5')
plt.scatter(data1['vel6']**2,data1['tau_z_6'], marker='+', label='motor6')
plt.xlabel('vel_sq')
plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('torque')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(3)
for i in range(0, 5):
    plt.plot(data1['time']**2,data1['f_z_'+str(i+1)], label='f_z_'+str(i+1))
plt.xlabel('time')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('f_z')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(4)
for i in range(0, 5):
    plt.plot(data1['time']**2,data1['f_x_'+str(i+1)], label='f_x_'+str(i+1))
plt.xlabel('time')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('f_x')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(5)
for i in range(0, 5):
    plt.plot(data1['time']**2,data1['f_y_'+str(i+1)], label='f_y_'+str(i+1))
plt.xlabel('time')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('f_y')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(6)
for i in range(0, 5):
    plt.plot(data1['time']**2,data1['tau_x_'+str(i+1)], label='tau_x_'+str(i+1))
plt.xlabel('time')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('tau_x')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(7)
for i in range(0, 5):
    plt.plot(data1['time']**2,data1['tau_y_'+str(i+1)], label='tau_y_'+str(i+1))
plt.xlabel('time')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('tau_y')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.figure(8)
for i in range(0, 5):
    plt.plot(data1['time']**2,data1['tau_z_'+str(i+1)], label='tau_z_'+str(i+1))
plt.xlabel('time')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('tau_z')
plt.grid()
plt.legend(loc='best', shadow=False)

plt.show()


