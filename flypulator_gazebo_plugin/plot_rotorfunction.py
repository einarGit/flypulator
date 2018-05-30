import numpy as np
import matplotlib.pyplot as plt


data1 = np.genfromtxt('rotor_data.csv', delimiter=',', skip_header=1,names=['time','rotor1','vel1', 'f_x_1', 'f_y_1','f_z_1','tau_x_1','tau_y_1','tau_z_1',\
                                                                              'rotor2','vel2', 'f_x_2', 'f_y_2','f_z_2','tau_x_2','tau_y_2','tau_z_2',\
                                                                              'rotor3','vel3', 'f_x_3', 'f_y_3','f_z_3','tau_x_3','tau_y_3','tau_z_3',\
                                                                              'rotor4','vel4', 'f_x_4', 'f_y_4','f_z_4','tau_x_4','tau_y_4','tau_z_4',\
                                                                              'rotor5','vel5', 'f_x_5', 'f_y_5','f_z_5','tau_x_5','tau_y_5','tau_z_5',\
                                                                              'rotor6','vel6', 'f_x_6', 'f_y_6','f_z_6','tau_x_6','tau_y_6','tau_z_6'])

data2 = np.genfromtxt('rotor_data_v=1.csv', delimiter=',', skip_header=1,names=['time','rotor1','vel1', 'f_x_1', 'f_y_1','f_z_1','tau_x_1','tau_y_1','tau_z_1',\
                                                                              'rotor2','vel2', 'f_x_2', 'f_y_2','f_z_2','tau_x_2','tau_y_2','tau_z_2',\
                                                                              'rotor3','vel3', 'f_x_3', 'f_y_3','f_z_3','tau_x_3','tau_y_3','tau_z_3',\
                                                                              'rotor4','vel4', 'f_x_4', 'f_y_4','f_z_4','tau_x_4','tau_y_4','tau_z_4',\
                                                                              'rotor5','vel5', 'f_x_5', 'f_y_5','f_z_5','tau_x_5','tau_y_5','tau_z_5',\
                                                                              'rotor6','vel6', 'f_x_6', 'f_y_6','f_z_6','tau_x_6','tau_y_6','tau_z_6'])

data3 = np.genfromtxt('rotor_data_v=-1.csv', delimiter=',', skip_header=1,names=['time','rotor1','vel1', 'f_x_1', 'f_y_1','f_z_1','tau_x_1','tau_y_1','tau_z_1',\
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
plt.scatter(data1['vel1'],data1['f_z_1'], label='v=[0,0,0]', c='b', marker='x')
plt.scatter(data2['vel1'],data2['f_z_1'], label='v=[1,1,1]',c='r', marker='x')
plt.scatter(data3['vel1'],data3['f_z_1'], label='v=[-1,-1,-1]',c='g', marker='x')
plt.xlabel('vel')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('force')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.savefig('f_z.png')

plt.figure(2)
plt.scatter(data1['vel1'],data1['tau_z_1'], label='v=[0,0,0]',c='b', marker='x')
plt.scatter(data2['vel1'],data2['tau_z_1'], label='v=[1,1,1]',c='r', marker='x')
plt.scatter(data3['vel1'],data3['tau_z_1'], label='v=[-1,-1,-1]',c='g', marker='x')
plt.xlabel('vel')
plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('torque')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.savefig('tau_z.png')

plt.figure(3)
plt.scatter(data1['vel1'],data1['f_x_1'], label='v=[0,0,0]',c='b', marker='x')
plt.scatter(data2['vel1'],data2['f_x_1'], label='v=[1,1,1]',c='r', marker='x')
plt.scatter(data3['vel1'],data3['f_x_1'], label='v=[-1,-1,-1]',c='g', marker='x')
plt.xlabel('vel')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('f_x')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.savefig('f_x.png')

plt.figure(4)
plt.scatter(data1['vel1'],data1['f_y_1'], label='v=[0,0,0]',c='b', marker='x')
plt.scatter(data2['vel1'],data2['f_y_1'], label='v=[1,1,1]',c='r', marker='x')
plt.scatter(data3['vel1'],data3['f_y_1'], label='v=[-1,-1,-1]',c='g', marker='x')
plt.xlabel('vel')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('f_y')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.savefig('f_y.png')

plt.figure(5)

plt.scatter(data1['vel1'],data1['tau_x_1'], label='v=[0,0,0]',c='b', marker='x')
plt.scatter(data2['vel1'],data2['tau_x_1'], label='v=[1,1,1]',c='r', marker='x')
plt.scatter(data3['vel1'],data3['tau_x_1'], label='v=[-1,-1,-1]',c='g', marker='x')
plt.xlabel('vel')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('tau_x')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.savefig('tau_x.png')

plt.figure(6)
plt.scatter(data1['vel1'],data1['tau_y_1'], label='v=[0,0,0]',c='b', marker='x')
plt.scatter(data2['vel1'],data2['tau_y_1'], label='v=[1,1,1]',c='r', marker='x')
plt.scatter(data3['vel1'],data3['tau_y_1'], label='v=[-1,-1,-1]',c='g', marker='x')
plt.xlabel('vel')
#plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
plt.ylabel('tau_y')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.savefig('tau_y.png')

plt.show()


