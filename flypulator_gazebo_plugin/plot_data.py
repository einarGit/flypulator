import numpy as np
import matplotlib.pyplot as plt


data1 = np.genfromtxt('result.csv', delimiter=',',names=['time','vel1','vel2', 'vel3', 'vel4','vel5','vel6','f1','f2','f3','f4','f5','f6'])

# data1['vel1'] = data1['vel1']
# data1['vel2'] = data1['vel2']
# data1['vel3'] = data1['vel3']
# data1['vel4'] = data1['vel4']
# data1['vel5'] = data1['vel5']
# data1['vel6'] = data1['vel6']

data1['vel1'] = data1['vel1']**2
data1['vel2'] = data1['vel2']**2
data1['vel3'] = data1['vel3']**2
data1['vel4'] = data1['vel4']**2
data1['vel5'] = data1['vel5']**2
data1['vel6'] = data1['vel6']**2


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
plt.scatter(data1['vel1'],data1['f1'], marker='x', label='motor1')
plt.scatter(data1['vel2'],data1['f2'], marker='+', label='motor2')
plt.scatter(data1['vel3'],data1['f3'], marker='x', label='motor3')
plt.scatter(data1['vel4'],data1['f4'], marker='+', label='motor4')
plt.scatter(data1['vel5'],data1['f5'], marker='x', label='motor5')
plt.scatter(data1['vel6'],data1['f6'], marker='+', label='motor6')
plt.xlabel('vel_sq')
plt.ticklabel_format(axis='x', style='sci', scilimits=(-2,2))
# plt.ylabel('force')
plt.ylabel('torque')  
plt.grid()
plt.legend(loc='best', shadow=False)

plt.show()
