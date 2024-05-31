import numpy as np
import matplotlib.pyplot as plt

# read the x,y,z position.csv file
data = np.genfromtxt('../position.csv', delimiter=',', skip_header=1)
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]


#make a histogram fo the z positions
plt.hist(z, bins=100)
plt.xlabel('z position')
plt.ylabel('frequency')
plt.title('z position histogram')
plt.show()

#make a scatter plot of the x and y positions
plt.scatter(x, y)
plt.xlabel('x position')
plt.ylabel('y position')
plt.title('x-y position scatter plot')
plt.show()

#3d scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
ax.set_xlabel('x position')
ax.set_ylabel('y position')
ax.set_zlabel('z position')
plt.title('3d position scatter plot')
plt.show()