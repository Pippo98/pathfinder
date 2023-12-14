import matplotlib.pyplot as plt
import numpy as np

def get_data(f):
    data = f.readlines()
    f.close()
    x = []
    y = []
    for line in data[1:]:
        line = line.strip().split(',')
        x.append(float(line[0]))
        y.append(float(line[1]))
    return x, y

x, y = get_data(open('path.csv', 'r'))
t_x, t_y = get_data(open('tree.csv', 'r'))

plt.plot(t_x, t_y, '.g')
plt.plot(x, y, 'r')
plt.axis('equal')
plt.show()