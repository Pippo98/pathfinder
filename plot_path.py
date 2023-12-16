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

def get_obstacles(f):
    data = f.readlines()
    f.close()
    obstacles = []
    x = []
    y = []
    for line in data[1:]:
        if line == '\n':
            obstacles.append([x, y])
            x = []
            y = []
            continue
        line = line.strip().split(',')
        x.append(float(line[0]))
        y.append(float(line[1]))
    return obstacles


plt.figure(figsize=(10, 10))
x, y = get_data(open('path.csv', 'r'))
t_x, t_y = get_data(open('tree.csv', 'r'))
obstacles = get_obstacles(open('obstacles.csv', 'r'))

plt.plot(t_x, t_y, '.g')
plt.plot(x, y, 'r')

for obstacle in obstacles:
    plt.plot(obstacle[0], obstacle[1], 'k')

plt.axis('equal')
plt.show()