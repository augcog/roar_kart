import geopy.distance
import matplotlib
from pyproj import Transformer
import math
import numpy as np
import matplotlib.pyplot as plt
import geopandas as gpd
import pandas as pd
import csv
import sys

if len(sys.argv) < 2:
    print('need a csv file as an argument')
    sys.exit(1)
t = Transformer.from_crs(4326, 6500).transform
f = Transformer.from_crs(6500, 4326).transform


def to_xyz(p):
    lat, lon = p
    x, y = t(lat, lon)
    return x, y


def to_latlon(p):
    x, y = p
    lat, lon = f(x, y)
    return lat, lon


def d(p1, p2):
    return geopy.distance.geodesic(p1, p2).m


def closest(p, track):
    return gradient_descent(i % min(n, m), lambda x: d(p, track[x % len(track)]), 20000, 100)


def discrete_derivative(f, x):
    return f(x+1) - f(x)


def gradient_descent(start, f, learn_rate, max_iter, tol=1e-8):
    steps = [start]  # history tracking
    x, min_x = [start]*2

    min_f = float('inf')
    for _ in range(max_iter):
        diff = learn_rate*discrete_derivative(f, x)
        if np.abs(diff) < tol:
            break
        x = int(x - diff)
        if f(x) < min_f:
            min_f = f(x)
            min_x = x
        steps.append(x)  # history tracing

    return min_f


file_path = sys.argv[1]
df = pd.read_csv(file_path, delimiter=',')
# csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
# csv_writer.writeheader()
data, removed_points = [], 0
seen = set()
i = 0
while i < len(df['lat'][:]):
    p = [df['lat'][i], df['lon'][i]]
    if (p[0], p[1]) not in seen:
        seen.add((p[0], p[1]))
        data.append(p)
        removed_points += 1
    i += 1
print('removed points: ' + str(removed_points))
xy_data_p = [to_xyz(d) for d in data]
xy_data = np.array(xy_data_p)
c = [sum(xy_data[:, 0]) / len(xy_data[:, 0]),
     sum(xy_data[:, 1]) / len(xy_data[:, 1])]
xy_data = np.array([[p[0] - c[0], p[1] - c[1]] for p in xy_data_p])
plt.axis('equal')
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.axis('equal')
sc, = ax.plot(xy_data[:, 0], xy_data[:, 1], c='blue', marker='o', ls='')
fig.show()
fig.canvas.draw()


while True:
    p_click = plt.ginput()[0]
    p = to_latlon([p_click[0]+c[0], p_click[1]+c[1]])
    min_dis, min_i = float('inf'), -1
    for i in range(len(data)):
        dis = d(p, data[i])
        if min_dis > dis:
            min_dis = dis
            min_i = i
    print(min_i)
    xy_data = np.vstack((xy_data[:min_i, :], xy_data[min_i + 1:, :]))
    data.pop(min_i)
    csvfile = open(file_path, 'w', newline='')
    fieldnames = ['lat', 'lon']
    with open(file_path, 'w', newline='') as csvfile:
        csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        csv_writer.writeheader()
        for i in range(len(data)):
            lat, lon = data[i]
            csv_writer.writerow({'lat': lat, 'lon': lon})
    sc.set_data(xy_data[:, 0], xy_data[:, 1])
    fig.canvas.draw()

# data[
# Reading csv


# Writing to csv

# csv_writer.writerow({'lat': lat, 'lon': lon})

# print('total # of waypoints after : ' + str(len(data)))
