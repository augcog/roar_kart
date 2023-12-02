import geopy.distance
import matplotlib
from pyproj import Transformer
import math
import numpy as np
import numpy as np
import random
import pandas as pd
import sys
import matplotlib.pyplot as plt
import os
import geopy.distance
import statistics as st

t = Transformer.from_crs(4326, 6500).transform
f = Transformer.from_crs(6500, 4326).transform


def to_xyz(p):
    lat, lon = p
    x, y = t(lat, lon)
    return [x, y]


def to_latlon(p):
    x, y = p
    lat, lon = f(x, y)
    return lat, lon


def closest(p, track):
    return gradient_descent(i % min(n, m),
                            lambda x: d(p, track[x % len(track)]), 20000, 100)


def d(p1, p2):
    return geopy.distance.geodesic(p1, p2).m


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

    return min_x, min_f


if len(sys.argv) < 3:
    print('need two csv files as an argument')
    sys.exit(1)

file_path1 = sys.argv[1]
file_path2 = sys.argv[2]
df1 = pd.read_csv(file_path1, delimiter=',')
df2 = pd.read_csv(file_path2, delimiter=',')

points1, points2 = [], []
for i in range(len(df1['lat'][:])):
    points1.append([df1['lat'][i], df1['lon'][i]])
for i in range(len(df2['lat'][:])):
    points2.append([df2['lat'][i], df2['lon'][i]])


n, m = len(points1), len(points2)
DEBUG_POINT = 4288  # 1122  # 4288  # , 1122
dists = [0] * max(n, m)
bigger_array = points1 if n >= m else points2
smaller_array = points1 if n < m else points2
xy_data_p = [to_xyz(d) for d in points1 + points2]
xy_data = np.array(xy_data_p)
c = [sum(xy_data[:, 0]) / len(xy_data[:, 0]),
     sum(xy_data[:, 1]) / len(xy_data[:, 1])]
xy_data = np.array([[p[0] - c[0], p[1] - c[1]] for p in xy_data_p])

q = 0
min_d, min_p = float('inf'), -1
max_d, max_p = float('-inf'), -1
def to_cart(x): return [to_xyz(x)[0] - c[0], to_xyz(x)[1] - c[1]]


for i in range(max(n, m)):
    if q < 100*(i/max(n, m)):
        print(str(q) + "% complete")
        q += 1

    def dist_(x): return d(bigger_array[i], smaller_array[x % min(n, m)]),
    dists[i] = min([dist_(j) for j in range(min(n, m))])

    # def cart_dist(x): return (bigger_array[i][0] - smaller_array[x % min(n,m)])
    # min_dist=float('inf')
    # for j in range(len(smaller_array)):
    #    dists[i]=dist_(j)[0]
    #    if min_dist > dists[i]:
    #        min_dist=dists[i]
    #        min_j=j
    # dists[i], min_j = gradient_descent(i % min(n, m), lambda x: d(
    #    bigger_array[i], smaller_array[x % min(n, m)]), 20000, 100)
    # if min_d > dists[i]:
    #    min_d = dists[i]
    #    min_p = [to_cart(bigger_array[i]), to_cart(
    #        smaller_array[min_j % min(n, m)])]
    # if max_d < dists[i]:
    #    max_d = dists[i]
    #    max_p = [to_cart(bigger_array[i]), to_cart(
    #        smaller_array[min_j % min(n, m)]), i, min_j]

    # min_p = np.array(min_p)
    # print(max_p)
    # max_p = np.array(max_p[:2])
    #
plt.scatter(xy_data[:, 0], xy_data[:, 1], c='blue', ls='')
plt.scatter(min_p[:, 0], min_p[:, 1], c='orange', ls='', s=300)
plt.scatter(max_p[:, 0], max_p[:, 1], c='green', ls='', s=300)
plt.axis('equal')
plt.show()
out = "Max Distance: " + str(max(dists)) + "\n"
out += "Min Distance: " + str(min(dists)) + "\n"
out += "Mean Distance: " + str(st.mean(dists)) + "\n"
out += "Standard Deviation: " + str(st.stdev(dists)) + "\n"

print(out)

# DEBUGGING GRADIENT DESCENT
# i = DEBUG_POINT
# dists = [0] * m
# for j in range(m):
#    dists[j] = d(bigger_array[i], smaller_array[j % min(n, m)])
# plt.plot(range(m), dists)
# calc = gradient_descent(i % min(n, m), lambda x: d(
#    bigger_array[i], smaller_array[x % min(n, m)]), 20000, 100)
# plt.scatter(calc[0], calc[1])
# plt.show()

if len(sys.argv) > 3:
    f = open(file_path1[: -3] + "_vs_" + file_path2[: -3] + "txt", "a")
    f.write(out)
    f.close()
