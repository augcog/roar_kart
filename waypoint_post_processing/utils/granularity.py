import pandas as pd
import sys
import os
import geopy.distance
import statistics as st
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print('need a csv file as an argument')
    sys.exit(1)

file_path = sys.argv[1]
df = pd.read_csv(file_path, delimiter=',')
points = []
for i in range(len(df['lat'][:])):
    points.append([df['lat'][i], df['lon'][i]])

n = len(points)

dists = [0] * n
for i in range(n):
    dists[i] = geopy.distance.geodesic(points[i], points[(i+1) % n]).m

last = dists[-1]
dists = dists[:-1]
out = "Max Distance: " + str(max(dists)) + "\n"
out += "Min Distance: " + str(min(dists)) + "\n"
out += "Mean Distance: " + str(st.mean(dists)) + "\n"
out += "Standard Deviation: " + str(st.stdev(dists)) + "\n"
out += "Beginning/End Gap Distance: " + str(last) + "\n"

print(out)
plt.scatter(range(n-1), dists)
plt.show()
# f = open(file_path[:-3] + "txt", "a")
# f.write(out)
# print(out)
# f.close()
