# ROAR Waypoint Post Processing

This repo contains two post processing scripts for the waypoint 
following project.

## Bag To CSV

This is a ROS node that connects to the /gps/fix topic and outputs
a csv file, named data.csv, in the current directory with latitude
and longitude data.

An optional argument could be added to change the name of the output
file.

### Usage

Make sure the ros bag you want to collect data from is running:

`ros2 bag play [ros_bag_name]`

Then call

`python3 bag_to_csv.py`

and let the script run until the bag is finished. Afterwards,
you should find the data.csv file in your current directory.



## Utils

This is a directory includes four scripts. A Map visualizer, a CSV cleaner,
and two statistics scripsts.


### Map Visualizer

This is python script that takes in a csv's file with latitude and
longitude data and visualizes it with google maps.

#### Usage

In order to use this script, you will need a Google Maps API Key.
This can be set up [here](https://developers.google.com/maps/documentation/embed/get-api-key).

Afterwards, add this line to your ~/.bashrc

`export GOOGLE_API_KEY=<your_key>`

and either open a new terminal or run:

`source ~/.bashrc`

Then you should be able to run the script with:

`python3 map.py [path to csv files]`

Noting that multiple files can be passed in as argument.
The map should be opened in your default browser,
with a side bar that can be clicked to allow zoom
and pan features. 

The circle.csv file is included and can be ran as an example using:

`python3 map.py circle.csv`

### CSV Cleaner

The CSV Cleaner expects to be given a single csv file as argument.
It will display the track using matplotlib. After this, one can 
delete surpfluous points by left clicking on them. The index of the
point will be printed to the terminal.

`python3 clean_csv.py [path to csv file]`

### Distance between two tracks

dists_between.py expects to be given two csv files with latitude
and longitude coordinates will output maximum, minimum, mean,
and standard devation of distances between each coordinate.
The distance comparison happens between the two spatially closest
points on each track, since there's no guarantee that indexes are the same.
This is currently being done using gradient descent, but there's issues with
this distance function being nonconvex. Comparing against every point, however,
takes excessive amounts of time to run.

### Granularity

granularity.py expects to be given a single csv file with latitude and longitude
coordinates. It will output the same statistics as dists_between, but for the distance
between the current coordinate and the next coordinate.

#### Usage

These scripts require geopy

`pip install geopy`

Then for granularity.py call

`python3 granularity.py <path to csv file>`

and the statistics will be printed to the terminal.

for dists_between.py call

`python3 dists_between.py <path to csv file> <path to csv file>`

and the statistics will be printed to the terminal.

This script takes a long time to run, so the percentage of how much it has run will be printed out before the statistics.
