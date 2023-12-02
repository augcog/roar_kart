import bokeh
import pandas as pd
import sys
from bokeh.io import show
import numpy as np
from bokeh.plotting import gmap
from bokeh.models import GMapOptions
from bokeh.models import ColumnDataSource
import os

api_key = os.environ['GOOGLE_API_KEY']
if len(sys.argv) < 2:
    print('need a csv file as an argument')
    sys.exit(1)

zoom = 20


def plot(lat, lng, source, name='sat', zoom=20, map_type='satellite'):
    bokeh_width, bokeh_height = int(1920), int(1080)
    gmap_options = GMapOptions(lat=lat, lng=lng,
                               map_type=map_type, zoom=zoom)
    p = gmap(api_key, gmap_options, title=name,
             width=bokeh_width, height=bokeh_height)
    center = p.circle('lon', 'lat', size=5, alpha=1,
                      color='red', source=source)
    show(p)
    return p


def read_n_plot(file_paths):
    points, p, source = [], None, None
    for file_path in file_paths:
        df = pd.read_csv(file_path, delimiter=',')
        for i in range(len(df['lat'][:])):
            points.append([df['lat'][i], df['lon'][i]])
        if not source:
            source = ColumnDataSource(df)
        else:
            source.data['lat'] = np.append(source.data['lat'], df['lat'][:])
            source.data['lon'] = np.append(source.data['lon'], df['lon'][:])

        n = len(points)
        center = [sum([p[0] for p in points]) / n,
                  sum([p[1] for p in points]) / n]
    p = plot(center[0], center[1], source, zoom=int(zoom))


read_n_plot(sys.argv[1:])

# mapp = folium.Map(location=center, zoom_start=20, attr='Ersi')
# for p in points:
# folium.CircleMarker(p, radius=1, color='red', fill=True).add_to(mapp)
# mapp.show_in_browser()
