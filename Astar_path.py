#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import pandas as pd
from heapq import heappush, heappop
from itertools import count
import networkx as nx
from networkx.algorithms.shortest_paths.weighted import _weight_function


# In[ ]:


# Downloading the map as a graph object
G = ox.graph_from_bbox(17.2, 17.8, 78.40, 78.70, network_type='drive')


# In[ ]:


#removing isolated nodes
ox.utils_graph.remove_isolated_nodes(G)
remove=[2509297571]
G.remove_nodes_from(remove)


# In[ ]:


# define origin and desination locations
origin_point = (17.2348, 78.4293)
destination_point = (17.5501, 78.561)
# get the nearest nodes to the locations
origin_node = ox.get_nearest_node(G, origin_point)
destination_node = ox.get_nearest_node(G, destination_point)
# printing the closest node id to origin and destination points
origin_node, destination_node


# In[ ]:


#adding speed and travel_time attributes
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)


# In[ ]:


#heuristic
def time(origin_node, destination_node):
    return nx.dijkstra_path_length(G, origin_node, destination_node, weight='travel_time')

#astar_algorithm
def astar_path(G, source, target, heuristic=None, weight="weight"):
    
    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if heuristic is None:
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop
    weight = _weight_function(G, weight)

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent = pop(queue)

        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            # Do not override the parent of starting node
            if explored[curnode] is None:
                continue

            # Skip bad paths that were enqueued before finding a better one
            qcost, h = enqueued[curnode]
            if qcost < dist:
                continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():
            ncost = dist + weight(curnode, neighbor, w)
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost <= ncost, a less costly path from the
                # neighbor to the source was already determined.
                # Therefore, we won't attempt to push this neighbor
                # to the queue
                if qcost <= ncost:
                    continue
            else:
                h = heuristic(neighbor, target)
            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))

    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")


# In[ ]:


route = astar_path(G, origin_node, destination_node, heuristic=time, weight= 'transit_time')
route


# In[ ]:


def node_list_to_path(G, node_list):
    """
    Given a list of nodes, return a list of lines that together follow the path
    defined by the list of nodes.
    """
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(), key=lambda x: x['length'])

        # if it has a geometry attribute (ie, a list of line segments)
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute, the edge is a straight
            # line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)
    return lines


# In[ ]:


# getting the list of coordinates from the path (which is a list of nodes)
lines = node_list_to_path(G, route)

long2 = []
lat2 = []

for i in range(len(lines)):
    z = list(lines[i])
    l1 = list(list(zip(*z))[0])
    l2 = list(list(zip(*z))[1])
    for j in range(len(l1)):
        long2.append(l1[j])
        lat2.append(l2[j])
        print(long2[j] , lat2[j])

# In[ ]:


def plot_path(lat, long, origin_point, destination_point):
    
    # adding the lines joining the nodes
    fig = go.Figure(go.Scattermapbox(
        name = "Path",
        mode = "lines",
        lon = long,
        lat = lat,
        marker = {'size': 10},
        line = dict(width = 4.5, color = 'blue')))

    # adding source marker
    fig.add_trace(go.Scattermapbox(
        name = "Source",
        mode = "markers",
        lon = [origin_point[1]],
        lat = [origin_point[0]],
        marker = {'size': 12, 'color':"red"}))

    # adding destination marker
    fig.add_trace(go.Scattermapbox(
        name = "Destination",
        mode = "markers",
        lon = [destination_point[1]],
        lat = [destination_point[0]],
        marker = {'size': 12, 'color':'green'}))

    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)

    # defining the layout using mapbox_style
    fig.update_layout(mapbox_style="stamen-terrain", mapbox_center_lat = 30, mapbox_center_lon=-80)
    fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0},
                     mapbox = {
                         'center': {'lat': lat_center, 'lon': long_center},
                         'zoom': 13})

    fig.show()


# In[ ]:


plot_path(lat2,long2,origin_point,destination_point)


# In[ ]:


sum=0
for u, v in zip(route[:-1], route[1:]):
    length = round(G.edges[(u, v, 0)]['length'])
    travel_time = round(G.edges[(u, v, 0)]['travel_time'])
    sum=sum+travel_time
print('Total time taken to travel : ' )
print(str(round(sum/60,2)+ " minutes") 

