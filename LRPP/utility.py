"""
Sample code from http://www.redblobgames.com/pathfinding/
Copyright 2014 Red Blob Games <redblobgames@gmail.com>

Feel free to use this code in your own projects, including commercial projects
License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>


utility functions that don't belong anywhere else, but are needed for LRPP
Modified by: Florence Tsang
"""
from __future__ import print_function # for end="" to work
import logging
logger = logging.getLogger(__name__)
import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import csv

import grid
from priority_queue import PriorityQueue
from graph import Graph, WeightedGraph

def isclose(a,b,rel_tol=1e-09, abs_tol=0.0):
    # Compares equality of two floats
    # implementation provided in python documentation
    return abs(a-b) <= max(rel_tol*max(abs(a), abs(b)), abs_tol)

def from_id_width(id, width):
    """ redblobgames@gmail.com """
    return id % width, id // width


def draw_tile(graph, id, style, width):
    """ redblobgames@gmail.com """
    r = "."

    try:
        if id not in graph.unblocked: r = "?"
    except AttributeError:
        pass

    if ('number' in style
          and id in style['number']
          and style['number'][id] != float('inf')):
        r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = "\u2192"
        if x2 == x1 - 1: r = "\u2190"
        if y2 == y1 + 1: r = "\u2193"
        if y2 == y1 - 1: r = "\u2191"
    if 'start' in style and id == style['start']:
        r = "A"
    if 'goal' in style and id == style['goal']:
        r = "Z"
    if 'path' in style and id in style['path']:
        r = "@"
    if id in graph.walls: r = "#" * width

    return r


def draw_grid(graph, width=2, **style):
    """ redblobgames@gmail.com """
    if 'file' in style:
        outfile = style['file']
    else:
        outfile=sys.stdout

    for y in range(1, graph.height + 1):
        for x in range(1, graph.width + 1):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width),
                  end="",file=outfile)
        print('',file=outfile)

def to_image(grid):
    BLACK = [0,0,0,1]
    WHITE = [1,1,1,1]

    image = np.ones((grid.width, grid.height,4))
    image[:, :, 3] = 1

    for wall in grid.walls:
        (x,y) = wall
        image[y-1,x-1] = BLACK

    return image

def save_to_image(filename, grid, **style):
    red = [1,0,0,1]
    blue = [0,0,1,1]
    green = [0,1,0,1]

    data = to_image(grid)

    if 'path' in style:
        for v in style['path']:
            (x,y) = v
            data[y-1,x-1] = blue
    if 'start' in style:
        (x,y) = style['start']
        data[y-1,x-1] = green
    if 'goal' in style:
        (x,y) = style['goal']
        data[y-1,x-1] = red

    fig = plt.figure(figsize=(10,10))
    ax = plt.axes()
    img = ax.imshow(data, interpolation='none')

    # format figure to have border but no labels
    ax.get_xaxis().set_ticks([])
    ax.get_yaxis().set_ticks([])
    plt.savefig(filename, transparent=True)
    plt.close()

def reconstruct_path(came_from, start, goal):
    """ redblobgames@gmail.com
        Reconstruct a shortest path from a dictionary of back-pointers
    """
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()  # optional
    return path

def calc_path_distance(path, base_graph):
    """ Inputs: list path - list of vertices, defined in base_graph
                WeightedGraph base_graph

        Output: float distance (or cost of path)
    """
    distance = 0
    for i in xrange(len(path) - 1):
        distance += base_graph.weight((path[i],path[i+1]))

    return distance

def get_coordinates(string):
    """ Assumes string is in "x y" format
    """
    coordinates = [int(c) for c in string.split()]
    return tuple(coordinates[:2])

def envs_from_csv(file):
    """
    Convert csv file into SquareGrid type object AND rules to generate the different
    environments

    Returns (g: SquareGrid, events: list of: ([(a,b),...], p: probability of event occurring)
    """
    # extract grid
    with open(file, 'rb') as csvfile:
        grid_data = csv.reader(csvfile, delimiter=',')
        lines = []
        for i, row in enumerate(grid_data):
            if row[0] == "end of grid": break
            else:
                lines.append(row)

    g = grid.SquareGrid(len(lines[0]), len(lines))
    for row, line in enumerate(lines):
        for col, num in enumerate(line):
            if int(num) < 50:
                g.walls.add((col + 1, row + 1))

    # extract events
    events = []

    with open(file, 'rb') as eventfile:
        for j in xrange(i):
            eventfile.next()
        for line in eventfile: 
            if line[0] == '0': 
                # extract probability of event
                semicolon = line.find(':')
                sharp = line.find('#') # indicates of coordinates
                newline = line.find('\n')
                p = float(line[0:semicolon])

                # extract coordinates of wall location  
                if sharp == -1:
                    vertex_data = line[semicolon+2:]
                else:
                    vertex_data = line[semicolon+2:sharp]
                vertex_data = [c.strip() for c in vertex_data.split(' ') if c.strip()]
                # vertex_data: ['a,b','c,d'...]
                vertices = []
                for substring in vertex_data:
                    comma = substring.find(',')
                    coordinates = (int(substring[0:comma]), int(substring[comma+1:]))
                    # coordinates: (x,y)
                    vertices.append(coordinates)
                events.append((vertices,p)) 

    logger.debug('Printing events...')
    for event in events:
        logger.debug('Prob: {} Walls: {}'.format(event[1],event[0]))

    return g,events

def grid_to_graph(grid, is_weighted=False):
    """
    Convert grid to Graph type format (adjacency list), with an option to create a
    weighted graph

    This is a pre-cursor for optimizing the grid later
    """
    if is_weighted == False: graph = Graph()
    else: graph = WeightedGraph()

    for x in xrange(1, grid.width + 1):
        for y in xrange(1, grid.height + 1):
            v = (x,y)
            graph.add_vertex(v)
            if grid.passable(v):
                # only add neighbors if the vertex is not in a wall
                neighbors = grid.neighbours(v)
                for u in neighbors:
                    if is_weighted == False:
                        graph.add_edge((v,u))
                    else: 
                        graph.add_edge((v,u),grid.weight((v,u)))

    return graph

def heuristic(a, b):
    """ redblobgames@gmail.com """
    (x1, y1) = a
    (x2, y2) = b
    return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))
    #return abs(x1 - x2) + abs(y1 - y2)


def a_star_search(graph, start, goal):
    """A star algorithm courtesy of http://www.redblobgames.com/pathfinding/

    """
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.pop()

        if current == goal:
            break

        for next in graph.neighbours(current):
            new_cost = cost_so_far[current] + graph.weight((current, next))
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far
