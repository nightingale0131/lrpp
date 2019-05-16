"""
Written by: Florence Tsang
Creation date: 2018-08-13

Dijkstra module

"""

#!/usr/bin/python
import logging
logger = logging.getLogger(__name__)

from graph import Graph

def dijkstra(G, Weight, s):
    """ Inputs: G - Grid/Graph type object, must have vertices, weight, and neighbours methods
                Weight - function that takes in two vertices: weight( (a,b) )
                s - starting vertex in G
        Output: dictionary: {v: {d: 22, path: [x, y, z]}, ...}
                    v - vertex in G, dictionary will have an entry for every vertex except for
                        s
                    distance - total weight from s to v
                    path - list of vertices in path from s to v
    """
    if len(G.vertices()) == 0: logger.error('Given graph is empty!')

    Q={}
    S={}

    logger.info("Running dijkstra's...")
    for v in G.vertices():
        if v == s: Q[v]={"dist": 0, "prev":None}
        else: Q[v]={"dist": float('inf'), "prev":None}
    logger.debug(Q)

    logger.debug("Listing vertices in increasing distance from {start}".format(start=s))
    while len(Q) > 0:
        u=_dijExtractMin(Q)
        S[u]=Q.pop(u)

        if S[u]['dist'] == float('inf'):
            logger.debug('No reachable vertex left')
            for v in Q.keys():
                S[v]=Q[v]
            break
        else:
           for v in G.neighbours(u):
               if v in Q.keys():
                    u_ = {'name': u, 'dist': S[u]['dist']}
                    _dijRelax(u_, Q[v], Weight((u,v)))

    # extract paths
    logger.debug('Printing min distance and paths to each vertex from ' + str(s) + '...')
    result={}
    for v in S.keys():
        shortest_path = _dijPath(S,v)
        result[v]={'dist': S[v]['dist'], 'path': shortest_path}
        logger.debug('{vertex}: {d:.3f}  {p}'.format(vertex=v, d=S[v]['dist'],
            p=shortest_path))

    return result

def _dijExtractMin(Q):
    """ returns the vertex that contains the smallest distance
    """
    min_vertex = min(Q, key=lambda u: Q[u]['dist'])
    logger.debug("Min is " + str(min_vertex) + ", dist: " + str(Q[min_vertex]['dist']))
    return min_vertex

def _dijRelax(u,v,dist):
    """ Inputs: u - {'name' = u, 'dist' = 20} dict
                v - dict entry in Q
                dist - distance b/w u and v
    """
    if v['dist'] > u['dist'] + dist:
        v['dist'] = u['dist'] + dist
        v['prev'] = u['name']

def _dijPath(S,v):
    """ Inputs: S - dict {u: {dist: 22, prev: s}, ...}
                v - vertex I want the shortest path to
    """
    path=[v]

    if S[v]['dist'] == float('inf'): path = None
    else:
        # generates path backwards, starting from target vertex v
        while S[path[0]]['prev'] != None:
            path.insert(0, S[path[0]]['prev'])

    return path
