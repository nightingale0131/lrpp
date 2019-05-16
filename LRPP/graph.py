#!/usr/bin/python
import logging
logger = logging.getLogger(__name__)

class Graph(object):
    """
    Graph type objects are encoded as adjacency lists.
    They are dictionaries that look like:
        { vertex (key) : [vertices] (list) ...}

    Ex:
    g={ "a" : ["d"],
        "b" : ["c"],
        "c" : ["b", "c", "d", "e"],
        "d" : ["a", "c"],
        "e" : ["c"],
        "f" : []
       }

    """
    def __init__(self, graph_dict=None):
        """ initializes a graph object
            If no dictionary or None is given, an empty dictionary will be used
        """
        if graph_dict == None:
            graph_dict = {}
        self.graph_dict = graph_dict

    def vertices(self):
        # returns vertices of a graph
        return list(self.graph_dict.keys())

    def edges(self):
        # returns edges of a graph
        return self._generate_edges()

    def neighbours(self,v):
        # returns all the vertices that are adjacent to v
        if v not in self.graph_dict.keys():
            logger.warn("{} is not in the graph!".format(v))
            return []
        else:
            return self.graph_dict[v]

    def edges_that_contain(self,v):
        edges = []
        for edge in self._generate_edges():
            if v in edge:
                edges.append(edge)
        return edges

    def _generate_edges(self):
        # assume it is a directional graph
        edges = []
        for vertex in self.graph_dict.keys():
            for neighbour in self.graph_dict[vertex]:
                edges.append((vertex, neighbour))
        return edges

    def __str__(self):
        res = "vertices: "
        for k in self.graph_dict.keys():
            res += str(k) + " "
        res += "\nedges: "
        for edge in self._generate_edges():
            res += "\n" + str(edge) 
        return res

    def add_vertex(self, vertex):
        if vertex not in self.graph_dict.keys():
            self.graph_dict[vertex]=[]    # initialize empty list for new vertex

    def add_edge(self,edge):
        """ This only adds an edge in one direction!
            Both directions need to be added separately

            I decided to not allow more than one bi-directional edge between two vertices
            for simplicity
        """
        (vertex1,vertex2)=tuple(edge)

        if vertex1 in self.graph_dict.keys():
            if vertex2 not in self.graph_dict[vertex1]:
                self.graph_dict[vertex1].append(vertex2)
            else:
                logger.warning("edge {} already exists".format(edge))
        else:
            self.graph_dict[vertex1]=[vertex2]

        # if necessary, add vertex2 to the list of vertices as well
        if vertex2 not in self.graph_dict.keys():
            self.graph_dict[vertex2]=[]

    def del_edge(self,edge):
        """ Only removes edge in one direction!
        """
        (vertex1,vertex2)=tuple(edge)

        if vertex1 in self.graph_dict.keys():
            try:
                self.graph_dict[vertex1].remove(vertex2)
            except ValueError:
                logger.warn(str(edge) + "does not exist!")
        else:
            logger.warn("Vertex " + str(vertex1) + " does not exist!")

    def clear(self):
        """ delete all edges and vertices """
        self.graph_dict = {}

class WeightedGraph(Graph):
    def __init__(self, graph_dict=None, weight_dict=None):
        """ initializes a graph object
            If no dictionary or None is given, an empty dictionary will be used
            If no dictionary of weights given, then assign a weight of 1 to each edge
            Assumes weights of each edge are given
        """
        Graph.__init__(self, graph_dict)

        if weight_dict == None:
            weight_dict={}
            for edge in self._generate_edges():
                weight_dict[tuple(edge)]=1
        self.weight_dict=weight_dict

    def weight(self,edge):
        # returns the weight of the given edge
        edge=tuple(edge)
        reverse=(edge[1],edge[0])

        if edge in self.weight_dict.keys():
            return self.weight_dict[edge]
        elif edge[1] == edge[0]:
            return 0
        else:
            logger.warn(str(edge) + " weight is not defined!")

    def __str__(self):
        res = "vertices: "
        for k in self.graph_dict.keys():
            res += str(k) + " "
        res += "\nedges: "
        for edge in self._generate_edges():
            res += "\n" + str(edge) + ": " + str(self.weight(edge)) + " "
        return res


    def clear(self):
        """ delete all edges and vertices """
        self.graph_dict={}
        self.weight_dict={}

    def change_weight(self,edge,weight):
        edge_t=tuple(edge)

        try:
            self.weight_dict[edge_t]=weight
        except KeyError:
            logger.warn("Edge " + str(edge) + " does not exist!")

    def del_edge(self,edge):
        """ Only removes edge in one direction!
        """
        Graph.del_edge(self, edge)

        try:
            del self.weight_dict[tuple(edge)]
        except KeyError:
            logger.warn(str(edge) + " weight not defined!")

    def add_edge(self,edge,weight):
        """ This only adds an edge in one direction!
            Both directions need to be added separately
            Be careful as you can add multiple edges between the same vertices 
        """
        Graph.add_edge(self, edge)

        # add weight to edge
        self.weight_dict[tuple(edge)]=weight
