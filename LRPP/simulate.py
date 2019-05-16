"""
Written by: Florence Tsang
Creation date: 2018-08-11

A* sample code from http://www.redblobgames.com/pathfinding/
"""

import logging, sys
logger = logging.getLogger(__name__)
import itertools, copy, random

from graph import Graph, WeightedGraph
from grid import SquareGrid,ObservedGrid
import utility

class Robot(object):
    """
        Simulates a robot interacting with its environment.
        Only works with SquareGrid as world and ObservedGrid as map.

        But generally Map is a child class of World, with the following methods:
            - World.observe(state): returns something that can be used by map.update
            - Map.neighbours(state): returns list of feasible states that robot can
              transition to from 'state'
            - Map.update(observation): updates 'map' to reflect new observations of world
    """
    BLOCKED = "blocked"
    UNBLOCKED = "unblocked"

    def __init__(self, obs_range=1):
        self.state = None  # state robot is in
        self.map = None    # no sensor inputs yet, robot's understanding of world
        self.world = None  # not placed in an environment yet 
        self.obs_range = obs_range # range of sensor

    def reset(self, G, state, obsv_type, obs_range=1):
        """ Inputs:
            G - environment robot is in, refer to Map class for methods G must have
            state - state of robot in G
            obsv_type - type of sensor inputs induced by robot interacting with G
                        Class with only G in constructor
        """
        self.state = state
        self.map = obsv_type(G)
        self.world = G
        self.obs_range = obs_range 

    def observe(self, next_steps=[]):
        """ Inputs: next_steps - remainder of path that robot must traverse

        """
        logger.debug('Taking observation at {}'.format(self.state))
        # take observation
        observation = self.world.observe(self.state, self.obs_range)
        self.map.update(observation)

        # check if robot's path is blocked
        prev_step = self.state
        for step in next_steps:
            if step not in self.map.neighbours(prev_step): 
                logger.info("Path is blocked at {}".format(step))
                return self.BLOCKED
            prev_step = step

        return self.UNBLOCKED

    def move(self, next_steps):
        """
            Robot moves from self.state to 1st state in next_steps 
        """
        next_state = next_steps[0]
        result = self.observe(next_steps)
        if result == self.UNBLOCKED:
            self.state = next_state
            logger.info("Moved to {}".format(next_state))
            return self.UNBLOCKED
        else:
            return self.BLOCKED

    def traverse_path(self, path):
        """ Input: path [v1, v2, ...] list of vertices

            Robot travels along path until an edge doesn't exist or it reaches the end of
            the path
        """
        # check if robot is at beginning of path
        try:
            start = path[0]
        except IndexError:
            logger.error("Path is empty!")
        except TypeError:
            logger.error("Path is not a list!")

        if self.state != start:
            logger.error("Robot is not at the beginning of the path!")
        else:
            for i in xrange(1, len(path)):
                result = self.move(path[i:])
                if result == self.BLOCKED:
                    return i # the index of vertex robot couldn't move to

        return self.UNBLOCKED

def setup_environment(file):
    """ Input: file - environment file that follows smallMap_grid example

        Output: grids, G - list of subgraphs, p - prob distr
    """
    grids = []
    G = []

    # import base map and events
    base_grid, events = utility.envs_from_csv(file)

    # calculate the total number of subgraphs and their prob distr
    combos, p = calc_all_combos(events)

    # generate each subgrid and their respective subgraph
    for combo in combos:
        current_grid = copy.deepcopy(base_grid)
        for i in combo:
            new_walls = events[i][0]
            current_grid.walls.update(new_walls)
        current_graph = utility.grid_to_graph(current_grid,True)

        grids.append(current_grid)
        G.append(current_graph)

    return grids, G, p

def calc_all_combos(events):
    """ Input: events - list of tuples: (event, probability)
    Output: combos - list of all different combinations of events, P - probability of each
    combination
    """
    combos = []
    P = []

    items = range(len(events))
    for N in xrange(0,len(items)+1):
        for subset in itertools.combinations(items, N):
            combos.append(subset)
            # calculate probability of subset <- list of tuples
            prob = 1
            for event in items:
                event_prob = events[event][1]
                if event in subset:
                    prob *= event_prob
                else:
                    prob *= 1 - event_prob
            P.append(prob)

    return combos, P

def select_env(grids,G,p):
    """ Input: SquareGrid list grids - full set of envs
               Graph list G
               list p -  probability distribution of subgraphs in G
                 [p1, p2, ...] index corresponds to subgraph in G
        Output: SquareGrid grid - selected grid
                Graph - selected graph
                int i - index of selected environment

        TODO: this could be implemented as a binary search by precalculating the cumulative
          probability. Doesn't seem to be worth the effort as this is only run once per
          task. Maybe when I'm bored.
    """
    x = random.random()
    p_cum = 0

    for i, prob in enumerate(p):
        p_cum += prob
        if x <= p_cum: break

    print('Selected G[{}]'.format(i))

    try:
        return grids[i],G[i],i
    except IndexError:
        logger.error("Probability distribution and graph set do not align")
        return None

def call_reactive_planner(robot,goal):
    """ most likely A*
        Input: 
               robot - robot type object
               goal - goal vertex
        Output: new_path (path that robot took while in reactive planning)
                can_go_to_goal boolean, True if robot reached the goal, False otherwise
    """
    logger.info('Running reactive planner')

    new_path = []
    can_go_to_goal = True
    count = 0
    while robot.state != goal and can_go_to_goal == True:
        count += 1
        logger.info('Calculating attempt {} to goal...'.format(count))
        came_from, cost = utility.a_star_search(robot.map,robot.state,goal)
        try:
            path = utility.reconstruct_path(came_from,robot.state,goal)
            logger.debug('New path: {}'.format(path))
        except KeyError:
            can_go_to_goal = False
            logger.info('A* could not calc path to goal')

        if can_go_to_goal == True:
            # only attempt to go to goal if A* could calculate a path
            logger.info('Attempting to traverse path...')
            res = robot.traverse_path(path)
            # No need to update robot's map, done automatically in traverse_path
            # But I want to remember the 'new information'
            if res != robot.UNBLOCKED:
                new_path.extend(path[:res])
            else:
                new_path.extend(path)

    return new_path, can_go_to_goal
