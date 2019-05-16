"""
Written by: Florence Tsang
Creation Date: Aug 20, 2018

This is to be the main body of the LRPP algorithm.
That said, I want to convert this into a ROS node so keep it light and understandable
"""
#!/usr/bin/env python2
import sys
import os

import logging
logger = logging.getLogger(__name__)
import random
import copy
import sys, getopt

import timing
import rpp
from graph import Graph, WeightedGraph
from grid import SquareGrid, ObservedGrid
from classes import Node, Cost, Observation, Outcome, Map
from animation import Ani

from simulate import Robot, setup_environment, select_env, call_reactive_planner
import utility

def map_filter(newmap, M):
    """ Update the set of super maps M with newmap, update number of times encountered
        Map type newmap
        (Super)Map list M = [M0, M1, ...]

    Output: int i - index of supermap that was updated/added, None if nothing needs to be
                    updated
            M is updated but not returned
    """
    # when to merge? just go w/ the convention I have for now (first one that matches)

    for i, m in enumerate(M):

        result, new_info = m.agrees_with(newmap)
        if result == True and new_info == {}:
            # newmap is a subset of m
            logger.info('No change to supermaps')
            m.n += 1
            return None
        elif result == True and new_info != {}:
            # newmap and m agree, but there is new information in newmap
            # then merge the two maps.
            logger.info('Update supermap {}'.format(i))
            # update sensor information
            m.updateG(new_info)
            m.n += 1
            return i

    logger.info('Added new supermap')
    M.append(newmap)
    return len(M) - 1 # return last index

def update_p_est(M,t):
    """ Return updated estimated probability distribution
        t = total number of tasks so far (+1 b/c of base_graph) 
    """
    p = []
    for m in M:
        n = m.n # number of times this experience has been encountered
        prob = float(n)/t
        p.append(prob)

    return p

def extract_observed_path(policy, env_num, goal):
    current_node = policy[0]
    path = []

    logger.debug('Calculating path taken by optimal policy...')
    while current_node.s != goal and current_node.s != 'no goal':
        logger.debug('Now at {}'.format(current_node.s))
        for child_node in current_node.children:
            if env_num in child_node.Y:
                current_node = child_node
                break

        logger.debug('Moving to {}'.format(current_node.s))
        if current_node.s != 'no goal':
            path.extend(current_node.path)

    return path

def parse_command_line(argv):
    env_file = ''
    res_file = "../results/result.dat"
    start = None
    goal = None
    T = 0
    debug_lvl = logging.INFO

    try:
        opts, args = getopt.getopt(argv, "he:o:s:g:t:d:")
    except getopt.GetoptError:
        print("test.py -e <envfile> -s <start coordinates> -g <goal coordinates> -t"
                "<#tasks> [-o <results file> -d <debug level: debug,info>]")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("test.py -e <envfile> -s <start coordinates> -g <goal coordinates> -t"
                    "<#tasks> [-o <results file> -d <debug level: debug,info>]")
            sys.exit()

        elif opt == "-e": env_file = arg.strip()
        elif opt == "-s": start = utility.get_coordinates(arg)
        elif opt == "-g": goal = utility.get_coordinates(arg)
        elif opt == "-t": T = int(arg)
        elif opt == "-o": res_file = arg.strip()
        elif opt == "-d":
            if arg == "debug":
                debug_lvl = logging.DEBUG
            elif arg == "info":
                debug_lvl = logging.INFO
            else:
                print("Invalid debug level, using default: INFO")

    if env_file == '' or start == None or goal == None or T == 0:
        print("Missing information, please double check your input")
        sys.exit(2)
    else:
        print("Environment: {}".format(env_file))
        print("start: {}  goal: {}".format(start,goal))
        print("# tasks: {}".format(T))
        print("Results file: {}".format(res_file))
        print("Debug level: {}".format(debug_lvl))

    return env_file, start, goal, T, res_file, debug_lvl

if __name__ == '__main__':
    """
    # get information, user friendly menu
    filename = raw_input('Which environment? Make sure the file is in environments dir:\n')
    env_file = '../environments/' + filename.strip()
    print('Using {}'.format(env_file))

    start = utility.get_coordinates(raw_input('Enter start coordinates (ex. 1 1): '))
    goal = utility.get_coordinates(raw_input('Enter goal coordinates (ex. 4 1): '))
    """
    env_file, start, goal, T, res_file, debug_lvl = parse_command_line(sys.argv[1:])
    NO_GOAL = 'no goal'

    logging.basicConfig(filename='debug.log', filemode='w', level=debug_lvl)

    # setup directories
    if not os.path.exists("../results/figures"):
        os.mkdir("../results/figures")
        print("Created figures directory")

    if not os.path.exists("../results/videos"):
        os.mkdir("../results/videos")
        print("Created videos directory")

    # initialize robot
    robot_range = 5
    robot = Robot(robot_range)
    print("Robot range: {}".format(robot.obs_range))

    # setup environments
    grids, G, p = setup_environment(env_file)

    # simulate perfect and full maps for comparison
    worlds = []
    feature_set= set()
    for i, grid in enumerate(grids):
        temp = ObservedGrid(grid)
        temp.walls = grid.walls
        temp.unblocked = set(grid.vertices()) - grid.walls
        worlds.append(Map(temp))
        worlds[i].update_cost(goal)
        worlds[i].update_all_feature_states(robot.obs_range)
        feature_set.update(worlds[i].features())

    sys.stdout = open(res_file, 'w')

    timing.log('Finished env setup, now calc opt policy', True)
    opt_policy = rpp.solve_RPPv2(worlds, p, feature_set, start, goal)
    print('\nPolicy:')
    print('Vertex    Observation            Belief              Path')
    opt_policy[0].print_policy()
    timing.log('Finished calculating optimal policy')

    base_graph = G[0]
    base_grid = worlds[0]
    print('Environment has {} vertices, {} edges, and {} realizations.'
          .format(len(base_graph.vertices()),
                  len(base_graph.edges()),
                  len(G)
                 )
         )

    timing.log('Start LRPP')
    logger.debug('Preparing initial supermap')
    # convert base_grid to observedGrid type object
    base_grid = ObservedGrid(grids[0])
    base_grid.walls = grids[0].walls
    base_grid.unblocked = set(base_grid.vertices()) - grids[0].walls
    M = [Map(base_grid)]
    p_est = [1] # estimated prob distr of M
    M[0].update_cost(goal)
    M[0].update_all_feature_states(robot.obs_range)
    feature_set = M[0].features() # currently: set of all possible features in any M

    # setup data collection
    policy_cost = []
    reactive_cost = []
    opt_policy_cost = []
    est_prob_react = []

    for t in xrange(1,T+1):
        logger.info('Start of task {}'.format(t))
        timing.log('Start of task {}'.format(t))

        # solve RPP for set of supermaps
        print('Update policy')
        policy = rpp.solve_RPPv2(M, p_est, feature_set, start, goal)
        print('\nPolicy:')
        policy[0].print_policy()

        print('\nRandomly select subgraph...')
        # initialize robot state and realization
        world, task, env_num = select_env(worlds,G,p)
        grid = world.G
        robot.reset(grid, start, ObservedGrid, robot.obs_range)
        node = policy[0]
        path = []
        can_go_to_goal = True

        print('\nTraversing through policy...')
        # execute policy
        while robot.state != goal and can_go_to_goal == True:
            if node.opair == None:
                node = node.next_node()
            else:
                # take obsv and select node that matches observed feature state 
                feature = node.opair.E
                robot.observe() # just observe
                node = node.next_node(robot.map.state(feature))

                # later will need to deal with the case if feature state isn't expected
                # ie. node = None

            if node == None or node.s == NO_GOAL:
                # if a feature state is not expected (ie. unknown still b/c some
                # unexpected wall was blocking view of feature)
                print('There is no known path to goal, calling reactive planner...')
                reactive_path, can_go_to_goal = call_reactive_planner(robot, goal)
                path.extend(reactive_path)
            else:
                print('Moving to next node: {}'.format(node.s))
                res = robot.traverse_path(node.path)
                if res == robot.UNBLOCKED:
                    path.extend(node.path)
                else: # if there is an unexpected obstacle
                    print('{} blocked, calling reactive planner...'.format(node.path[res]))
                    reactive_path, can_go_to_goal = call_reactive_planner(robot, goal)
                    path.extend(node.path[:res])
                    path.extend(reactive_path)
                print('Now at {}'.format(robot.state))

        utility.draw_grid(grid,width=2,path=path)

        timing.log('Offline updates start')
        # update M
        newmap = Map(robot.map)
        newmap.update_all_feature_states(robot.obs_range)
        # the above line is really inefficient, I essentially have to recalc visibility
        # twice if there is new info or a new map is added, because map_filter only
        # updates G attribute, not the _feature dictionary if a pre-existing map is being
        # updated.
        updated_map = map_filter(newmap, M)
        p_est = update_p_est(M,t+1)
        logger.info("Estimated probabilities: {}".format(p_est))
        # update cost_to_goals and feature_set
        if updated_map == None:
            logger.info('No supermaps need to be updated')
        else:
            logger.info("Updating cost to goal for supermap {} and feature list".format(updated_map))
            M[updated_map].update_cost(goal)
            M[updated_map].update_all_feature_states(robot.obs_range)
            feature_set.update(M[updated_map].features())

            timing.log('Finished necessary updates, end of task {}'.format(t))

        # Do task with just reactive planner
        pure_reactive_path = []
        robot.reset(grid, start, ObservedGrid, robot.obs_range) 

        pure_reactive_path, pure_reactive_goal = call_reactive_planner(robot, goal)
        logger.debug('Path taken by only reactive planner: {}'.format(pure_reactive_path))

        # do task using optimal policy
        opt_pol_path = extract_observed_path(opt_policy,env_num,goal)
        logger.debug('Path taken by opt policy: {}'.format(opt_pol_path))

        # collect data
        policy_cost.append(utility.calc_path_distance(path, base_graph))
        reactive_cost.append(utility.calc_path_distance(pure_reactive_path, base_graph))
        opt_policy_cost.append(utility.calc_path_distance(opt_pol_path, base_graph))

        # generate video
        anim = Ani(path, grid, goal, robot.obs_range)
        video = anim.create_animation('../results/videos/{}.gif'.format(t))
        del anim # to close figure

        # generate images
        utility.save_to_image("../results/figures/{}_algo.png".format(t), grid, path=path,
                              start=start,goal=goal)
        utility.save_to_image("../results/figures/{}_react.png".format(t), grid,
                              path=pure_reactive_path, start=start, goal=goal)
        utility.save_to_image("../results/figures/{}_opt.png".format(t), grid,
                              path=opt_pol_path, start=start, goal=goal)

    # nicely print my data
    timing.log('Finished tasks')
    print('\nTask cost comparison\ntask  costP  costR  costOpt')
    for t in xrange(1,T+1):
        print('{:4}  {:6.2f}  {:6.2f}  {:6.2f}'
              .format(t,policy_cost[t-1],reactive_cost[t-1],opt_policy_cost[t-1]))

    print('average of costP = {:.2f}'.format(sum(policy_cost)/len(policy_cost)))
    print('average of costR = {:.2f}'.format(sum(reactive_cost)/len(reactive_cost)))
    print('average of costOpt = {:.2f}'.format(sum(opt_policy_cost)/len(opt_policy_cost)))

    sys.stdout.close()
    sys.stdout = sys.__stdout__
