"""
Written by: Florence Tsang
Creation date: 2018-08-14

Compatible with Python 2.7

Contains main RPP solvers and additional functions
Convert this into a module afterwards
"""

#!/usr/bin/env python2
import logging
logger = logging.getLogger(__name__)
import timing # measures runtime
import sys
import math

from collections import deque # queue class
from copy import deepcopy

from graph import Graph, WeightedGraph
from grid import SquareGrid, ObservedGrid
from classes import Cost, Node, Outcome, Observation
from dijkstra import dijkstra
from utility import isclose

def updateGraphs(conG,knownG,G,p_Xy):
    """ update conG and known G based on belief and their probabilities
        conG, knownG, and G[0] are weighted graphs 
        G = [G0, G1, ...] list of subgraphs
        p_Xy = {1: 0.3, ...} dictionary where keys are subgraph and values are the
        probabilities
    """
    logger.info("Updating conG and knownG...")
    knownG.clear()  # reset knownG
    for v in conG.vertices():
        knownG.add_vertex(v)
        for u in conG.neighbours(v):
            prob = 0
            for i in p_Xy.keys():
                if u in G[i].neighbours(v): prob+=p_Xy[i]

            conG.change_weight([v,u],prob)
            if isclose(prob,1):
                knownG.add_edge([v,u], G[0].weight([v,u]))

def solve_RPP(G,p,start,goal):

    # check if a probability is assigned to each graph, and if they equal to 1
    if len(G) != len(p) or not isclose(sum(p),1): sys.exit('solve_RPP failed: Re-check probabilities')

    """ Setup graphs ------------------------------------------
        - either import them from a text file or a module
        - all subgraphs need to have the same vertices
        - G=[G0,G1,G2,G3...]
        - p=[probability of G1, probability of G2, ...]
        - assume G0 is the master graph (all possible edges are in G0), and weights are
          defined in G0
    """
    environments=[int(n) for n in range(len(G))]
    conG=WeightedGraph(G[0].graph_dict) # conG only needs probabilities
    knownG=WeightedGraph()


    """ Determine possible constructive observations ----------
        - these edges do not exist in every given subgraph
        - probably horribly slow
    """
    logger.info('Generating all possible observations')
    theta = {}
    for v in conG.vertices():
        theta[v]=[]
        for u in conG.neighbours(v):
            # add both directions for each observation
            # single edge observations
            o=Observation({(v,u),(u,v)})
            theta[v].append(o)
            k = len(theta[v])-1
            logger.debug('Added observation ' + str(k) + ': ' + str(theta[v][k].E))

    """ Calculate edge subsets --------------------------------
        - probably horribly slow from iterating through all the edges every time
          edges_that_contain is called
    """
    logger.info('Calculating edge subsets for all observations')
    S = {}
    for i in environments:
        S[i] = {}
        for v in conG.vertices():
            S[i][v]=[]
            for o in theta[v]:
                subset = set(o.E) & set(G[i].edges_that_contain(v))
                S[i][v].append(subset)
            logger.debug('S[{}][{}] = {}'.format(i,v,S[i][v]))

    #timing.log('Time to prep algorithm inputs', True)

    """ Calculate policy --------------------------------------
        Q  = [(Y,parent node),(Y,parent node)...]
        From here onwards can probably be a function for algo 1
    """
    logger.info("Calculating min cost to goal for each subgraph...")
    cost_to_goal={}
    for i in environments:
        cost_to_goal[i]=Cost(dijkstra(G[i], goal))

    #timing.log('Time to calculate cost to goal', True)

    # if any of the subgraph probabilities are 0, that shouldn't even be in the initial
    # belief
    init_Y=[]
    for i in environments:
        if p[i] != 0:
            init_Y.append(i)

    logger.info("Generating policy... -------------")
    policy=[Node(init_Y,start), Node(init_Y)]
    policy[0].add_outcome(policy[1])
    Q=deque([(init_Y,start,policy[-1])]) # [-1] means last element of list

    while len(Q) > 0:
        (Y,v,new_node)=Q.popleft()
        logger.info('Start of a new Q loop for {}'.format(v))

        # check if it is possible for robot to go to goal 
        can_terminate=True
        for i in Y:
            if cost_to_goal[i].cost(v) != float('inf'):
                can_terminate=False
                break

        if can_terminate == True:
            logger.info('Cannot reach goal')
            new_node.add_leg("no goal")
        else:
            # calculate new probabilities of each i in Y
            p_sum=0
            p_Xy={}
            for i in Y:
                p_sum+=p[i]
            logger.debug('p_sum = {:.3f}'.format(p_sum))

            for i in Y:
                p_Xy[i]=p[i]/p_sum
                logger.info('p_Xy[{}] = {:.3f}'.format(i,p_Xy[i]))

            # update consistent and known graph
            updateGraphs(conG,knownG,G,p_Xy)
            logger.debug('conG:\n' + str(conG))
            logger.debug('knownG:\n' + str(knownG))

            # compute Rv ----------------------------------------
            # find constructive observations first
            logger.info('Finding constructive observations...')
            constrO={}
            for u in conG.vertices():
                # check if v has any constructive observations
                is_constructive=False
                for z in conG.neighbours(u):
                    if 0 < conG.weight([u,z]) < 1:
                        is_constructive=True
                        break
                if is_constructive==True:
                    logger.debug('{} has constructive observations'.format(u))
                    constrO[u]=[]
                    for o,observation in enumerate(theta[u]):
                        observation.outcomes=[] # clear outcomes from previous loops of Q
                        # check if observations at v are constructive
                        for i in Y:
                            is_new_outcome=True
                            possible_outcome=S[i][u][o]
                            for outcome in observation.outcomes:
                                if possible_outcome ^ outcome.Eo == set():
                                    outcome.Yo.append(i)
                                    outcome.p+=p_Xy[i]
                                    is_new_outcome=False
                            if is_new_outcome == True and p_Xy[i] != 0:
                                observation.outcomes.append(Outcome(possible_outcome,[i],p=p_Xy[i]))
                        if len(observation.outcomes) > 1:
                            # observation is only constructive if there is more than 1
                            # outcome
                            constrO[u].append(o)
                        # atm there should not be more than 2 outcomes per observation
                        if len(observation.outcomes) > 2:
                            logger.error("O{} at {} has more than 2 outcomes"
                                   " ?_?".format(o,u))
                    msg = "\n{}: {}".format(u,constrO[u])
                    for o in constrO[u]:
                        msg += "\n  O" + str(o) + ": "
                        for outcome in theta[u][o].outcomes:
                            msg += "\n  " + str(outcome)
                    logger.debug(msg)

            logger.info('Calculating min cost from {} to all other vertices'.format(v))
            c_knownG=Cost(dijkstra(knownG,v))
            Rv={} # dict of constructive and reachable observations

            logger.info('Checking constructive observations with eqn 9')
            # only add observations to Rv that satisfy eqn 9
            for u in constrO.keys():
                # calculate expected cost
                expected_cost = 0
                for i in Y:
                    if cost_to_goal[i].cost(u) < float('inf'):
                        expected_cost += cost_to_goal[i].cost(u)*p_Xy[i]

                cost_to_u = c_knownG.cost(u)
                cost_u = cost_to_u + expected_cost
                # + cost_to_take_obs (will require additional loop through constrO[u]

                if c_knownG.cost(goal) > cost_u:
                    Rv[u]=(constrO[u],cost_u) # Rv is tuples of (observations at u, cost)

            logger.info('Rv = {}'.format(Rv))

            if len(Rv) == 0:
                # if no elements left in Rv then set state to goal
                logger.info('Made it to the goal')
                new_node.add_leg(goal,None,c_knownG.path(goal))
            else:
                # else find the best observation (min of eqn 10), add tiebreaker
                minScore = float('inf') # result of eqn 10
                for u in Rv.keys():
                   # calculate entropy of each observation
                   (observations,cost_u) = Rv[u]
                   for o in observations:
                       logger.info('Calculating entropy of (O{},{})'.format(o,u))
                       obs = theta[u][o]  # observation object

                       H = 0 # negated conditional entropy
                       for outcome in obs.outcomes:
                           h = 0
                           for i in outcome.Yo:
                               p_cond=p_Xy[i]/outcome.p
                               logger.debug('p_cond of {} = {:.3f}'.format(i,p_cond))
                               if p_cond == 0:
                                    h += 0 # log(0) returns NaN
                               else:
                                    h += p_cond*math.log(p_cond)

                           H += outcome.p*h

                       H = -H
                       score = cost_u*H
                       logger.info('Entropy: {:.4f}  Score: {:.4f}'.format(H,score))
                       if score < minScore:
                            minScore = score
                            minO = (o,u,cost_u)

                       # tie breaker in the event of multiple observations score = 0
                       if score == 0 and minScore == 0:
                            if cost_u < minO[2]:
                                minO = (o,u,cost_u)

                (min_o, min_u, min_cost) = minO
                logger.info("MinO: (O{},{}), edges:"
                       " {}".format(min_o,min_u,theta[min_u][min_o].E))

                minObservation = deepcopy(theta[min_u][min_o])

                # set node observation, state, and path
                new_node.add_leg(min_u,minObservation,c_knownG.path(min_u))

                # Add new children nodes and add those to the queue
                for outcome in theta[min_u][min_o].outcomes:
                    policy.append(Node(outcome.Yo))
                    new_node.add_outcome(policy[-1])
                    Q.append((outcome.Yo,min_u,policy[-1]))

    return policy

def get_knownG(features, supermaps, belief):
    """
        Inputs: list of all possible features in list of supermaps
                Supermap list M = [M0, M1, ...]

        Output: knownG - same type as supermaps.G, updated for this node
    """
    logger.info("Updating knownG...")
    base_map = supermaps[0]
    knownG=type(base_map.G)(base_map.G) # use same type as Map.G
    # all feature states are unknown

    # compare features for all supermaps
    # features should be same type as keys in dict returned by supermaps.G.observe
    known_features = {}
    for feature in features:
        # logger.debug("Checking {}".format(feature))
        state = supermaps[belief[0]].feature_state(feature)
        is_known = True
        for i in belief:
            new_state = supermaps[i].feature_state(feature)
            if new_state != state:
                is_known = False

        # if feature state is the same in all supermaps in the belief, use it to update
        # the knownG
        if is_known: known_features[feature] = state

    logger.debug("known features = {}".format(known_features))
    knownG.update(known_features)
    return knownG

def solve_RPPv2(M, p, features, start, goal):
    """ This is the modified version for LRPP
        What's changed:
            - G is M = [(Eb,Eu,n),...] (supermap set from LRPP)
              Similar to G, no modifications should be made to this data here!
            - cost_to_goal is calculated outside of this solver, by LRPP, and updated
            - list of features is calculated outside of solver
    """
    logger.info('Running solve_RPPv2')
    if len(M) != len(p): sys.exit('solve_RPP failed: # supermaps and # prob mismatch.') 

    """ Setup graphs ------------------------------------------
        - all subgraphs need to have the same vertices
        - M=[M0, M1, ...]
        - assume M[0] is the master graph (all vertices defined)
    """
    environments=[int(n) for n in range(len(M))]
    base_map = M[0] # this is used pretty often

    # if any of the subgraph probabilities are 0, that shouldn't even be in the initial
    # belief
    init_Y=[i for i in environments if p[i] != 0]

    logger.info("Generating policy... -------------")
    policy=[Node(init_Y,start), Node(init_Y)]
    policy[0].add_outcome(policy[1])
    Q=deque([(init_Y,start,policy[-1])]) # [-1] means last element of list

    while len(Q) > 0:
        (Y,v,new_node)=Q.popleft()
        logger.info('Start of a new Q loop for {}'.format(v))

        # check if it is possible for robot to go to goal based on what it knows
        can_terminate=True
        for i in Y:
            if M[i].get_cost(goal, v) != float('inf'):
                can_terminate=False
                break

        if can_terminate == True:
            logger.info('Cannot reach goal')
            new_node.add_leg("no goal")
        else:
            # calculate P(Xy=i) of each i in Y
            p_sum=0
            p_Xy={}
            for i in Y:
                p_sum+=p[i]
            logger.debug('p_sum = {:.3f}'.format(p_sum))

            for i in Y:
                p_Xy[i]=p[i]/p_sum
                logger.info('p_Xy[{}] = {:.3f}'.format(i,p_Xy[i]))

            # update known graph & calc transition costs from v
            knownG = get_knownG(features, M, Y)
            logger.info("Calculating c_knownG")
            c_knownG = Cost(dijkstra(knownG, knownG.known_weight, v))

            # compute R ----------------------------------------
            R, D = useful_features( features, M, p_Xy, c_knownG, Y, goal )
            # R - [(O1,v1), (O2, v2), ...]
            # D - {v1: cost_v1, v2: cost_v2, ...}
            logger.info('R = {}'.format(R))

            # find minO -------------------------------------------------------
            logger.info("Finding minO...")
            if len(R) == 0:
                # If R is empty, then it's cheaper to go to the goal than to go
                #   anywhere else
                logger.info('Add goal to policy')
                new_node.add_leg(goal,None,c_knownG.path(goal))
            else:
                # else find the best observation (min of eqn 10), add tiebreaker
                minScore = float('inf') # result of eqn 10
                for item in R:
                    # calculate entropy of each observation
                    (obsv, v) = item
                    logger.debug('Calculating entropy of ({},{})'.format(obsv.E,v))

                    H = 0 # negated conditional entropy
                    for outcome in obsv.outcomes:
                        h = 0
                        for i in outcome.Yo:
                            # add conditional prob of supermaps that have mapped obsv
                            # p_cond -> P(Xy=i|Eo=E)
                            p_cond=p_Xy[i]/outcome.p
                            logger.debug('  p_cond of {} = {:.3f}'.format(i,p_cond))
                            if p_cond == 0:
                                h += 0 # log(0) returns NaN
                            else:
                                h += p_cond*math.log(p_cond)

                            for i in outcome.unknown:
                               # add conditional prob of supermaps that have not mapped
                               #    obsv 
                               p_cond = p_Xy[i]
                               logger.debug(' p_cond of {} = {:.3f}'.format(i,p_cond))
                               if p_cond == 0:
                                    h += 0 # log(0) returns NaN
                               else:
                                    h += p_cond*math.log(p_cond)

                        H += outcome.p*h

                    H = -H
                    score = D[v]*H
                    logger.info('Entropy: {:.4f}  Score: {:.4f}'.format(H,score))
                    if score < minScore:
                        minScore = score
                        minO = (obsv, v, D[v])

                    # in the event of multiple observations score = 0, select (O, v) w/ 
                    #   least expected cost to goal if we go to v
                    if score == 0 and minScore == 0:
                        if D[v] < minO[2]:
                            minO = (obsv, v, D[v])

                (min_o, min_u, min_cost) = minO
                logger.info("MinO: ({}, {})".format(min_o.E, min_u))

                minObservation = deepcopy(min_o)

                # set node observation, state, and path
                new_node.add_leg(min_u,minObservation,c_knownG.path(min_u))

                # Add new children nodes and add those to the queue
                for outcome in min_o.outcomes:
                    policy.append(Node(outcome.new_belief()))
                    new_node.add_outcome(policy[-1])
                    Q.append((outcome.new_belief(),min_u,policy[-1]))

    return policy

def useful_features( features, supermaps, p_Xy, c_knownG, belief, goal ):
    """
    Compute Rv
    Calculate set of reachable and constructive observations
    """

    # determine which features are constructive (compare feature subsets)-------
    logger.info('Checking constructive features...')
    constrO = [] # list of observation objects w/ constructive feature

    for feature in features:
        # init Observation object to store outcomes of feature
        obsv = Observation(feature)

        # vars for dealing with features that are UNKNOWN in some supermaps
        unknown_i = []
        has_unknowns = False
        outcome_norm = 0

        for i in belief:
            is_new_outcome = True
            possible_state = supermaps[i].feature_state(feature)

            if possible_state == supermaps[i].G.UNKNOWN:
                # if the feature state is unknown in supermap[i]
                unknown_i.append(i)
                has_unknowns = True
            else:
                for outcome in obsv.outcomes:
                    if outcome.state == possible_state:
                        outcome.Yo.append(i)
                        outcome.p += p_Xy[i]
                        is_new_outcome = False
                if is_new_outcome == True and p_Xy[i] != 0:
                    obsv.outcomes.append(Outcome(possible_state, [i], p_Xy[i]))

                outcome_norm += p_Xy[i] # for normalizing outcome probability

        # normalize outcome probabilities to exclude unknowns
        if has_unknowns == True:
            for outcome in obsv.outcomes:
                outcome.unknown.extend(unknown_i)
                outcome.p = outcome.p/outcome_norm

        if len(obsv.outcomes) > 1:
            # observation is only constructive if there is more than 1
            # outcome
            constrO.append(deepcopy(obsv))
            msg = "{}:\n".format(obsv.E)
            for outcome in obsv.outcomes:
                msg += "\t{}\n".format(outcome)
            logger.info(msg)

    # determine which vertices are reachable (eq 9)-----------------------------
    logger.info('Checking reachable vertices with eqn 9...')
    reachable_v = {}  # dict: {v: cost_v, u: cost_u,...}

    for v in supermaps[0].G.vertices():
        # calculate expected cost
        expected_cost = 0
        for i in belief:
            cost_to_goal = supermaps[i].get_cost(goal, v)
            if cost_to_goal < float('inf'):
                # is this still a valid assumption? In LRPP, the cost is not zero
                # if it is a no goal, it has to check via reactive planner, so
                # it's actually very expensive.
                # Does it make sense to ignore the cost of a no goal when
                # calculating the policy? I think so, if there is a supermap with
                # no possible path to goal, it'll just inflate the cost of EVERY
                # observation in constrO. Unless I can somehow calculate the
                # actual cost of the reactive planner to determine no goal, then I
                # can pick better observations to minimize that path
                expected_cost += cost_to_goal*p_Xy[i]

        cost_to_v = c_knownG.cost(v)
        cost_v = cost_to_v + expected_cost

        if c_knownG.cost(goal) > cost_v:
            reachable_v[v]= cost_v

    logger.debug('reachable_v = {}'.format(reachable_v))

    # combine the two to decide which (e,u) pairs are feasible.
    # if e in (e,u) isn't visible for ALL maps in the belief
    #    (e,u) is not feasible
    R=[] # list of constructive and reachable observations: (O, v)
    for v, cost_v in reachable_v.items():
        for o in constrO:
            is_viewable = True 

            for i in belief:
                if v not in supermaps[i].feature_viewable_from(o.E):
                    is_viewable = False
                    break

            if is_viewable: R.append((o,v))

    logger.info("Completed calculating set of constructive and reachable obsv pairs!")
    return R, reachable_v
