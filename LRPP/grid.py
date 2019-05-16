import math
import utility
import logging
logger = logging.getLogger(__name__)
# import visilibity as vis

class SquareGrid(object):
    """
        Simulates occupancy grid.
        Each vertex is the center of a cell in the grid.
        Each feature is also the cell.
    """
    # features
    WALL = 1
    PASSABLE = 0

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = set()
        self._visibility = {} # adjacency matrix that holds which pairs of vertices are
                                # visible to each other

    def passable(self, id):
        (x,y) = id
        return id not in self.walls

    def in_bounds(self, id):
        # grid boundaries are from 1 to width/height
        (x, y) = id
        return 0 < x <= self.width and 0 < y <= self.height

    def vertices(self):
        # return all cells in the grid
        results = [(x, y) for x in range(1, self.width+1)
                          for y in range(1, self.height+1)
                   if (self.in_bounds((x, y)))]
        return results

    def weight(self, edge):
        (from_node, to_node) = edge
        (x1, y1) = from_node
        (x2, y2) = to_node

        # unknowns are treated as passables
        if from_node in self.walls or to_node in self.walls:
            return float('inf')
        else:
            return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def neighbours(self, id):
        # this function returns the surrounding cells that the robot can GO to
        # 'neighbors' include any squares that are diagonal to (x,y) as well
        (px, py) = id
        results = [(x, y) for x in range(px - 1, px + 2)
                          for y in range(py - 1, py + 2)
                   if (self.in_bounds((x, y)) and self.passable((x,y)))]

        # dirty check for diagonal cells that can't be neighbours.
        # . a x  <- In this case , b is not a neighbour of a, but c is
        # c . b
        to_be_deleted = [(px,py)]
        if (px + 1, py) not in results or (px, py - 1) not in results:
            to_be_deleted.append((px + 1, py - 1))
        if (px, py - 1) not in results or (px - 1, py) not in results:
            to_be_deleted.append((px - 1, py - 1))
        if (px - 1, py) not in results or (px, py + 1) not in results:
            to_be_deleted.append((px - 1, py + 1))
        if (px, py + 1) not in results or (px + 1, py) not in results:
            to_be_deleted.append((px + 1, py + 1))

        for node in to_be_deleted:
            try:
                results.remove(node)
            except ValueError:
                pass

        return list(results)

    def observe(self, position, obs_range=2):
        # returns dict of cells: state that robot can SEE
        # - obs_range isn't needed anymore, but keeping it in case I change my mind
        cells = self._visibility[position]
        return {cell: self.WALL if cell in self.walls else self.PASSABLE for cell in cells}

    def calc_vis(self, obs_range=2):
        self._visibility = {} # clear previous data, redo all calculations

        for v in self.vertices():
            v_is_passable = self.passable(v)
            if v_is_passable:
                # can see your own state, check if v already has state
                if self._visibility.has_key(v):
                    self._visibility[v].add(v)
                else:
                    self._visibility[v] = {v}

            # logger.debug("Calculating visible cells from {}".format(v))
            (vx, vy) = v # break apart for readability

            for oct in xrange(0,4): # go through the right half octants
                # logger.debug("Checking octant {}".format(oct))

                line = ShadowLine()
                row = 2
                for row in xrange(2, obs_range+2):
                    # logger.debug("Checking row {}".format(row))
                    # check if row is still in bounds of map
                    dx, dy = transform_octant(row, 1, oct)
                    x = vx + dx
                    y = vy + dy
                    if not self.in_bounds((x,y)): break

                    if line.is_full(): break

                    for col in xrange(1, row + 1):
                        # if out of bounds, move to next row 
                        dx, dy = transform_octant(row, col, oct)
                        x = vx + dx
                        y = vy + dy
                        # logger.debug("Checking cell ({}, {})".format(x,y))

                        if not self.in_bounds((x,y)): break

                        # if there is a full shadow, all remaining cells in octant
                        # are not visible
                        if line.is_full(): break

                        projection = project_cell(row, col)

                        # set visibility
                        if not line.in_shadow(projection):
                            if v_is_passable:
                               # if v is not a wall, then (x,y) is visible from v
                               self._visibility[v].add((x,y))

                            # if (x,y) is a wall, add to shadow line
                            if not self.passable((x,y)):
                                line.add(projection)
                            else:
                                if self._visibility.has_key((x,y)):
                                    self._visibility[(x,y)].add(v)
                                else:
                                    self._visibility[(x,y)] = {v}


                    row += 1

    def find_visible_cells_from(self, vx, vy):
        # return dictionary with cells that are visible to each other
        # this method is only for testing purposes!
        visible_dict = { (vx,vy): set((vx,vy)) }

        for oct in xrange(0,4): # go through the right half octants
            logger.debug("Checking octant {}".format(oct))

            line = ShadowLine()
            row = 2
            while not line.is_full():
                logger.debug("  row {}".format(row))
                # check if row is still in bounds of map
                dx, dy = transform_octant(row, 1, oct)
                x = vx + dx
                y = vy + dy
                if not self.in_bounds((x,y)): break

                for col in xrange(1, row + 1):
                    # if out of bounds, move to next row 
                    dx, dy = transform_octant(row, col, oct)
                    x = vx + dx
                    y = vy + dy
                    logger.debug("  cell ({}, {})".format(x,y))

                    if not self.in_bounds((x,y)): break

                    # if there is a full shadow, all remaining cells in octant
                    # are not visible
                    if line.is_full(): break

                    projection = project_cell(row, col)

                    # set visibility
                    if not line.in_shadow(projection):
                        if not self.passable((vx,vy)):
                            visible_dict[(vx,vy)].add((x,y))

                        # if (x,y) is a wall, add to shadow line
                        if not self.passable((x,y)):
                            line.add(projection)
                        else:
                            visible_dict[(x,y)] = {(vx,vy)}

                logger.debug(line)
                logger.debug("Line is full? {}".format(line.is_full()))
                row += 1
            # end while loop
        return visible_dict

class ObservedGrid(SquareGrid):
    # class for collecting data on ^, has additional UNKNOWN state

    def __init__(self, G):
        SquareGrid.__init__(self, G.width, G.height)
        self.unblocked = set()
        self.UNKNOWN = -1 

    def known_weight(self, edge):
        # weight to any cells with unknown state are inf
        # need to do adjacency checking b/c neighbours only checks for walls
        (from_node, to_node) = edge
        (x1, y1) = from_node
        (x2, y2) = to_node
        cells_to_check = [from_node, to_node, (x1+(x2-x1), y1), (x1, y1+(y2-y1))]

        # unknowns are treated as walls
        for cell in cells_to_check:
            if cell not in self.unblocked: return float('inf')

        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def update(self, observation):
        # update observed cells 
        # observation - returned result of SquareGrid.observe
        walls_in_obs = set()
        unblocked_in_obs = set()

        for cell, state in observation.items():
            if state == self.WALL: walls_in_obs.add(cell)
            else: unblocked_in_obs.add(cell)

        self.walls.update(walls_in_obs)
        self.unblocked.update(unblocked_in_obs)

    def state(self, cell):
        # return state of cell in ObservedGrid
        # assume cell is in-bounds
        if cell in self.walls: return self.WALL
        elif cell in self.unblocked: return self.PASSABLE
        else: return self.UNKNOWN

    def observe(self, position, obs_range=2):
        # returns dict of cells: state that robot can SEE
        # - obs_range isn't needed anymore, but keeping it in case I change my mind
        # Different from above because cell -> state mapping is different in parent
        cells = self._visibility[position]
        return {cell: self.state(cell) for cell in cells}

    def clear():
        # clears all fields
        self.walls = set()
        self.unblocked = set()

""" 
If I have time I'll try to implement something else, for now this will
have to do.

The following code uses the MIT License:

    Copyright (c) 2000-2014 Bob Nystrom

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated
    documentation files (the "Software"), to deal in the
    Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute,
    sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall
    be included in all copies or substantial portions of the
    Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
    KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
    OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
    OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""
class ShadowLine(object):
    def __init__(self):
        self.shadows = []

    def in_shadow(self, projection):
        # checks if cell is not visible, must be completely covered by shadow
        for shadow in self.shadows:
            if shadow.contains(projection):
                return True

        return False

    def is_full(self):
        # checks if shadowline is full (from 0 to 1)
        if self.shadows == []: return False
        return utility.isclose(self.shadows[0].start, 0) and utility.isclose(self.shadows[0].end, 1)

    def add(self, new):
        # logic to add a new shadow to shadow line

        # first, find where shadow should be inserted
        index = 0
        for i, shadow in enumerate(self.shadows):
            if shadow.start >= new.start:
                index = i
                break
            if i == len(self.shadows) - 1:
                # if new shadow is last
                index = len(self.shadows) 


        # determine if new shadow overlaps previous or next
        overlaps_prev = False
        if index > 0 and self.shadows[index-1].end >= new.start:
            prev_shadow = self.shadows[index-1]
            overlaps_prev = True

        overlaps_next = False
        if index < len(self.shadows) and self.shadows[index].start <= new.end:
            next_shadow = self.shadows[index]
            overlaps_next = True

        # insert and combine with other shadows
        if overlaps_next and overlaps_prev:
            # merge all three shadows
            prev_shadow.end = next_shadow.end
            self.shadows.remove(next_shadow)
        elif overlaps_next:
            # merge new with next
            next_shadow.start = new.start
        elif overlaps_prev:
            # merge new with prev
            prev_shadow.end = new.end
        else:
            # doesn't overlap with anything, just add new shadow
            self.shadows.insert(index, new)

    def __str__(self):
        msg = "["
        for shadow in self.shadows:
            msg += " {}".format(shadow)
        msg += " ]"
        return msg

class Shadow(object):
    # range of shadow on shadowline, between 0 & 1
    def __init__(self, start, end):
        if start < 0 or start > 1:
            logger.error("start of shadow is not in [0,1]!")
        if end < 0:
            logger.error("end of shadow is before 0!")
        if start > end:
            logger.error("shadow ends before it starts!")

        self.start = start

        if end > 1: self.end = 1
        else: self.end = end

    def contains(self, other):
        # other: another Shadow object
        if other.start >= self.start and other.end <= self.end:
            return True
        return False

    def overlaps(self, other):
        if other.start > self.start and other.start < self.end:
            # if the start of other is between start and end of self
            return True
        if other.end > self.start and other.end < self.end:
            # if the end of other is between start and end of self
            return True
        return False

    def __str__(self):
        return "({:.3f},{:.3f})".format(self.start, self.end)

def transform_octant(row, col, octant):
    # transforms octant coordinates into grid coordinates
    # only need the right half octants
    real = {
        0: ( col-1, 1-row ),
        1: ( row-1, 1-col ),
        2: ( row-1, col-1 ),
        3: ( col-1, row-1 )
    }
    (x, y) = real[octant]
    return x, y

def project_cell(row, col):
    # determine projection from 'origin' of octant to cell
    topLeft = (col-1)/float(row)
    bottomRight = col/float(row-1)
    return Shadow(topLeft, bottomRight)
