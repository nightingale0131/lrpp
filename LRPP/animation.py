"""
    Module for animating LRPP behaviour
"""
import logging
logger = logging.getLogger(__name__)
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
from grid import SquareGrid

class Ani(object):
    """
        class for animating
    """
    def __init__(self, path, grid, goal, obsv_range):
        self.grid = grid
        self.goal = goal #  goal location in (x,y)
        self.knownWalls = set()
        self.path = path
        self.obsv_range = obsv_range

        self.init_drawing = self._empty()

        self._fig = plt.figure(figsize=(10,10))
        ax = plt.axes()
        # format figure to have border but no labels
        ax.get_xaxis().set_ticks([])
        ax.get_yaxis().set_ticks([])
        self._img = ax.imshow(self._empty(), interpolation = 'none')
        #logger.debug('Created animation figure')

    def __del__(self):
        plt.close(self._fig)
        #logger.debug('Closed animation figure')

    def _empty(self):
        data = 0.5*np.ones((self.grid.width, self.grid.height,4))
        data[:, :, 3] = 1

        for wall in self.grid.walls:
            (x,y) = wall
            data[y-1,x-1] = [0.3,0.3,0.3,1]

        return data

    def _initialize(self):
        data = self.init_drawing
        self._img.set_data(data)
        return self._img,

    def _observe(self,i):
        location = self.path[i]
        observations = self.grid.observe(location)
        for cell, state in observations.items():
            if state == self.grid.WALL: self.knownWalls.add(cell)

        return observations.keys()

    def _animate(self,i):
        red = [1,0,0,1]
        blue = [0,0,1,1]
        green = [0,1,0,1]
        black = [0,0,0,1]

        location = self.path[i]

        frame = self.init_drawing
        frame[self.goal[1]-1, self.goal[0]-1] = red
        frame[location[1]-1, location[0]-1] = green
        obsv = self._observe(i)

        for (x,y) in obsv:
            if (x, y) == location:
                continue
            else:
                (r,g,b,a) = frame[y-1,x-1]
                frame[y-1,x-1] = [r,g,b,0.7]
        for (x,y) in self.knownWalls:
            frame[y-1,x-1] = black
 
        self._img.set_data(frame)
        return self._img,

    def create_animation(self, outfile):
        video = animation.FuncAnimation(self._fig,
                                self._animate,init_func=self._initialize,
                                frames=range(len(self.path)), blit=True)
        #plt.show()
        video.save(outfile, dpi=80, writer='imagemagick')
        return video

