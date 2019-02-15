import random 
import matplotlib.pyplot as plt


class Obstacle:
    def __init__(self, min_size, max_size, area_size):
        self.size = random.random()*(max_size-min_size)+min_size    # obstacle square dimension
        self.x = random.random()*(area_size-2*self.size)+self.size  # x-coordinate of center
        self.y = random.random()*(area_size-2*self.size)+self.size  # y-coordinate of center

    def draw(self):
        # Plot the square obstacle
        plt.plot([self.x-self.size, self.x+self.size], [self.y-self.size, self.y-self.size], color='red')
        plt.plot([self.x-self.size, self.x+self.size], [self.y+self.size, self.y+self.size], color='red')
        plt.plot([self.x-self.size, self.x-self.size], [self.y+self.size, self.y-self.size], color='red')
        plt.plot([self.x+self.size, self.x+self.size], [self.y+self.size, self.y-self.size], color='red')

    def intersect(self,other):
        # Check whether two obstacles intersect each other
        return not(self.x+self.size < other.x-other.size or self.x-self.size > other.x+other.size or self.y+self.size < other.y-other.size or self.y-self.size > other.y+other.size)







