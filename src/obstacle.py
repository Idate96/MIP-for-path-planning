import random 
import matplotlib.pyplot as plt

class obstacle:
    def __init__(self,x_min,x_max,y_min,y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
    def draw(self):
        ##plot the square obstacle
        #ca = plt.gca()
        #ca.add_patch(Rectangle(((self.x-self.size),(self.y-self.size)),self.size, self.size))
        plt.plot([self.x_min,self.x_max],[self.y_max,self.y_max],color='black')
        plt.plot([self.x_min,self.x_max],[self.y_min,self.y_min],color='black')
        plt.plot([self.x_min,self.x_min],[self.y_min,self.y_max],color='black')
        plt.plot([self.x_max,self.x_max],[self.y_max,self.y_min],color='black')