import random 
import matplotlib.pyplot as plt

class obstacle:
    def __init__(self,min_size,max_size,area_size):
        self.size = random.random()*(max_size-min_size)+min_size
        self.x = random.random()*(area_size-2*self.size)+self.size
        self.y = random.random()*(area_size-2*self.size)+self.size
    def draw(self):
        ##plot the square obstacle
        #ca = plt.gca()
        #ca.add_patch(Rectangle(((self.x-self.size),(self.y-self.size)),self.size, self.size))
        plt.plot([self.x-self.size,self.x+self.size],[self.y-self.size,self.y-self.size],color='red')
        plt.plot([self.x-self.size,self.x+self.size],[self.y+self.size,self.y+self.size],color='red')
        plt.plot([self.x-self.size,self.x-self.size],[self.y+self.size,self.y-self.size],color='red')
        plt.plot([self.x+self.size,self.x+self.size],[self.y+self.size,self.y-self.size],color='red')
    def intersect(self,other):
        return not(self.x+self.size < other.x-other.size or self.x-self.size > other.x+other.size or self.y+self.size < other.y-other.size or self.y-self.size > other.y+other.size)







