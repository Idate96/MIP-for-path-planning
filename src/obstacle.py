import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import random 



class obstacle:
    def __init__(self,min_size,max_size,area_size):
        self.size = random.random()*(max_size-min_size)+min_size
        self.x = random.random()*(area_size-2*self.size)+self.size
        self.y = random.random()*(area_size-2*self.size)+self.size
    def draw(self):
        ##plot the square obstacle
        #ca = plt.gca()
        #ca.add_patch(Rectangle(((self.x-self.size),(self.y-self.size)),self.size, self.size))
        plt.plot([self.x-self.size,self.x+self.size],[self.y-self.size,self.y-self.size])
        plt.plot([self.x-self.size,self.x+self.size],[self.y+self.size,self.y+self.size])
        plt.plot([self.x-self.size,self.x-self.size],[self.y+self.size,self.y-self.size])
        plt.plot([self.x+self.size,self.x+self.size],[self.y+self.size,self.y-self.size])
    def intersect(self,other):
        return not(self.x+self.size < other.x-other.size or self.x-self.size > other.x+other.size or self.y+self.size < other.y-other.size or self.y-self.size > other.y+other.size)


obstacles = []
min_size = 1
max_size = 3
area_size = 100
num_obs = 10
i=0
while i<num_obs:
    intercept  = False
    x = obstacle(min_size,max_size,area_size)
    for o in obstacles:
        if(x.intersect(o)):
            intercept = True
            break
    
    if(intercept):
        continue
    
    obstacles.append(x)
    obstacles[i].draw()
    i+=1

plt.show()


#     intercept = False
    
#     obstacles.append(obstacle(min_size,max_size,area_size))

#     print(i)
#     if(i!=0):
#         for j in range(0,i):
#             if (obstacles[i].intersect(obstacles[j])==False):   
#                 intercept = True
#     if(intercept==False):
#         obstacles[i].draw()
#         i+=1
# plt.show()



