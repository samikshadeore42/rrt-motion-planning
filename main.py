from rrt import RRT
from visualisation import plot_tree
from visualisation import animate_tree
from utils import path_length
import matplotlib.pyplot as plt
import numpy as np

iterations=[]
costs=[]

start = (0,0)
goal = (9,9)

#obstacles=(x,y,radius)
obstacles = [
    (4,4,1.5),
    (5,5,1.5),
    (6,6,1.5),
    (3,7,1.2),
    (7,3,1.2)
]

runs =10

rrt_lengths =[]
rrt_star_lengths=[]

for _ in range(runs):
    rrt = RRT(start,goal,obstacles)
    path_rrt=rrt.build()

    rrt_star = RRT(start,goal,obstacles)
    path_rrt_star = rrt_star.build()

    if path_rrt:
        rrt_lengths.append(path_length(path_rrt))
    if path_rrt_star:
        rrt_star_lengths.append(path_length(path_rrt_star))
    
print("Avg RRT:", sum(rrt_lengths)/len(rrt_lengths))
print("Avg RRT*:", sum(rrt_star_lengths)/len(rrt_star_lengths))

plot_tree(rrt.nodes,path_rrt,start,goal,obstacles)
plot_tree(rrt_star.nodes,path_rrt_star,start,goal,obstacles)

# animate the tree
animate_tree(rrt_star.nodes,start,goal,obstacles)
# RRT* CONVERGENCE GRAPH
iterations=[]
costs=[]

for i in range(200,2000,200):
    rrt_star=RRT(start,goal,obstacles,max_iter=i)
    path=rrt_star.build()
    
    if path:
        iterations.append(i)
        costs.append(path_length(path))
        
plt.figure()
plt.plot(iterations,costs,marker='o')
plt.xlabel("Iterations")
plt.ylabel("Path Length")
plt.title("RRT* Convergence (Path COst vs Iterations)")
plt.grid()
plt.show()
