from rrt import RRT
from visualisation import plot_tree,animate_tree
from utils import path_length
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LogisticRegression

# Config
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

# Average Comparison
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

# Visualization
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


# AI Model training
def label_data(points,path):
    labels=[]
    for(x,y) in points:
        min_dist = min([np.hypot(x-px,y-py) for (px,py) in path])
        labels.append(1 if min_dist < 1.0 else 0)
    return labels

# collecct data from one run
rrt_star = RRT(start,goal,obstacles)
path = rrt_star.build()

X = rrt_star.sampled_points
y = label_data(X,path)

#train model
model = LogisticRegression()
model.fit(X,y)

print("AI model trained!")

# AI-GUIDED RRT*
rrt_ai = RRT(start,goal,obstacles)
rrt_ai.model = model

path_ai = rrt_ai.build()

if path_ai:
    print("AI Path Length:",path_length(path_ai))
    
plot_tree(rrt_ai.nodes,path_ai,start,goal,obstacles)