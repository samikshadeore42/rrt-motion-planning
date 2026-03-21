import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

def plot_tree(nodes, path=None, start=None, goal=None,obstacles=None):
    plt.figure(figsize=(6,6))
    
    # Plot tree
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x],
                     [node.y, node.parent.y],
                     "g-", linewidth=0.5)
            
    # Plot path
    if path:
        x=[p[0] for p in path]
        y=[p[1] for p in path]            
        plt.plot(x,y,"b-",linewidth=2,label="Path")
        
    if obstacles:
        for(ox,oy,r) in obstacles:
            circle=patches.Circle((ox,oy),r,color='red',alpha=0.3)
            plt.gca().add_patch(circle)
        
    # Plot start & goal
    if start:
        plt.plot(start[0],start[1],"ro",markersize=8,label="Start")
    if goal:
        plt.plot(goal[0],goal[1],"bo",markersize=8,label="Goal")
        
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.legend()
    plt.title("RRT Path Planning with Obstacles")
    plt.grid()
    plt.savefig("output.png")
    plt.show()
    
def animate_tree(nodes,start,goal,obstacles):
    plt.figure(figsize=(6,6))
    
    for node in nodes:
        if node.parent:
            plt.plot(
                [node.x,node.parent.x],
                [node.y, node.parent.y],
                "g-", linewidth=0.5
            )
            plt.pause(0.001)
            
    if obstacles:
        for(ox,oy,r) in obstacles:
            circle=patches.Circle((ox,oy),r,color='red',alpha=0.3)
            plt.gca().add_patch(circle)
            
    plt.plot(start[0],start[1],"ro")
    plt.plot(goal[0],goal[1],"bo")
    
    plt.xlim(0,10)
    plt.ylim(0,10)
    plt.title("RRT Growth Animation")
    plt.show()
    
    