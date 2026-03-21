import numpy as np

def is_collision_free(from_node,to_node,obstacles):
    for obs in obstacles:
        ox,oy,radius=obs
        
        if abs(from_node.x - ox) > radius+1: 
            continue
        
        #check multiple points along line
        for t in np.linspace(0,1,5):
            x=from_node.x + t*(to_node.x-from_node.x)
            y=from_node.y + t*(to_node.y-from_node.y)
            
            if np.hypot(x-ox,y-oy) <= radius:
                return False
    return True


def path_length(path):
    length=0
    for i in range(len(path)-1):
        length+=np.hypot(
            path[i][0]-path[i+1][0],
            path[i][1]-path[i+1][1]
        )
    return length