import numpy as np
from utils import is_collision_free

class Node:
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.parent = None
        self.cost=0
        
class RRT:
    def __init__(self,start,goal,obstacles,step_size=0.5,max_iter=800):
        self.start = Node(*start)
        self.start.cost=0
        self.goal= Node(*goal)
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.obstacles = obstacles
        self.sampled_points=[]
    
    def distance(self,n1,n2):
        return np.hypot(n1.x-n2.x,n1.y-n2.y)
    
    def get_random_point(self):
        r= np.random.rand()
        
        # goal bias
        if r<0.2:
            return (self.goal.x,self.goal.y)
        #direction bias
        elif r<0.4:
            dx=self.goal.x-self.start.x
            dy=self.goal.y-self.start.y
            return (
                self.start.x + np.random.rand() *dx,
                self.start.y + np.random.rand() *dy
            )
        
        elif r<0.6:
            best = self.get_promising_node()
            if best:
                return (
                    best.x + np.random.uniform(-1,1),
                    best.y + np.random.uniform(-1,1)
                )
        else:
            # Obstacle free random sampling
            while True:
                x = np.random.uniform(0,10)
                y = np.random.uniform(0,10)
            
                # check for insiide obstacle
                collision = False
                for(ox,oy,r) in self.obstacles:
                    if np.hypot(x-ox,y-oy) <=r:
                        collision = True
                        break
            
                if not collision:
                    return (x,y)

    def get_nearest_node(self,point):
        distances = [(node.x - point[0])**2 + (node.y - point[1])**2 for node in self.nodes]
        return self.nodes[np.argmin(distances)]

    def get_nearby_nodes(self,new_node,radius=1.5):
        nearby=[]
        for node in self.nodes:
            if self.distance(node,new_node) <=radius:
                nearby.append(node)
        return nearby
    
    def get_promising_node(self):
        if len(self.nodes) < 5:
            return None
        return min(self.nodes,key=lambda n: n.cost)
    
    def steer(self,from_node,to_point):
        direction = np.array([to_point[0]- from_node.x, to_point[1] - from_node.y])
        length = np.linalg.norm(direction)
    
        if length==0:
            return from_node
    
        direction=direction/length
        new_x=from_node.x + self.step_size*direction[0]
        new_y=from_node.y + self.step_size*direction[1]
    
        new_node = Node(new_x,new_y)
        new_node.parent=from_node
    
        return new_node

    def is_goal_reached(self,node):
        return np.hypot(node.x-self.goal.x, node.y-self.goal.y) < self.step_size

    def choose_best_parent(self,new_node,nearby_nodes):
        best_parent=None
        min_cost=float('inf')
        
        for node in nearby_nodes:
            if is_collision_free(node,new_node,self.obstacles):
                cost=node.cost+self.distance(node,new_node)
                
                if cost <min_cost:
                    min_cost=cost
                    best_parent=node
        
        if best_parent:
            new_node.parent=best_parent
            new_node.cost=min_cost
        
        return new_node
        
    def rewire(self,new_node,nearby_nodes):
        for node in nearby_nodes:
            if new_node.cost >= node.cost:
                continue
            
            if is_collision_free(new_node,node,self.obstacles):
                new_cost = new_node.cost+self.distance(new_node,node)
                
                if new_cost < node.cost:
                    node.parent = new_node
                    node.cost = new_cost
                    
    def build(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest = self.get_nearest_node(rand_point)
            new_node = self.steer(nearest,rand_point)
            self.sampled_points = []
            if not is_collision_free(nearest,new_node,self.obstacles):
                continue
        
            nearby = self.get_nearby_nodes(new_node)
            
            new_node= self.choose_best_parent(new_node,nearby)
            
            self.nodes.append(new_node)
            self.sampled_points.append((new_node.x,new_node.y))
            self.rewire(new_node,nearby)
            
            if self.is_goal_reached(new_node):
                if self.goal.parent is None or new_node.cost < self.goal.cost:
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost
        
        if self.goal.parent:
            return self.extract_path()
        
        return None

    def extract_path(self):
        path=[]
        node=self.goal
    
        while node:
            path.append((node.x,node.y))
            node=node.parent
        
        return path[::-1]
    
        