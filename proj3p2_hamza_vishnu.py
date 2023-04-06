import numpy as np
import time
import cv2
import math
from queue import PriorityQueue

R = 0.033   # Radius of the wheel
r = 0.105   # Radius of the robot
L = 0.160   # Distance between the wheels
map_width, map_height = 600, 250    # Map dimensions
threshold = 0.5 # Threshold for the goal node

# Define the move functions
def move_func(input_node, UL, UR, plot=False):
    t = 0
    dt = 0.1 #Time step
    Xi, Yi, Thetai = input_node #Input point's coordinates
    Thetan = 3.14 * Thetai / 180 #Convert end point angle to radian
    Xn, Yn = Xi, Yi #End point coordinates
    scale = 1
    Xs = Xi
    Ys = Yi 
    Cost=0
    while t<1:
        t = t + dt
        X_prev, Y_prev = Xs, Ys
        Xn += (0.5*R * (UL + UR) * math.cos(Thetan) * dt)
        Yn += (0.5*R * (UL + UR) * math.sin(Thetan) * dt)
        Thetan += (R / L) * (UR - UL) * dt
        Xs = Xn * scale
        Ys = Yn * scale
        Cost += math.sqrt(math.pow(Xn, 2) + math.pow(Yn, 2))
        if plot:
            cv2.arrowedLine(pixels, (int(X_prev), map_height - 1 - int(Y_prev)), (int(Xs), map_height - 1 - int(Ys)), (255, 0, 0), thickness=1)
        else:
            pass
    Thetan = (180 * (Thetan) / 3.14) % 360 #Convert back to degrees
    return (Xn, Yn, Thetan), Cost

def obstacles(clearance):
    #Define the Obstacle Equations and Map Parameters
    eqns = {
        "Rectangle1": lambda x, y: 0 <= y <= 100 and 100 <= x <= 150,
        "Rectangle2": lambda x, y: 150 <= y <= 250 and 100 <= x <= 150,
        "Hexagon": lambda x, y: (75/2) * np.abs(x-300)/75 + 50 <= y <= 250 - (75/2) * np.abs(x-300)/75 - 50 and 235 <= x <= 365,
        "Triangle": lambda x, y: (200/100) * (x-460) + 25 <= y <= (-200/100) * (x-460) + 225 and 460 <= x <= 510
    }

    clearance = clearance + r
    pixels = np.full((map_height, map_width, 3), 255, dtype=np.uint8)
    
    for i in range(map_height):
        for j in range(map_width):
            is_obstacle = any(eqn(j, i) for eqn in eqns.values())
            if is_obstacle:
                pixels[i, j] = [0, 0, 0]  # obstacle
            else:
                is_clearance = any(
                    eqn(x, y)
                    for eqn in eqns.values()
                    for y in np.arange(i - clearance, i + clearance + 1, 0.5)
                    for x in np.arange(j - clearance, j + clearance + 1, 0.5)
                    if (x - j)**2 + (y - i)**2 <= clearance**2
                )
                if i < clearance or i >= map_height - clearance or j < clearance or j >= map_width - clearance:
                    pixels[i, j] = [192, 192, 192]  # boundary
                elif is_clearance:
                    pixels[i, j] = [192, 192, 192]  # clearance
                else:
                    pixels[i, j] = [255, 255, 255]  # free space
    return pixels

# Define a function to check if current node is in range
def is_in_range(node):
    if len(node) == 3:
        x, y, _ = node
    else:
        x, y = node
    y = map_height - y - 1
    return 0 <= x < map_width and 0 <= y < map_height and (pixels[int(y), int(x)] == [255, 255, 255]).all()

def is_valid_node(node, visited):
    if not is_in_range(node):
        return False  # out of range
    x, y, _ = node
    y = map_height - y - 1
    if not (pixels[int(y), int(x)] == [255, 255, 255]).all():
        return False  # in obstacle space
    threshold_theta = math.radians(30)
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                neighbor_node = (x + i * threshold, y + j * threshold, k * threshold_theta)
                if neighbor_node in visited:
                    return False  # Too close to a visited node
    return True

# Define a function to check if current node is the goal node
def is_goal(current_node, goal_node):
    return np.sqrt((goal_node[0]-current_node[0])**2 + (goal_node[1]-current_node[1])**2) <= 1.5

# Define a function to find the optimal path
def backtrack_path(parents, start_node, goal_node):
    path, current_node = [goal_node], goal_node
    while current_node != start_node:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(start_node)
    return path[::-1]

# Define a function to calculate the euclidean distance
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2, _ = node2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Define the A* algorithm
def a_star(start_node, goal_node, display_animation=True):
    rows = int(map_height / threshold)  # number of rows
    cols = int(map_width / threshold)   # number of columns
    angles = int(360 / 30)           # number of angles
    V = [[[False for _ in range(angles)] for _ in range(cols)] for _ in range(rows)]    # visited nodes matrix 

    open_list = PriorityQueue()
    closed_list = set()
    cost_to_come = {start_node: 0}
    cost_to_go = {start_node: euclidean_distance(start_node, goal_node)}
    cost = {start_node: cost_to_come[start_node] + cost_to_go[start_node]}
    parent = {start_node: None}
    open_list.put((cost[start_node], start_node))
    visited = set([start_node])
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('animation.mp4', fourcc, 15.0, (map_width, map_height))

    while not open_list.empty():
        _, current_node = open_list.get()
        closed_list.add(current_node)
        x, y, theta = current_node
        if theta == 360: theta = 0
        V[int(y / threshold)][int(x / threshold)][int(theta / 30)] = True   # Mark current node as visited
        out.write(pixels)
        if display_animation:
            cv2.imshow('Explored', pixels)
            cv2.waitKey(1)
        # Check if current node is the goal node
        if is_goal(current_node, goal_node):
            approx_goal_node = current_node # Approximate goal node (within threshold distance) 
            cost[goal_node] = cost [current_node]   # Cost of goal node
            path = backtrack_path(parent, start_node, approx_goal_node) # Backtrack the path
            if display_animation:
                for node in path:
                    x, y, _ = node
                    cv2.circle(pixels, (int(x), map_height - 1 - int(y)), 1, (0, 0, 255), thickness=-1)
                out.write(pixels)
                cv2.waitKey(0)
            print("Final Cost: ", cost[goal_node])
            out.release()
            cv2.destroyAllWindows()
            return path

        for action in actions:    # Iterate through all possible moves
            new_node, move_cost = move_func(current_node, action[0], action[1])
            if is_valid_node(new_node, visited):    # Check if the node is valid
                i, j, k = int(new_node[1] / threshold), int(new_node[0] / threshold), int(new_node[2] / 30) # Get the index of the node in the 3D array
                if k == 12: k =0
                if not V[i][j][k]:  # Check if the node is in closed list
                    new_cost_to_come = cost_to_come[current_node] + move_cost
                    new_cost_to_go = euclidean_distance(new_node, goal_node)
                    new_cost = new_cost_to_come + new_cost_to_go    # Update cost
                    if new_node not in cost_to_come or new_cost_to_come < cost_to_come[new_node]:
                        cost_to_come[new_node] = new_cost_to_come   # Update cost to come
                        cost_to_go[new_node] = new_cost_to_go    # Update cost to go
                        cost[new_node] = new_cost   # Update cost
                        parent[new_node] = current_node  # Update parent
                        open_list.put((new_cost, new_node)) # Add to open list
                        visited.add(new_node)   # Add to visited list
                        # Draw vector from current_node to new_node
                        X1, _ = move_func(current_node, action[0], action[1], plot=True)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break

    out.release()
    cv2.destroyAllWindows()
    return None

# Get valid start and goal nodes from user input
while True:
    clearance = int(input("\nEnter the clearance: "))
    pixels = obstacles(clearance)
    start_node = tuple(map(int, input("Enter the start node (in the format 'x y o'): ").split()))
    if not is_in_range(start_node):
        print("Error: Start node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    goal_node = tuple(map(int, input("Enter the goal node (in the format 'x y'): ").split()))
    if not is_in_range(goal_node):
        print("Error: Goal node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    RPM1 = int(input("Enter RPM1: "))
    RPM2 = int(input("Enter RPM2: "))
    break

actions=[[RPM1,RPM1], [0,RPM1], [RPM1,0], [RPM2,RPM2], [0,RPM2], [RPM2,0], [RPM1,RPM2], [RPM2,RPM1]]  # List of actions

# Run A* algorithm
start_time = time.time()
path = a_star(start_node, goal_node)
if path is None:
    print("\nError: No path found.")
else:
    print("\nGoal Node Reached!\nShortest Path: ", path, "\n")
end_time = time.time()
print("Runtime:", end_time - start_time, "seconds\n")