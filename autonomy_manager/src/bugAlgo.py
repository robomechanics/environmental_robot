#/usr/bin/env python3
import numpy as np

# for testing purpose only
obstacle_map = np.array([
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0]
])

turn_tol = 0.5
forward_tol = 0.5

# this function slices the front of the robot and returns the left, middle, and right section density value
def forward_section(obstacle_map):
    num_rows = 3
    #get the last 3 rows of the obstacle map
    m = len(obstacle_map)
    n = len(obstacle_map[0])
    n1 = n // 3
    n2 = n1 + (n % 3)
    frontVision = obstacle_map[-num_rows:,:]
    left = np.asarray([row[:n1] for row in frontVision]) 
    middle = np.asarray([row[n1:n1+n2] for row in frontVision]) 
    right = np.asarray([row[n1+n2:] for row in frontVision]) 
    front = middle[-1:,:]

    result = []
    #get the density of each section
    for each in (left, middle, right, front):
        numElem = np.prod(each.shape)
        sum = np.sum(each)
        density = sum / numElem
        result.append(density)
    return result

def obstacle_avoidance(obstacle_map):
    #get the density of each section
    density = forward_section(obstacle_map)
    #if the density of the front section is greater than 0.5, then the robot is blocked
    if density[3] > forward_tol:
        print("blocked")
        # back up
        return "backward"
    elif density[1] < forward_tol:
        print("go straight")
        # go straight
        return "forward"
    elif density[1] > turn_tol:
        if density[0] > density[2]:
            print("turn right")
            # turn right
            return "right"
        else:
            print("turn left")
            # turn left
            return "left"
    elif density[0] > turn_tol:
        print("turn right")
        # turn right
        return "right"
    elif density[2] > turn_tol:
        print("turn left")
        # turn left
        return "left"

# def bug2_algorithm(obstacle_map, start, goal):
#     def move_toward_goal(current_position, goal):
#         delta_x = goal[0] - current_position[0]
#         delta_y = goal[1] - current_position[1]
#         direction = np.array([delta_x, delta_y])
#         next_position = current_position + np.sign(direction)
#         return tuple(next_position)

#     def follow_obstacle_boundary(current_position, obstacle_map, goal):
#         neighbors = [(current_position[0] + dx, current_position[1] + dy)
#                      for dx in [-1, 0, 1] for dy in [-1, 0, 1]
#                      if dx != 0 or dy != 0]
#         free_neighbors = [neighbor for neighbor in neighbors if obstacle_map[neighbor] == 0]
#         distances = [np.linalg.norm(np.array(neighbor) - np.array(goal)) for neighbor in free_neighbors]
#         next_position = free_neighbors[np.argmin(distances)]
#         return next_position

#     def distance_to_goal(position, goal):
#         return np.linalg.norm(np.array(position) - np.array(goal))

#     position = start
#     path = [start]
#     leave_obstacle = False

#     while position != goal:
#         if not leave_obstacle:
#             next_position = move_toward_goal(position, goal)

#             if obstacle_map[next_position] == 1:  # Obstacle encountered
#                 leave_obstacle = False
#                 next_position = follow_obstacle_boundary(position, obstacle_map, goal)
#             else:
#                 leave_obstacle = True

#         if leave_obstacle:
#             next_position = move_toward_goal(position, goal)
#             if obstacle_map[next_position] == 1:  # Obstacle encountered
#                 leave_obstacle = False
#                 next_position = follow_obstacle_boundary(position, obstacle_map, goal)
#             elif distance_to_goal(next_position, goal) < distance_to_goal(position, goal):
#                 leave_obstacle = True

#         position = next_position
#         path.append(position)

#     return path

# # Example usage
# obstacle_map = np.array([
#     [0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 1, 1, 0],
#     [0, 0, 0, 1, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0]
# ])

# start = (0, 0)
# goal = (4, 6)

# path = bug2_algorithm(obstacle_map, start, goal)
# print("Path:", path)
