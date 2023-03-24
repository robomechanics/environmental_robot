#/usr/bin/env python3
import numpy as np






# for testing purpose only
obstacle_map = np.array([
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0]
])

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
