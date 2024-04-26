#!/usr/bin/env python

import os
import numpy as np
import math
import heapq


def get_map(test_case):
    
    map_file = open('test_cases/' + str(test_case) + '/map.txt', 'r')
    map_file = str(map_file.read())
    lines = map_file.strip().split('\n')
    obstacles = [tuple(line.split(',')) for line in lines[1:]]
    obstacles = [(int(obstacle[0]),int(obstacle[1])) for obstacle in obstacles]
    grid_size = (int(lines[0]), int(lines[0]))

    return grid_size, obstacles


def get_kings(test_case):
    '''
    This function returns the start and 
    the goal position of the kings
    as a list of tuples' tuples
    '''
    kings_file = open('test_cases/' + str(test_case) + '/kings.txt', 'r')
    kings_file = str(kings_file.read())
    start_goal = kings_file.strip().split('\n')
    total_no_kings = len(start_goal)
    
    kings = np.zeros(total_no_kings, dtype=object)

    for i, king in enumerate(start_goal):
        king_i = king.split(',')
        start = (int(king_i[0]),int(king_i[1]))
        goal = (int(king_i[2]),int(king_i[3]))
        kings[i] = (start, goal)

    return kings, total_no_kings

def initialize_env(test_case):

    grid_size, obstacles = get_map(test_case)
    kings, total_no_kings = get_kings(test_case)

    return grid_size, obstacles, kings, total_no_kings


## A* Algorithm

class AStarPathfinder:
    def __init__(self, grid_size, start_pos, goal_pos, obstacles):
        self.grid_size = grid_size
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.obstacles = obstacles
        self.grid = np.zeros(grid_size, dtype=np.int16)
        for obs in obstacles:
            self.grid[obs] = 1  # Mark obstacles on the grid


    def heuristic(self, a, b):
        """Calculate the Manhattan distance from point a to point b."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(self, node):
        """Generate the neighbors of a given node."""
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4 possible movements
        result = []
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_size[0] and 0 <= ny < self.grid_size[1]:
                if self.grid[nx, ny] == 0:  # Check if it's not an obstacle
                    result.append((nx, ny))
        return result

    def a_star_search(self):
        """Perform the A* search algorithm."""
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(self.start_pos, self.goal_pos), self.start_pos))
        came_from = {}
        g_score = {self.start_pos: 0}
        f_score = {self.start_pos: self.heuristic(self.start_pos, self.goal_pos)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == self.goal_pos:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g_score = g_score[current] + 1  # step cost is always 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal_pos)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path

# Usage example
# grid_size = (16, 16)
# start_pos = (1, 3)
# goal_pos = (3, 1)
# obstacles = [(0,2), (2,2), (2,1)]
# pathfinder = AStarPathfinder(grid_size, start_pos, goal_pos, obstacles)
# path = pathfinder.a_star_search()
# print(path)


#kings (ndarray) --> [((start1), (goal1))  ((start2), (goal2)) ... ((startN), (goalN))]
def get_paths(grid_size, kings, obstacles): 
    # all_paths = np.zeros(total_no_kings, dtype=object)
    all_paths = [None] * len(kings)
    all_path_costs = np.zeros(len(kings), dtype=int)

    for i, king in enumerate(kings):
        pathfinder = AStarPathfinder(grid_size, king[0], king[1], obstacles)
        path = pathfinder.a_star_search()
        if path != None:
            all_path_costs[i] = len(path)
            all_paths[i] = path
    
    solution_cost = np.sum(all_path_costs)
    
    return all_paths, solution_cos


def get_kings_neighbors(king_current):
    
    kernel = np.array([[(-1,-1),(-1,0),(-1,1)],
                    [(0,-1),(0,0),(0,1)],
                    [(1,-1),(1,0),(1,1)]])
    neighbors = king_current[0] + kernel

    return neighbors

## Check if any other kings and it's neighbors occupy next path node 
## inputs - path_node, kings

def get_obstacle_kings_neighbors(king_no, all_kings_current_pose):

    kings_other = np.delete(all_kings_current_pose, king_no, axis=0)
    kings_other_neighbors = np.empty([1,len(all_kings_current_pose)-1], dtype=object)
    for k, king in enumerate(kings_other):
        kings_other_neighbors[0][k] = get_kings_neighbors(king)   #all kings with their neighbors

    return kings_other_neighbors


## Checking the occurrence of the next node in the neighbors of other kings
def get_conflict_indices(next_node, total_no_kings, kings_other_neighbors):

    conflict_indices = []
    for k in range(total_no_kings):
        kth_king_neighbors = kings_other_neighbors[0][k]   # Neighbors of kth king
        indices = np.where(np.all(kth_king_neighbors == next_node, axis=0))
        conflict_index = (indices[0], indices[1])          # index of conflict of next_node with the neighbors of kth king
        conflict_indices.append(conflict_index)

    return conflict_indices

# Updating the obstacle list
def update_obstacles(next_node, grid_size, obstacles, kings_other_neighbors):
   
    updated_obstacles = [None] * (len(obstacles) + (len(kings_other_neighbors)*9))
    updated_obstacles[:len(obstacles)] = obstacles 
    if next_node != ():
        conflict_indices = get_conflict_indices(next_node, len(kings_other_neighbors), kings_other_neighbors)
        updated_obstacles = obstacles.extend(conflict_indices)

    else:
        idx = 0
        temp_obstacle_list = [None] * (len(kings_other_neighbors[0])*9)
        for ith_king_neighbors in kings_other_neighbors:
            for neighbor_matrix in ith_king_neighbors:
                for i, row in enumerate(neighbor_matrix):
                    for j, col in enumerate(row):
                        temp_obstacle_list[idx] = tuple(col)
                        idx += 1

        updated_obstacles[len(obstacles):] = (temp_obstacle_list)

        final_updated_obs = set()

        # Check edge conditions
        for k, obs in enumerate(updated_obstacles):
            if (obs[0] < grid_size[0] and obs[1] < grid_size[0]) and (obs[0] > 0 and obs[1] > 0):
    
                final_updated_obs.add(obs)  

        final_updated_obs = list(final_updated_obs)

        return final_updated_obs
            

def main():
    test_case = 8

    ## Initializing the Environment
    global grid_size, total_no_kings
    grid_size, obstacles, kings, total_no_kings = initialize_env(test_case)
    king_stat = [0] * total_no_kings
    next_node = ()
    final_path = [[None]] * total_no_kings
    count = 0

    while np.sum(king_stat) <= total_no_kings:
        for king, status in enumerate(king_stat):
            if status == 1:
                break
            else:
                if next_node == ():

                    all_kings_current_pose = np.empty(total_no_kings, dtype=object)
                    for k, start_goal in enumerate(kings):
                        all_kings_current_pose[k] = start_goal[0]
                    kings_other_neighbors = get_obstacle_kings_neighbors(king, all_kings_current_pose)
                    updated_obstacles = update_obstacles(next_node, grid_size, obstacles, kings_other_neighbors)
                    init_paths, init_SIC = get_paths(grid_size, kings, updated_obstacles)
                    all_kings_current_pose[0] = init_paths[king][0]
                    final_path[king].append(all_kings_current_pose[0])

                    if all_kings_current_pose[0] == kings[king][1]:   #Checking if current node is same as goal node
                        king_stat[king] = 1

                else:
                    
                    kings_other_neighbors = get_obstacle_kings_neighbors(king, all_kings_current_pose)
                    updated_obstacles = update_obstacles(all_kings_current_pose[king], grid_size, obstacles, kings_other_neighbors)
                    all_paths, new_SIC = get_paths(grid_size, kings, updated_obstacles)    
                    all_kings_current_pose[0] = all_paths[king][0]         
                    final_path[king].append(all_kings_current_pose[0])
                    
                    if all_kings_current_pose[0] == kings[king][1]:
                        king_stat[king] = 1 

    print(final_path)

if __name__ == "__main__":
    main()