Designing and implementing a planner algorithm that computes a path for kings on a chessboard with restricted cells, considering the given constraints, involves careful planning and understanding of pathfinding algorithms, collision detection, and optimization techniques. Below, we'll outline a suitable approach and provide a basic implementation in C++.

Algorithm Overview
For this planner:

Breadth-First Search (BFS) for pathfinding: BFS is effective for finding the shortest path on an unweighted grid like a chessboard.
Sequential King Movement: Ensure each king takes one step in sequence, avoiding other kings.
Collision and Adjacency Avoidance: Implement checks to avoid restricted cells and proximity to other kings.
Performance and Memory Constraints: Use an appropriate data structure to maintain low memory usage and fast access times.
C++ Implementation
The implementation will include:

File Parsing: Load starting positions, targets, and restricted cells.
Chessboard Setup: Represent the board, restricted areas, and initial king positions.
Pathfinding: Implement the BFS to compute paths considering the given restrictions.
Sequential Movement Logic: Manage the movement order of kings to comply with the rules.
Output: If a solution is found, print the path for each move of each king.

Considerations for Completing the Solution
Implement the bfsPathFinding function to compute paths for all kings. This includes handling edge cases where a path may not be available or kings come too close to each other.
Implement the outputPlan function to display the path or note the impossibility of a solution.
Improve efficiency and ensure that the solution operates within the given memory and time constraints.
Consider using additional data structures or heuristics to avoid recomputation and handle kings' adjacency constraints more efficiently.
This solution sets a base for the planner algorithm, which will need further refinement and testing to ensure it meets all provided specifications and performs well under edge cases.