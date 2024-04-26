## Rushi Bhavesh Shah
#### rushi.shah@nyu.edu

The primary goal of this implementation is to compute paths for multiple kings on a chessboard, ensuring that they neither enter restricted red cells nor come too close to one another. An A* pathfinding algorithm is utilized, specifically tailored to navigate within the constraints of a grid that represents the chessboard. Each cell on the grid can either be accessible or marked as an obstacle. Additionally, dynamic obstacles are generated based on the proximity restrictions between the kings to ensure compliance with the movement rules.

As the algorithm processes each king's move sequentially, the grid is recalculated to temporarily mark cells surrounding other kings as obstacles. This dynamic updating of the grid's state is essential for recalculating each king's path in the context of the current positions of all other kings, thereby adhering to the game's strategic requirements. The process iterates, updating paths and positions until all kings reach their designated targets or a determination is made that no valid paths exist. In the latter case, the algorithm will report the failure to find a solution, highlighting the need for careful coordination and dynamic adjustment based on real-time changes in the game state.

Algorithm :

(a) All the data is read from the test case files
(b) Environment (grid, obstacles, king start and goal positions) is initialized based on the read data
(c) Based on all the obstacles, path is planned for all the kings using A* Algorithm
(d) Additional obstacles are determined which are the positions of other kings (i.e. all the kings excpet the king that's about to move, hereafter referred as 'king of interest') and their neighbors
(e) The next step for the king of interest, based on the planned path is checked in the list of obstacles
(f) if present:
        add it to the obstacle list and plan a new path using A*
        compute the cost of this path and compare it to the original cost+1 (empty step)
        if cost of path with empty step < cost of new path:
            Add current node again to the king's path list     
        else:
            Add first node of the new path to the path list
    else:
        continue
    update king's current postion 
(g) Repeat from (c) to (f) till the time all the kings reach their goal positions
(h) Write the final paths for all the kings in a text file


Note : The main() function (of the last cell of my notebook) consists the final algorithm, which is yet incpomplete for implementation of few points from the above algorithm. I would finish the implementation and submit the final solution in next few days if allowed. I would really like to discuss my approach and your thoughts on it over a call.