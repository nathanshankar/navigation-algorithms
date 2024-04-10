# AMR Coursework 2 - A* Path Planning (Code execution speed has been improved by eliminating function call overheads and by directly manipulating nodes)
class Node:
    def __init__(self, position, g_cost, h_cost, parent=None):
        """
        Initialize a node with its position, cost from start (g_cost), 
        heuristic cost to the goal (h_cost), and parent node.
        """
        self.position = position
        self.cost_from_start = g_cost
        self.cost_to_goal = h_cost
        self.total_cost = self.cost_from_start + self.cost_to_goal
        self.parent = parent

def do_a_star(grid, start, end, display_message):
    """
    Performing the A* algorithm to find the shortest path from start to end 
    in the given grid.

    Args:
    grid (list): A 2D list representing the grid with obstacles.
    start (tuple): Tuple representing the start position (x, y).
    end (tuple): Tuple representing the end position (x, y).
    display_message (function): Function to display messages.

    Returns:
    list: The shortest path from start to end.
    """
    # Getting valid neighbours of a point (up, down, left, right) (constraint 1)
    possible_directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    num_cols = len(grid[0])
    num_rows = len(grid)
    visited = set()
    start = (start[1], start[0])  # Convert start position to match grid indices
    end = (end[1], end[0])  # Convert end position to match grid indices

    open_list = [Node(start, 0, ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)**0.5)] # Initialize open list with start node and heuristic estimate to goal.

    while open_list:
        min_f_cost = float('inf')
        min_node = None
        # Find the node with minimum total cost (f-cost) in the open list
        for node in open_list:
            if node.total_cost < min_f_cost:
                min_f_cost = node.total_cost
                min_node = node
        current_node = min_node
        open_list.remove(current_node)
        current_pos = current_node.position

        if current_pos in visited:
            continue

        visited.add(current_pos)

        if current_pos == end:
            # Reconstruct and return the path
            path = []
            current = current_node
            while current:
                path.append((current.position[1], current.position[0]))  # Convert back to (x, y) format
                current = current.parent
            path = path[::-1]   # Reverse the path to get it from start to end
            display_message("Path found between the start and end points!")
            display_message("The path is: " + str(path))    # Displaying the path taken from the start to the end (constraint 3) 
            return path

        # Explore neighbours
        for dx, dy in possible_directions:
            neighbour_position = (current_pos[0] + dx, current_pos[1] + dy)
            x, y = neighbour_position
            if (0 <= x < num_cols and 0 <= y < num_rows) and (grid[y][x] != 0):
                if neighbour_position not in visited:
                    # Calculate heuristic cost for the neighbour
                    heuristic = ((end[0] - neighbour_position[0]) ** 2 + (end[1] - neighbour_position[1]) ** 2)**0.5  # Calculate Euclidean distance between two points (constraint 2)
                    # Create a neighbour node with updated costs and parent
                    neighbour = Node(neighbour_position, current_node.cost_from_start + 1, heuristic, current_node)
                    open_list.append(neighbour)
    # If no path is found, return an empty list to the gui
    display_message(f"No valid path found! End point {end} is unreachable from Start point {start}, mostly blocked by an obstacle!")
    return []
