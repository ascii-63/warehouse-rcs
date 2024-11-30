class Node:
    def __init__(self, x, y, edge_num=0, parent=None):
        self.x = x
        self.y = y
        self.edge_num = edge_num  # Tracks distance traveled from start
        self.parent = parent
        # Cost of reaching this node (heuristic + distance)
        self.cost = float('inf')

    def __repr__(self):
        return f"Node({self.x}, {self.y}, edge_num={self.edge_num})"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


def get_neighbors(pt):
    """
    Returns the four neighboring nodes (up, down, left, right) of the current node `pt`.
    Assumes a grid-based map.
    """
    neighbors = [
        Node(pt.x, pt.y - 1),  # Up
        Node(pt.x, pt.y + 1),  # Down
        Node(pt.x - 1, pt.y),  # Left
        Node(pt.x + 1, pt.y)   # Right
    ]
    return neighbors


def vertex_collision(pt, time, planned_paths):
    """
    Checks if a vertex collision exists at node `pt` at time `time`.
    Returns True if a collision is detected, False otherwise.
    """
    for path in planned_paths:
        for node in path:
            if node.x == pt.x and node.y == pt.y and node.time == time:
                return True
    return False


def edge_collision(pt1, pt2, time, planned_paths):
    """
    Checks if there is an edge collision between nodes `pt1` and `pt2` at the given time.
    An edge collision occurs when two robots cross paths in the same direction at the same time.
    """
    for path in planned_paths:
        for i in range(len(path) - 1):
            if (path[i] == pt1 and path[i + 1] == pt2 and path[i].time == time) or \
               (path[i] == pt2 and path[i + 1] == pt1 and path[i].time == time):
                return True
    return False


def no_collision(pt, time, planned_paths):
    """
    Checks the surrounding neighbors of `pt` at `time` for any vertex or edge collisions.
    Returns a list of valid neighbors with no collisions.
    """
    valid_neighbors = []

    neighbors = get_neighbors(pt)

    for npt in neighbors:
        # Check vertex collision at the next time step
        vertex_flag = vertex_collision(npt, time + 1, planned_paths)
        if vertex_flag:
            continue

        # Check edge collision between current node and neighbor
        edge_flag = edge_collision(pt, npt, time, planned_paths)
        if edge_flag:
            continue

        # If no collision, add the neighbor to the valid list
        valid_neighbors.append(npt)

    return valid_neighbors

# Sample usage:


# Assume planned_paths is a list of paths that have already been planned for other robots
planned_paths = [
    [Node(0, 0, edge_num=0, parent=None), Node(0, 1, edge_num=1, parent=None)],
    [Node(1, 0, edge_num=0, parent=None), Node(2, 0, edge_num=1, parent=None)]
]

# Current robot's current position and time
current_node = Node(0, 0)
current_time = 1

# Get valid neighbors without collisions
valid_neighbors = no_collision(current_node, current_time, planned_paths)
print("Valid Neighbors:", valid_neighbors)
