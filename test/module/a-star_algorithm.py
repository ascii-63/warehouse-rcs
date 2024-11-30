import heapq
from math import inf

# Define a simple Node class to hold the state of each point in the search


class Node:
    def __init__(self, x, y, parent=None, g=0, h=0, time=0, robot_id=None):
        self.x = x
        self.y = y
        self.parent = parent  # The parent node to reconstruct the path
        self.g = g            # Cost to reach this node
        self.h = h            # Heuristic cost (Manhattan distance)
        self.f = g + h        # f = g + h, for sorting open list
        self.time = time      # Time step when the robot reaches this node
        self.robot_id = robot_id  # Which robot this node belongs to (optional)
        self.edge_num = 0     # How far we've traveled (edge number)

    def __lt__(self, other):
        return self.f < other.f  # Used for heap comparison

    def __repr__(self):
        return f"Node({self.x}, {self.y}, g={self.g}, h={self.h}, f={self.f}, time={self.time})"

# Heuristic function: Manhattan Distance


def manhattan_distance(node, goal):
    return abs(node.x - goal.x) + abs(node.y - goal.y)

# Check for collisions (vertex or edge) based on previously planned paths


def no_collision(current_node, planned_paths, time):
    valid_neighbors = []
    for planned in planned_paths:
        # Check for vertex and edge collisions
        for point in planned:
            if point.time == time and (point.x == current_node.x and point.y == current_node.y):
                return []  # No valid move, collision detected
            # More collision detection logic can be added here
    return [current_node]  # If no collision, return the node itself

# A* Search Algorithm (Improved version)


def improved_astar_search(task, planned_paths):
    start = task['start']
    goal = task['goal']
    start_time = task['start_time']

    # Initialize the start node
    start_node = Node(start[0], start[1], g=0,
                      h=manhattan_distance(start, goal), time=start_time)

    # Open list (priority queue) and closed list (visited nodes)
    open_list = []
    closed_list = set()

    # Add the start node to the open list
    heapq.heappush(open_list, start_node)

    while open_list:
        # Get the node with the lowest f value (f = g + h)
        current_node = heapq.heappop(open_list)

        # If we have reached the goal, reconstruct the path and return
        if (current_node.x, current_node.y) == (goal[0], goal[1]):
            path = []
            while current_node:
                path.append(
                    (current_node.x, current_node.y, current_node.time))
                current_node = current_node.parent
            path.reverse()  # Reverse to get path from start to goal
            return path

        # Add to closed list
        closed_list.add((current_node.x, current_node.y, current_node.time))

        # Calculate the time for the next step
        current_time = current_node.time + 1  # Update time step

        # Get valid neighbors considering collisions
        valid_neighbors = no_collision(
            current_node, planned_paths, current_time)

        for neighbor in valid_neighbors:
            if (neighbor.x, neighbor.y, neighbor.time) in closed_list:
                continue  # Skip if the neighbor is already in the closed list

            # Calculate g, h, and f for the neighbor
            neighbor.g = current_node.g + 1  # Assuming a uniform cost of 1 per step
            neighbor.h = manhattan_distance(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.parent = current_node
            neighbor.time = current_time
            neighbor.edge_num = current_node.edge_num + 1

            # Add the neighbor to the open list if not already there
            if not any(n.x == neighbor.x and n.y == neighbor.y and n.time == neighbor.time for n in open_list):
                heapq.heappush(open_list, neighbor)

    # If we exhaust the open list without finding a solution, return failure
    return None


# Sample usage with task and planned paths
task = {
    'start': (0, 0),
    'goal': (5, 5),
    'start_time': 0
}

# Planned paths for other robots (dummy data for illustration)
planned_paths = [
    [Node(0, 0, time=0, robot_id=1), Node(1, 0, time=1,
                                          robot_id=1), Node(2, 0, time=2, robot_id=1)],
    [Node(2, 2, time=3, robot_id=2), Node(2, 3, time=4, robot_id=2)]
]

# Run the A* search
result_path = improved_astar_search(task, planned_paths)
print("Planned Path:", result_path)
