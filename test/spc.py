import heapq
import math
import random

# ================================
# Helper Functions: A* and Collision Checking
# ================================


def heuristic(a, b):
    """Heuristic function for A* (Manhattan distance)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(start, goal, planned_paths):
    """
    A* search algorithm that plans a path from start to goal avoiding the planned paths.
    Returns the path as a list of coordinates or None if no path is found.
    """
    open_list = []
    closed_list = set()
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    # Push the starting point into the open list
    heapq.heappush(open_list, (f_score[start], start))

    while open_list:
        # Get the node with the lowest f_score
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            # Return the path
            return [{"time": 0, "start": start, "goal": goal, "path": path}]

        closed_list.add(current)

        # Explore neighbors (4 cardinal directions)
        for neighbor in [(current[0] + 1, current[1]), (current[0] - 1, current[1]),
                         (current[0], current[1] + 1), (current[0], current[1] - 1)]:
            if neighbor in closed_list or neighbor in planned_paths:
                continue

            tentative_g_score = g_score.get(current, float('inf')) + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + \
                    heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None  # Return None if no path is found


def check_collision(path, planned_paths):
    """
    Check if the given path collides with any previously planned path.
    Returns True if there is a collision, False otherwise.
    """
    for p in path:
        if p in planned_paths:
            return True
    return False

# ================================
# Continuous Path Planning
# ================================


def continuous_path_planning(agents_plans):
    """
    Main function to handle continuous path planning for multiple agents. 
    Plans paths for each agent while avoiding conflicts.
    """
    tasks = []
    result_solution = {}
    step_time = 0
    planned_paths = set()  # To store already planned paths (as a set for fast lookups)

    # Initialize tasks for each agent
    for agent_name, plans in agents_plans.items():
        result_solution[agent_name] = []
        if plans:
            task = plans.pop(0)
            tasks.append((task["time"], agent_name, task))

    while tasks:
        tasks.sort()  # Sort tasks by their scheduled start time
        task_time, agent_name, task = tasks.pop(0)

        if task["time"] != step_time:
            continue  # Skip this task if its start time is not the current step time

        # Plan the path for the current task using A* search
        solution = a_star_search(task["start"], task["goal"], planned_paths)

        if solution is None:  # Path planning failed
            task["time"] = step_time + 1  # Retry this task after 1 time unit
            result_solution[agent_name].append(
                {"time": step_time, "start": task["start"], "goal": task["goal"]})
            tasks.append((task["time"], agent_name, task))
        else:
            # If planning is successful, add the path to the planned paths
            path = solution[0]["path"]
            if check_collision(path, planned_paths):  # If there is a collision
                # Retry this task after 1 time unit
                task["time"] = step_time + 1
                result_solution[agent_name].append(
                    {"time": step_time, "start": task["start"], "goal": task["goal"]})
                tasks.append((task["time"], agent_name, task))
            else:
                # Add the new path to the set of planned paths
                planned_paths.update(path)
                result_solution[agent_name].append(solution)
                if agents_plans[agent_name]:  # If the agent has more tasks
                    next_task = agents_plans[agent_name].pop(0)
                    # Update the start time of the next task
                    next_task["time"] = solution[-1]["time"] + 1
                    tasks.append((next_task["time"], agent_name, next_task))

        # Increment the global time step after processing all tasks for this time step
        step_time += 1

    return result_solution  # Return the final conflict-free paths for all agents

# ================================
# Testing the Module with Example
# ================================


if __name__ == "__main__":
    # Example of task lists for multiple agents
    agents_plans = {
        "agent1": [
            {"start": (0, 0), "goal": (5, 5), "time": 0},
            {"start": (5, 5), "goal": (10, 10), "time": 10}
        ],
        "agent2": [
            {"start": (1, 0), "goal": (5, 5), "time": 0},
            {"start": (5, 5), "goal": (0, 0), "time": 10}
        ]
    }

    # Call the continuous path planning function
    result = continuous_path_planning(agents_plans)

    # Print the result (conflict-free paths for each agent)
    print("Global Conflict-Free Paths:")
    for agent_name, solution in result.items():
        print(f"\n{agent_name}:")
        for task_solution in solution:
            print(
                f"  Task (Start: {task_solution[0]['start']}, Goal: {task_solution[0]['goal']}):")
            for step in task_solution[0]["path"]:
                print(f"    {step}")
