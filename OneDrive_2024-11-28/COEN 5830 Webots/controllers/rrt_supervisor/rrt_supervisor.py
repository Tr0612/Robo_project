from controller import Supervisor, Emitter, Receiver
import numpy as np
import struct
import time
import math

class Node:
    """Class to represent a node in the RRT tree."""
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return np.linalg.norm(np.array(p1) - np.array(p2))

def randomly_sample_state(goal, arena_bounds, goal_bias=0.9):
    """Randomly sample a state with a goal bias."""
    if np.random.random() < goal_bias:
        return np.array(goal)
    else:
        return np.array([
            np.random.uniform(arena_bounds[0], arena_bounds[1]),
            np.random.uniform(arena_bounds[2], arena_bounds[3])
        ])

def nearest_node(tree, random_position):
    """Find the nearest node in the tree to the random position."""
    return min(tree, key=lambda node: distance(node.position, random_position))

def steer(from_node, to_position, step_size):
    """Move from 'from_node' towards 'to_position' by a step size."""
    direction = np.array(to_position) - np.array(from_node.position)
    norm = np.linalg.norm(direction)
    if norm == 0:
        return from_node.position
    direction = direction / norm
    new_position = np.array(from_node.position) + step_size * direction
    return new_position

def is_within_bounds(position, arena_bounds):
    """Check if a position is within the defined arena bounds."""
    return (arena_bounds[0] <= position[0] <= arena_bounds[1] and
            arena_bounds[2] <= position[1] <= arena_bounds[3])

def construct_path(goal_node):
    """Construct the path by tracing back from the goal node."""
    path = []
    current_node = goal_node
    while current_node:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]  # Reverse the path

def rrt_planning(supervisor, emitter, receiver, start, goal, arena_bounds, step_size, max_iterations):
    """Perform RRT planning from start to goal."""
    tree = [Node(start)]
    for _ in range(max_iterations):
        random_position = randomly_sample_state(goal, arena_bounds)
        nearest = nearest_node(tree, random_position)
        new_position = steer(nearest, random_position, step_size)

        if is_within_bounds(new_position, arena_bounds):
            # Send propagation command to the controller
            command = struct.pack('f', 1.0)
            emitter.send(command)

            # Wait for propagation result
            while receiver.getQueueLength() == 0:
                supervisor.step(int(supervisor.getBasicTimeStep()))

            data = receiver.getBytes()
            receiver.nextPacket()
            success, left_speed, right_speed = struct.unpack('fff', data)

            if success:
                new_node = Node(new_position, nearest)
                tree.append(new_node)

                if distance(new_position, goal) <= 0.1:  # Goal region radius
                    return construct_path(new_node)  # Path found

    return None  # Path not found

def reset_and_replay(supervisor, path):
    """Reset the simulation and replay the series of propagations to reach the goal."""
    puck = supervisor.getFromDef("e-puck")
    puck.getField("translation").setSFVec3f([0, 0, 0])
    supervisor.simulationResetPhysics()

    for waypoint in path:
        puck.getField("translation").setSFVec3f([waypoint[0], waypoint[1], 0])
        supervisor.step(int(supervisor.getBasicTimeStep()))
        time.sleep(0.5)  # Simulate movement time

def main():
    supervisor = Supervisor()
    puck = supervisor.getFromDef("e-puck")
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    emitter.setChannel(10)
    receiver.setChannel(10)
    receiver.enable(int(supervisor.getBasicTimeStep()))

    # RRT parameters
    arena_bounds = [-1, 1, -1, 1]  # [x_min, x_max, y_min, y_max]
    start = [0, 0]
    goal = [0.6, 0.6]
    step_size = 0.1
    max_iterations = 1000

    # Plan path using RRT
    path = rrt_planning(supervisor, emitter, receiver, start, goal, arena_bounds, step_size, max_iterations)

    if path is None:
        print("Path not found.")
        return

    print("Path found:", path)

    # Reset and replay the solution path
    reset_and_replay(supervisor, path)

if __name__ == "__main__":
    main()
