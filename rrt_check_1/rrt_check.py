from controller import Supervisor, Emitter, Receiver
import numpy as np
import random

# Initialize Supervisor and Robot
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# E-puck robot references
robot = supervisor.getFromDef("e-puck")
translation = robot.getField("translation")
rotation = robot.getField("rotation")

# Communication setup
emitter_sup = supervisor.getDevice("emitter")
receiver_sup = supervisor.getDevice("receiver")
emitter_rob = supervisor.getDevice("emitter_puck")
receiver_rob = supervisor.getDevice("receiver_puck")

receiver_sup.enable(timestep)
# receiver_rob.enable(timestep)

# Constants
arena_bounds = [0, 1.0, 0, 1.0]  # Arena dimensions (x_min, x_max, y_min, y_max)
goal = np.array([0.6, 0.6, 0])  # Goal state
goal_radius = 0.1
propagation_count = 5

# Helper Functions
def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def sample_point(goal_bias=0.5):
    """Randomly sample a point with goal bias."""
    if random.random() < goal_bias:
        return goal
    else:
        return [
            random.uniform(arena_bounds[0], arena_bounds[1]),
            random.uniform(arena_bounds[2], arena_bounds[3]),
            0,
        ]

def monte_carlo_propagation(current_state, target_state):
    """Perform Monte Carlo propagation to find the best move."""
    best_propagation = None
    min_distance = float("inf")
    
    for _ in range(propagation_count):
        # Generate random actions and duration
        random_action = np.random.uniform(-1.0, 1.0, size=2)  # Linear and angular speeds
        random_duration = np.random.uniform(0.1, 0.5)  # Duration of action
        
        # Simulate propagation
        new_state = current_state + random_action * random_duration
        
        # Collision checking
        if (
            arena_bounds[0] <= new_state[0] <= arena_bounds[1]
            and arena_bounds[2] <= new_state[1] <= arena_bounds[3]
        ):
            dist = distance(new_state, target_state)
            if dist < min_distance:
                best_propagation = (new_state, random_action, random_duration)
                min_distance = dist

    return best_propagation

def is_goal_reached(node):
    """Check if the goal is reached."""
    return distance(node, goal) <= goal_radius

# RRT Initialization
rrt_tree = [{"state": translation.getSFVec3f(), "parent": None, "action": None}]

# Main Loop
while supervisor.step(timestep) != -1:
    current_state = rrt_tree[-1]["state"]

    # Sample a new point
    sampled_point = sample_point()

    # Find nearest node
    nearest_node = min(rrt_tree, key=lambda node: distance(node["state"], sampled_point))

    # Perform Monte Carlo propagation
    propagation = monte_carlo_propagation(nearest_node["state"], sampled_point)
    if propagation is None:
        continue

    new_state, action, duration = propagation

    # Add new node to the tree
    rrt_tree.append({"state": new_state, "parent": nearest_node, "action": action})

    # Check if goal is reached
    if is_goal_reached(new_state):
        print("Goal reached!")
        break

# Deploy the Path (Bonus)
path = []
node = rrt_tree[-1]
while node["parent"] is not None:
    path.insert(0, node)
    node = node["parent"]

print("Path found! Deploying...")
for step in path:
    action = step["action"]
    emitter_sup.send(action.tobytes())  # Send action to robot
    supervisor.step(int(step["duration"] * 1000))  # Execute for the given duration

print("Task completed.")
