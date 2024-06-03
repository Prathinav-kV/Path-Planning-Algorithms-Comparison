import numpy as np
import matplotlib.pyplot as plt
import random
import time
from scipy.spatial import distance as sp_dist

# Define a global clearance distance
clearence= 5

class Vertex:
    def __init__(self, pos_x, pos_y):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.predecessor = None
        self.expense = 0  

def is_valid_position(pos_x, pos_y, rad):
    dist = rad + clearence
    within_boundaries = dist <= pos_x <= 600 - dist and dist <= pos_y <= 200 - dist
    outside_rect1 = not (150 - dist <= pos_x <= 175 + dist and pos_y >= 100 - dist)
    outside_rect2 = not (250 - dist <= pos_x <= 275 + dist and pos_y <= 100 + dist)
    dist_to_circle_center = ((pos_x - 420)**2 + (pos_y - 120)**2) ** 0.5
    outside_circle = dist_to_circle_center > (60 + dist)
    return within_boundaries and outside_rect1 and outside_rect2 and outside_circle

def move_towards(from_vertex, to_vertex, max_step):
    if sp_dist.euclidean((from_vertex.pos_x, from_vertex.pos_y), (to_vertex.pos_x, to_vertex.pos_y)) < max_step:
        return to_vertex
    else:
        angle = np.arctan2(to_vertex.pos_y - from_vertex.pos_y, to_vertex.pos_x - from_vertex.pos_x)
        return Vertex(from_vertex.pos_x + max_step * np.cos(angle), from_vertex.pos_y + max_step * np.sin(angle))

def is_valid_segment(point_a, point_b):
    segment_steps = int(sp_dist.euclidean((point_a.pos_x, point_a.pos_y), (point_b.pos_x, point_b.pos_y)) / (clearence / 2))
    for step in range(1, segment_steps + 1):
        t = step / float(segment_steps)
        interim_x = point_a.pos_x * (1 - t) + point_b.pos_x * t
        interim_y = point_a.pos_y * (1 - t) + point_b.pos_y * t
        if not is_valid_position(interim_x, interim_y, 0):
            return False
    return True

def expand_tree(tree, random_vertex, max_step):
    closest_vertex = min(tree, key=lambda vertex: sp_dist.euclidean((vertex.pos_x, vertex.pos_y), (random_vertex.pos_x, random_vertex.pos_y)))
    new_vertex = move_towards(closest_vertex, random_vertex, max_step)
    if new_vertex and is_valid_segment(closest_vertex, new_vertex):
        new_vertex.predecessor = closest_vertex
        new_vertex.expense = closest_vertex.expense + sp_dist.euclidean((closest_vertex.pos_x, closest_vertex.pos_y), (new_vertex.pos_x, new_vertex.pos_y))
        tree.append(new_vertex)
        return new_vertex
    return None

def construct_path(vertex_from_tree_a, vertex_from_tree_b):
    """ Constructs the complete path by linking two vertices from separate trees. """
    full_path = []
    # Path from tree A
    vertex = vertex_from_tree_a
    while vertex:
        full_path.append(vertex)
        vertex = vertex.predecessor
    full_path.reverse()
    
    # Path from tree B
    vertex = vertex_from_tree_b
    while vertex:
        full_path.append(vertex)
        vertex = vertex.predecessor
    
    return full_path

def draw_path(vertices, route, title="Route"):
    fig, ax = plt.subplots(figsize=(12, 6))
    # Plot vertices
    for vertex in vertices:
        if vertex.predecessor:
            plt.plot([vertex.pos_x, vertex.predecessor.pos_x], [vertex.pos_y, vertex.predecessor.pos_y], "y-")
    # Highlight the route
    for vertex in route:
        if vertex.predecessor:
            plt.plot([vertex.pos_x, vertex.predecessor.pos_x], [vertex.pos_y, vertex.predecessor.pos_y], "r-")
        plt.scatter(vertex.pos_x, vertex.pos_y, color="none")

    # Draw obstacles
    rect1 = plt.Rectangle((150, 100), 25, 100, color="grey")
    rect2 = plt.Rectangle((250, 0), 25, 100, color="grey")
    circle = plt.Circle((420, 120), 60, color="grey", fill=True)

    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(circle)

    plt.xlim(0, 600)
    plt.ylim(0, 200)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title(title)
    plt.show()

def draw_rrt_connect(Tree_A, Tree_B, route, title="RRT-Connect Route"):
    fig, ax = plt.subplots(figsize=(12, 6))
    # Plot Tree A
    for vertex in Tree_A:
        if vertex.predecessor:
            plt.plot([vertex.pos_x, vertex.predecessor.pos_x], [vertex.pos_y, vertex.predecessor.pos_y], "g-")
    # Plot Tree B
    for vertex in Tree_B:
        if vertex.predecessor:
            plt.plot([vertex.pos_x, vertex.predecessor.pos_x], [vertex.pos_y, vertex.predecessor.pos_y], "b-")
    # Highlight the route
    if route:
        for vertex in route:
            if vertex.predecessor:
                plt.plot([vertex.pos_x, vertex.predecessor.pos_x], [vertex.pos_y, vertex.predecessor.pos_y], "r-")
            plt.scatter(vertex.pos_x, vertex.pos_y, color="none")

    # Draw obstacles
    rect1 = plt.Rectangle((150, 100), 25, 100, color="grey")
    rect2 = plt.Rectangle((250, 0), 25, 100, color="grey")
    circle = plt.Circle((420, 120), 60, color="grey", fill=True)

    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(circle)

    plt.xlim(0, 600)
    plt.ylim(0, 200)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title(title)
    plt.show()

# RRT Algorithm
def rrt_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations):
    start_vertex = Vertex(start_pos_x, start_pos_y)
    target_vertex = Vertex(target_pos_x, target_pos_y)
    vertices = [start_vertex]

    for _ in range(max_iterations):
        rand_pos_x = random.uniform(0, 600)
        rand_pos_y = random.uniform(0, 200)
        rand_vertex = Vertex(rand_pos_x, rand_pos_y)

        closest_vertex = min(vertices, key=lambda vertex: sp_dist.euclidean((vertex.pos_x, vertex.pos_y), (rand_vertex.pos_x, rand_vertex.pos_y)))
        new_vertex = move_towards(closest_vertex, rand_vertex, max_step)

        if is_valid_position(new_vertex.pos_x, new_vertex.pos_y, 0) and is_valid_segment(closest_vertex, new_vertex):
            new_vertex.predecessor = closest_vertex
            vertices.append(new_vertex)

            if sp_dist.euclidean((new_vertex.pos_x, new_vertex.pos_y), (target_vertex.pos_x, target_vertex.pos_y)) <= max_step:
                target_vertex.predecessor = new_vertex
                vertices.append(target_vertex)
                break

    route = []
    last_vertex = target_vertex
    while last_vertex.predecessor is not None:
        route.append(last_vertex)
        last_vertex = last_vertex.predecessor
    route.append(last_vertex)

    return route[::-1], vertices

# RRT-Connect Algorithm
def rrt_connect_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations):
    start_vertex = Vertex(start_pos_x, start_pos_y)
    target_vertex = Vertex(target_pos_x, target_pos_y)
    Tree_A = [start_vertex]
    Tree_B = [target_vertex]

    for _ in range(max_iterations):
        rand_vertex = Vertex(random.uniform(0, 600), random.uniform(0, 200))
        new_vertex_a = expand_tree(Tree_A, rand_vertex, max_step)
        if new_vertex_a:
            new_vertex_b = expand_tree(Tree_B, new_vertex_a, max_step)
            if new_vertex_b and is_valid_segment(new_vertex_a, new_vertex_b):
                # Connect if vertices are close
                if sp_dist.euclidean((new_vertex_a.pos_x, new_vertex_a.pos_y), (new_vertex_b.pos_x, new_vertex_b.pos_y)) <= max_step:
                    # Construct the full path by linking nodes from both trees
                    return construct_path(new_vertex_a, new_vertex_b), Tree_A + Tree_B
        # Swap trees to alternate direction
        Tree_A, Tree_B = Tree_B, Tree_A

    return None, Tree_A + Tree_B

def informed_rrt_star_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations):
    start_vertex = Vertex(start_pos_x, start_pos_y)
    target_vertex = Vertex(target_pos_x, target_pos_y)
    vertices = [start_vertex]

    for _ in range(max_iterations):
        if random.random() < 0.7:  # Increase ellipsoid sampling
            center = np.array([(start_pos_x + target_pos_x) / 2, (start_pos_y + target_pos_y) / 2])
            vector = np.array([target_pos_x - start_pos_x, target_pos_y - start_pos_y])
            distance = np.linalg.norm(vector)
            if distance > 0:
                vector = vector / distance
            else:
                vector = np.array([1.0, 0.0])
            angle = random.uniform(0, 2 * np.pi)
            radius = distance / 2
            rand = np.random.uniform(0, radius)
            rand_pos_x = center[0] + rand * np.cos(angle) * vector[0] - rand * np.sin(angle) * vector[1]
            rand_pos_y = center[1] + rand * np.cos(angle) * vector[1] + rand * np.sin(angle) * vector[0]
        else:
            rand_pos_x = random.uniform(0, 600)
            rand_pos_y = random.uniform(0, 200)

        rand_vertex = Vertex(rand_pos_x, rand_pos_y)

        closest_vertex = min(vertices, key=lambda vertex: sp_dist.euclidean((vertex.pos_x, vertex.pos_y), (rand_vertex.pos_x, rand_vertex.pos_y)))
        new_vertex = move_towards(closest_vertex, rand_vertex, max_step)

        if is_valid_position(new_vertex.pos_x, new_vertex.pos_y, 0) and is_valid_segment(closest_vertex, new_vertex):
            new_vertex.predecessor = closest_vertex
            new_vertex.expense = closest_vertex.expense + sp_dist.euclidean((closest_vertex.pos_x, closest_vertex.pos_y), (new_vertex.pos_x, new_vertex.pos_y))
            vertices.append(new_vertex)

            # Attempt to connect to the goal if within step size
            if sp_dist.euclidean((new_vertex.pos_x, new_vertex.pos_y), (target_vertex.pos_x, target_vertex.pos_y)) <= max_step:
                if is_valid_segment(new_vertex, target_vertex):
                    target_vertex.predecessor = new_vertex
                    target_vertex.expense = new_vertex.expense + sp_dist.euclidean((new_vertex.pos_x, new_vertex.pos_y), (target_vertex.pos_x, target_vertex.pos_y))
                    vertices.append(target_vertex)
                    break

    route = []
    last_vertex = target_vertex
    while last_vertex.predecessor is not None:
        route.append(last_vertex)
        last_vertex = last_vertex.predecessor
    route.append(last_vertex)

    return route[::-1], vertices

# EP-RRT Algorithm
def ep_rrt_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations, exploration_bias=0.5):
    start_vertex = Vertex(start_pos_x, start_pos_y)
    target_vertex = Vertex(target_pos_x, target_pos_y)
    vertices = [start_vertex]

    for _ in range(max_iterations):
        if random.random() < exploration_bias:
            # Exploration phase: random point in the environment
            rand_pos_x = random.uniform(0, 600)
            rand_pos_y = random.uniform(0, 200)
        else:
            # Exploitation phase: move towards goal
            rand_pos_x = target_pos_x
            rand_pos_y = target_pos_y

        rand_vertex = Vertex(rand_pos_x, rand_pos_y)

        closest_vertex = min(vertices, key=lambda vertex: sp_dist.euclidean((vertex.pos_x, vertex.pos_y), (rand_vertex.pos_x, rand_vertex.pos_y)))
        new_vertex = move_towards(closest_vertex, rand_vertex, max_step)

        if is_valid_position(new_vertex.pos_x, new_vertex.pos_y, 0) and is_valid_segment(closest_vertex, new_vertex):
            new_vertex.predecessor = closest_vertex
            vertices.append(new_vertex)

            if sp_dist.euclidean((new_vertex.pos_x, new_vertex.pos_y), (target_vertex.pos_x, target_vertex.pos_y)) <= max_step:
                target_vertex.predecessor = new_vertex
                vertices.append(target_vertex)
                break

    route = []
    last_vertex = target_vertex
    while last_vertex.predecessor is not None:
        route.append(last_vertex)
        last_vertex = last_vertex.predecessor
    route.append(last_vertex)

    return route[::-1], vertices

# User input for start, goal coordinates, step size, and number of iterations
start_pos_x = int(input("Enter start x-coordinate (0-600): "))
start_pos_y = int(input("Enter start y-coordinate (0-200): "))
target_pos_x = int(input("Enter goal x-coordinate (0-600): "))
target_pos_y = int(input("Enter goal y-coordinate (0-200): "))
max_step = int(input("Enter step size: "))
max_iterations = int(input("Enter number of iterations: "))

# Execute and plot each RRT variant
start_t = time.time()
route_rrt, vertices_rrt = rrt_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations)
end_t = time.time()
print(f"RRT completed in {end_t - start_t:.2f} seconds and found route of length {len(route_rrt)}")
draw_path(vertices_rrt, route_rrt, "RRT Path")

# RRT-Connect
start_t = time.time()
route_rrt_connect, vertices_rrt_connect = rrt_connect_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations)
end_t = time.time()
print(f"RRT-Connect completed in {end_t - start_t:.2f} seconds and found route of length {len(route_rrt_connect)}")
draw_rrt_connect(vertices_rrt_connect[:len(vertices_rrt_connect)//2], vertices_rrt_connect[len(vertices_rrt_connect)//2:], route_rrt_connect, "RRT-Connect Path")

# Informed RRT*
start_t = time.time()
route_informed_rrt_star, vertices_informed_rrt_star = informed_rrt_star_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations)
end_t = time.time()
print(f"Informed RRT* completed in {end_t - start_t:.2f} seconds and found route of length {len(route_informed_rrt_star)}")
draw_path(vertices_informed_rrt_star, route_informed_rrt_star, "Informed RRT* Path")

# EP-RRT
start_t = time.time()
route_ep_rrt, vertices_ep_rrt = ep_rrt_algo(start_pos_x, start_pos_y, target_pos_x, target_pos_y, max_step, max_iterations)
end_t = time.time()
print(f"EP-RRT completed in {end_t - start_t:.2f} seconds and found route of length {len(route_ep_rrt)}")
draw_path(vertices_ep_rrt, route_ep_rrt, "EP-RRT Path")
