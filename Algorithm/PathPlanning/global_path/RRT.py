import numpy as np
from shapely.geometry import Point, LineString
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as pltPolygon
from shapely.geometry import Point, LineString, Polygon


def RRT(start, goal, terrain_polygons, max_iter=5000, step_size=1, goal_sample_rate=5):
    # validate and normalise the input types
    if not isinstance(start, tuple):
        if isinstance(start, (list, np.ndarray)):
            start = tuple(start)
        else:
            print(
                f"[RRT] Warning: Converting start from {type(start)} to tuple")
            start = tuple(start) if hasattr(start, '__iter__') else start

    if not isinstance(goal, tuple):
        if isinstance(goal, (list, np.ndarray)):
            goal = tuple(goal)
        else:
            print(f"[RRT] Warning: Converting goal from {type(goal)} to tuple")
            goal = tuple(goal) if hasattr(goal, '__iter__') else goal

    class Node:
        def __init__(self, point, parent=None):
            # normalise point to a tuple
            try:
                if isinstance(point, tuple):
                    self.point = point
                elif isinstance(point, (list, np.ndarray)):
                    point_array = np.array(point).flatten()
                    if len(point_array) >= 2:
                        self.point = (
                            float(point_array[0]), float(point_array[1]))
                    else:
                        print(
                            f"[RRT] Warning: Point has less than 2 dimensions: {point}")
                        self.point = point
                else:
                    # any other type: attempt a conversion
                    try:
                        point_array = np.array(point).flatten()
                        if len(point_array) >= 2:
                            self.point = (
                                float(point_array[0]), float(point_array[1]))
                        else:
                            print(
                                f"[RRT] Warning: Invalid point format: {point}")
                            self.point = point
                    except Exception:
                        print(
                            f"[RRT] Warning: Cannot convert point to tuple: {point}, type: {type(point)}")
                        self.point = point
                self.parent = parent
            except Exception as e:
                print(f"[RRT] Error initializing Node: {e}")
                self.point = point
                self.parent = parent

    def distance(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def nearest_node(nodes, random_point):
        try:
            # keep only the well-formed nodes
            valid_nodes = []
            for node in nodes:
                if hasattr(node, 'point') and node.point is not None:
                    if isinstance(node.point, (tuple, list, np.ndarray)):
                        try:
                            point_array = np.array(node.point)
                            if point_array.shape == (2,):
                                valid_nodes.append(node)
                            else:
                                print(
                                    f"[RRT] Skipping node with invalid point shape: {point_array.shape}")
                        except Exception:
                            print(
                                f"[RRT] Skipping node with invalid point: {node.point}")
                    else:
                        print(
                            f"[RRT] Skipping node with invalid point type: {type(node.point)}")
                elif isinstance(node, tuple):
                    # convert tuples to Node objects
                    valid_nodes.append(Node(node))
                else:
                    print(f"[RRT] Skipping invalid node type: {type(node)}")

            if not valid_nodes:
                print(f"[RRT] Error: No valid nodes found!")
                return nodes[0] if nodes else None

            points = np.array([node.point for node in valid_nodes])
            random_point = np.array(random_point)
            distances = np.linalg.norm(points - random_point, axis=1)
            return valid_nodes[np.argmin(distances)]
        except Exception as e:
            # never let a malformed node stop the tree from growing
            print(f"[RRT] Error in nearest_node: {e}")
            print(f"[RRT] Number of nodes: {len(nodes)}")
            if nodes:
                print(f"[RRT] First node type: {type(nodes[0])}")
                if hasattr(nodes[0], 'point'):
                    print(
                        f"[RRT] First node.point: {nodes[0].point}, type: {type(nodes[0].point)}")
            # fall back to the first node
            return nodes[0] if nodes else None

    def steer(from_node, to_point, step_size):
        if distance(from_node.point, to_point) < step_size:
            return to_point
        else:
            from_point = np.array(from_node.point)
            to_point = np.array(to_point)
            direction = (to_point - from_point) / \
                np.linalg.norm(to_point - from_point)
            new_point = from_point + step_size * direction
            return tuple(new_point)

    def line_collision_free(from_point, to_point, polygons):
        line = LineString([from_point, to_point])
        for poly in polygons:
            if poly.intersects(line):
                return False
        return True

    def generate_random_point(x_bounds, y_bounds):
        # Increased chance to sample the goal for faster convergence
        if np.random.random() < goal_sample_rate / 100:
            return goal

        # Biased sampling - sample more points near start and goal
        if np.random.random() < 0.4:  # 40% chance for biased sampling
            # Choose either start or goal to bias towards
            if np.random.random() < 0.5:
                bias_point = start
            else:
                bias_point = goal

            # Sample in a circle around the bias point
            radius = np.random.uniform(0, 30)  # Adjust radius as needed
            angle = np.random.uniform(0, 2 * np.pi)
            x = bias_point[0] + radius * np.cos(angle)
            y = bias_point[1] + radius * np.sin(angle)
            return (x, y)
        else:
            # Regular uniform sampling
            x = np.random.uniform(x_bounds[0], x_bounds[1])
            y = np.random.uniform(y_bounds[0], y_bounds[1])
        return (x, y)

    # Expanded search space
    dist_between = distance(start, goal)
    # At least 30 units or 30% of the distance
    buffer = max(30, dist_between * 0.3)

    x_bounds = [min(start[0], goal[0]) - buffer,
                max(start[0], goal[0]) + buffer]
    y_bounds = [min(start[1], goal[1]) - buffer,
                max(start[1], goal[1]) + buffer]
    root = Node(start)
    nodes = [root]

    # Convert terrain_polygons to shapely Polygon objects if they aren't already
    polygons = []
    for p in terrain_polygons:
        if isinstance(p, Polygon):
            polygons.append(p)
        else:
            polygons.append(Polygon(p))

    for i in range(max_iter):
        random_point = generate_random_point(x_bounds, y_bounds)
        nearest = nearest_node(nodes, random_point)
        new_point = steer(nearest, random_point, step_size)

        if line_collision_free(nearest.point, new_point, polygons):
            new_node = Node(new_point, nearest)
            nodes.append(new_node)

            # can we connect straight to the goal? (with a widened tolerance)
            if distance(new_point, goal) <= step_size * 3:
                if line_collision_free(new_point, goal, polygons):
                    final_node = Node(goal, new_node)
                    return reconstruct_path(final_node)

        # check for early termination every 50 iterations
        if i % 50 == 0 and len(nodes) > 30:
            # Check if any node is close to goal
            check_count = min(30, len(nodes))  # test the 30 most recent nodes
            for node in nodes[-check_count:]:
                if distance(node.point, goal) <= step_size * 4:
                    if line_collision_free(node.point, goal, polygons):
                        final_node = Node(goal, node)
                        return reconstruct_path(final_node)

    # If no path found but we have nodes, return the path to the closest node to goal
    if nodes:
        distances_to_goal = [distance(node.point, goal) for node in nodes]
        closest_index = np.argmin(distances_to_goal)
        if line_collision_free(nodes[closest_index].point, goal, polygons):
            final_node = Node(goal, nodes[closest_index])
            return reconstruct_path(final_node)
        else:
            return reconstruct_path(nodes[closest_index])

    return []  # Return empty if no path is found


def reconstruct_path(node):
    path = []
    while node is not None:
        path.append(node.point)
        node = node.parent
    path.reverse()
    return path
