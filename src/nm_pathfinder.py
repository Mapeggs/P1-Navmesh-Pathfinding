# def find_path (source_point, destination_point, mesh):

#     """
#     Searches for a path from source_point to destination_point through the mesh

#     Args:
#         source_point: starting point of the pathfinder
#         destination_point: the ultimate goal the pathfinder must reach
#         mesh: pathway constraints the path adheres to

#     Returns:

#         A path (list of points) from source_point to destination_point if exists
#         A list of boxes explored by the algorithm
#     """

#     path = []
#     boxes = {}

#     return path, boxes.keys()


# from training claude ai to help:

# This implementation includes the following key features:

# 1. Bidirectional A* search using a single priority queue
# 2. Helper functions for:
#    - Calculating Euclidean distance
#    - Point-in-box testing
#    - Point constraining to box boundaries
#    - Finding boxes containing points
#    - Path reconstruction

# The algorithm:
# 1. Identifies source and destination boxes
# 2. Handles special cases (no path, degenerate path)
# 3. Maintains separate forward and backward search data structures
# 4. Uses A* heuristic (Euclidean distance) for both search directions
# 5. Terminates when the searches meet
# 6. Returns both the path and visited boxes for visualization

# The implementation follows the workflow suggested in the assignment:
# - Identifies source and destination boxes
# - Implements complete search (will find a path if one exists)
# - Uses detail points for precise path construction
# - Implements bidirectional A* with proper heuristics



from math import sqrt
from heapq import heappush, heappop

def distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def point_in_box(point, box):
    """Check if a point is inside a box"""
    x, y = point
    x1, x2, y1, y2 = box
    return x1 <= x <= x2 and y1 <= y <= y2

def constrain_point(point, box):
    """Constrain a point to lie within a box"""
    x, y = point
    x1, x2, y1, y2 = box
    return (max(x1, min(x2, x)), max(y1, min(y2, y)))

def find_box_containing_point(point, mesh):
    """Find the box in the mesh that contains the given point"""
    for box in mesh['boxes']:
        if point_in_box(point, box):
            return box
    return None

def reconstruct_path(meeting_box, forward_prev, backward_prev, detail_points):
    """Reconstruct the complete path from source to destination"""
    # Build forward path
    forward_path = []
    current = meeting_box
    while current in forward_prev:
        forward_path.append(detail_points[current])
        current = forward_prev[current]
    forward_path.append(detail_points[current])
    forward_path.reverse()
    
    # Build backward path
    backward_path = []
    current = meeting_box
    while current in backward_prev:
        backward_path.append(detail_points[current])
        current = backward_prev[current]
    backward_path.append(detail_points[current])
    
    # Combine paths
    return forward_path + backward_path[1:]

def find_path(source_point, destination_point, mesh):
    """Find a path from source_point to destination_point through the mesh"""
    
    # Find source and destination boxes
    source_box = find_box_containing_point(source_point, mesh)
    dest_box = find_box_containing_point(destination_point, mesh)
    
    if not source_box or not dest_box:
        print("No path!")
        return [], []
        
    if source_box == dest_box:
        return [source_point, destination_point], [source_box]
    
    # Initialize data structures for bidirectional search
    detail_points = {
        source_box: source_point,
        dest_box: destination_point
    }
    
    forward_prev = {}   # Previous box in path from source
    backward_prev = {}  # Previous box in path from destination
    
    forward_dist = {source_box: 0}   # Distance from source
    backward_dist = {dest_box: 0}    # Distance from destination
    
    queue = []  # Priority queue
    visited = set()  # Track visited boxes for visualization
    
    # Add starting points to queue with initial priorities
    heappush(queue, (0, source_box, 'destination'))  # Forward search
    heappush(queue, (0, dest_box, 'source'))        # Backward search
    
    while queue:
        priority, current_box, goal = heappop(queue)
        visited.add(current_box)
        
        # Check if we've found a meeting point
        if (goal == 'destination' and current_box in backward_prev) or \
           (goal == 'source' and current_box in forward_prev):
            path = reconstruct_path(
                current_box,
                forward_prev,
                backward_prev,
                detail_points
            )
            return path, list(visited)
            
        # Choose appropriate distance and previous dictionaries based on search direction
        if goal == 'destination':
            dist_map = forward_dist
            prev_map = forward_prev
            target_point = destination_point
        else:
            dist_map = backward_dist
            prev_map = backward_prev
            target_point = source_point
            
        current_dist = dist_map[current_box]
            
        # Explore neighbors
        for neighbor in mesh['adj'][current_box]:
            # Get point of entry into neighbor box
            entry_point = constrain_point(detail_points[current_box], neighbor)
            
            # Calculate new distance
            edge_cost = distance(detail_points[current_box], entry_point)
            new_dist = current_dist + edge_cost
            
            if neighbor not in dist_map or new_dist < dist_map[neighbor]:
                detail_points[neighbor] = entry_point
                dist_map[neighbor] = new_dist
                prev_map[neighbor] = current_box
                
                # Calculate heuristic (straight-line distance to goal)
                h = distance(entry_point, target_point)
                priority = new_dist + h
                
                heappush(queue, (priority, neighbor, goal))
                
    print("No path!")
    return [], list(visited)

