# # def find_path (source_point, destination_point, mesh):

# #     """
# #     Searches for a path from source_point to destination_point through the mesh

# #     Args:
# #         source_point: starting point of the pathfinder
# #         destination_point: the ultimate goal the pathfinder must reach
# #         mesh: pathway constraints the path adheres to

# #     Returns:

# #         A path (list of points) from source_point to destination_point if exists
# #         A list of boxes explored by the algorithm
# #     """

# #     path = []
# #     boxes = {}

# #     return path, boxes.keys()


# # from training claude ai to help:

# # This implementation includes the following key features:

# # 1. Bidirectional A* search using a single priority queue
# # 2. Helper functions for:
# #    - Calculating Euclidean distance
# #    - Point-in-box testing
# #    - Point constraining to box boundaries
# #    - Finding boxes containing points
# #    - Path reconstruction

# # The algorithm:
# # 1. Identifies source and destination boxes
# # 2. Handles special cases (no path, degenerate path)
# # 3. Maintains separate forward and backward search data structures
# # 4. Uses A* heuristic (Euclidean distance) for both search directions
# # 5. Terminates when the searches meet
# # 6. Returns both the path and visited boxes for visualization

# # The implementation follows the workflow suggested in the assignment:
# # - Identifies source and destination boxes
# # - Implements complete search (will find a path if one exists)
# # - Uses detail points for precise path construction
# # - Implements bidirectional A* with proper heuristics


from math import sqrt
from heapq import heappush, heappop

def distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def find_box_containing_point(point, mesh):
    """Find the box that contains the given point"""
    x, y = point
    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if x1 <= x <= x2 and y1 <= y <= y2:
            return box
    return None

def find_path(source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh
    using bidirectional A* search.

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: contains 'boxes' and 'adj' - boxes are traversable spaces, adj shows valid connections

    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    
    # Find source and destination boxes
    source_box = find_box_containing_point(source_point, mesh)
    dest_box = find_box_containing_point(destination_point, mesh)
    
    if not source_box or not dest_box:
        print("No path!")
        return [], []
        
    if source_box == dest_box:
        return [source_point, destination_point], [source_box]
    
    # Initialize detail points for each box we visit
    detail_points = {
        source_box: source_point,
        dest_box: destination_point
    }
    
    # Bidirectional search data structures
    forward_dist = {source_box: 0}
    backward_dist = {dest_box: 0}
    forward_prev = {}
    backward_prev = {}
    visited = set()
    
    # Priority queue entries are (priority, box, direction)
    # where direction is either 'forward' or 'backward'
    queue = []
    heappush(queue, (0, source_box, 'forward'))
    heappush(queue, (0, dest_box, 'backward'))
    
    while queue:
        _, current_box, direction = heappop(queue)
        visited.add(current_box)
        
        # Check if the searches have met
        if ((direction == 'forward' and current_box in backward_prev) or
            (direction == 'backward' and current_box in forward_prev)):
            # Build path from both directions
            path = []
            
            # Forward path
            current = current_box
            while current in forward_prev:
                path.insert(0, detail_points[current])
                current = forward_prev[current]
            path.insert(0, detail_points[current])
            
            # Backward path
            current = current_box
            while current in backward_prev:
                path.append(detail_points[current])
                current = backward_prev[current]
            path.append(detail_points[current])
            
            return path, list(visited)
            
        # Get appropriate search direction data structures
        if direction == 'forward':
            dist_map = forward_dist
            prev_map = forward_prev
            target_point = destination_point
        else:
            dist_map = backward_dist
            prev_map = backward_prev
            target_point = source_point
        
        # Only explore valid adjacent boxes (using mesh's adjacency info)
        for next_box in mesh['adj'][current_box]:
            # Use current point constrained to next box's boundaries
            current_point = detail_points[current_box]
            x1, x2, y1, y2 = next_box
            entry_x = max(x1, min(x2, current_point[0]))
            entry_y = max(y1, min(y2, current_point[1]))
            entry_point = (entry_x, entry_y)
            
            # Calculate new distance and check if this path is better
            new_dist = dist_map[current_box] + distance(current_point, entry_point)
            
            if next_box not in dist_map or new_dist < dist_map[next_box]:
                detail_points[next_box] = entry_point
                dist_map[next_box] = new_dist
                prev_map[next_box] = current_box
                
                # Priority is f = g + h where g is distance so far and h is heuristic
                h = distance(entry_point, target_point)
                heappush(queue, (new_dist + h, next_box, direction))
    
    print("No path!")
    return [], list(visited)

