
# A*

# from math import sqrt
# from heapq import heappush, heappop

# def distance(p1, p2):
#     """Calculate Euclidean distance between two points"""
#     return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# def find_box_containing_point(point, mesh):
#     """Find the box that contains the given point"""
#     x, y = point
#     for box in mesh['boxes']:
#         x1, x2, y1, y2 = box
#         if x1 <= x <= x2 and y1 <= y <= y2:
#             return box
#     return None

# def find_path(source_point, destination_point, mesh):
#     """
#     Searches for a path from source_point to destination_point through the mesh
#     using bidirectional A* search.
#     """
#     # Find source and destination boxes
#     source_box = find_box_containing_point(source_point, mesh)
#     dest_box = find_box_containing_point(destination_point, mesh)
    
#     if not source_box or not dest_box:
#         print("No path!")
#         return [], []
        
#     if source_box == dest_box:
#         return [source_point, destination_point], [source_box]
    
#     # Initialize data structures
#     detail_points = {source_box: source_point}
#     forward_dist = {source_box: 0}
#     forward_prev = {}
#     visited = set([source_box])
    
#     # Priority queue for A* search
#     queue = []
#     heappush(queue, (0, source_box))
    
#     while queue:
#         _, current_box = heappop(queue)
        
#         if current_box == dest_box:
#             # Reconstruct path
#             path = [destination_point]
#             current = current_box
#             while current in forward_prev:
#                 path.insert(0, detail_points[current])
#                 current = forward_prev[current]
#             path.insert(0, source_point)
#             return path, list(visited)
        
#         current_point = detail_points[current_box]
        
#         # Explore neighbors
#         for next_box in mesh['adj'][current_box]:
#             visited.add(next_box)
            
#             # Calculate entry point into next box
#             x1, x2, y1, y2 = next_box
#             entry_x = min(max(x1, current_point[0]), x2)
#             entry_y = min(max(y1, current_point[1]), y2)
#             entry_point = (entry_x, entry_y)
            
#             # Calculate new distance
#             new_dist = forward_dist[current_box] + distance(current_point, entry_point)
            
#             if next_box not in forward_dist or new_dist < forward_dist[next_box]:
#                 detail_points[next_box] = entry_point
#                 forward_dist[next_box] = new_dist
#                 forward_prev[next_box] = current_box
                
#                 # A* estimate to goal
#                 h = distance(entry_point, destination_point)
#                 heappush(queue, (new_dist + h, next_box))
    
#     print("No path!")
#     return [], list(visited)




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
    """
    # Find source and destination boxes
    source_box = find_box_containing_point(source_point, mesh)
    dest_box = find_box_containing_point(destination_point, mesh)
    
    if not source_box or not dest_box:
        print("No path!")
        return [], []
        
    if source_box == dest_box:
        return [source_point, destination_point], [source_box]
    
    # Initialize forward search data structures
    detail_points_fwd = {source_box: source_point}
    forward_dist = {source_box: 0}
    forward_prev = {}
    
    # Initialize backward search data structures
    detail_points_back = {dest_box: destination_point}
    backward_dist = {dest_box: 0}
    backward_prev = {}
    
    # Track visited nodes from both directions
    visited = set([source_box, dest_box])
    
    # Priority queues for bidirectional A* search
    queue_fwd = [(0, source_box)]
    queue_back = [(0, dest_box)]
    
    # Track best meeting point
    best_total_dist = float('inf')
    meeting_box = None
    
    while queue_fwd and queue_back:
        # Process forward search
        if queue_fwd:
            _, current_box_fwd = heappop(queue_fwd)
            
            if current_box_fwd in backward_dist:  # Check for intersection
                total_dist = forward_dist[current_box_fwd] + backward_dist[current_box_fwd]
                if total_dist < best_total_dist:
                    best_total_dist = total_dist
                    meeting_box = current_box_fwd
            
            current_point = detail_points_fwd[current_box_fwd]
            
            # Explore forward neighbors
            for next_box in mesh['adj'][current_box_fwd]:
                visited.add(next_box)
                
                # Calculate entry point into next box
                x1, x2, y1, y2 = next_box
                entry_x = min(max(x1, current_point[0]), x2)
                entry_y = min(max(y1, current_point[1]), y2)
                entry_point = (entry_x, entry_y)
                
                # Calculate new distance
                new_dist = forward_dist[current_box_fwd] + distance(current_point, entry_point)
                
                if next_box not in forward_dist or new_dist < forward_dist[next_box]:
                    detail_points_fwd[next_box] = entry_point
                    forward_dist[next_box] = new_dist
                    forward_prev[next_box] = current_box_fwd
                    
                    # A* estimate to goal
                    h = distance(entry_point, destination_point)
                    heappush(queue_fwd, (new_dist + h, next_box))
        
        # Process backward search
        if queue_back:
            _, current_box_back = heappop(queue_back)
            
            if current_box_back in forward_dist:  # Check for intersection
                total_dist = forward_dist[current_box_back] + backward_dist[current_box_back]
                if total_dist < best_total_dist:
                    best_total_dist = total_dist
                    meeting_box = current_box_back
            
            current_point = detail_points_back[current_box_back]
            
            # Explore backward neighbors
            for next_box in mesh['adj'][current_box_back]:
                visited.add(next_box)
                
                # Calculate entry point into next box
                x1, x2, y1, y2 = next_box
                entry_x = min(max(x1, current_point[0]), x2)
                entry_y = min(max(y1, current_point[1]), y2)
                entry_point = (entry_x, entry_y)
                
                # Calculate new distance
                new_dist = backward_dist[current_box_back] + distance(current_point, entry_point)
                
                if next_box not in backward_dist or new_dist < backward_dist[next_box]:
                    detail_points_back[next_box] = entry_point
                    backward_dist[next_box] = new_dist
                    backward_prev[next_box] = current_box_back
                    
                    # A* estimate to start
                    h = distance(entry_point, source_point)
                    heappush(queue_back, (new_dist + h, next_box))
    
        # Check if we can terminate
        if meeting_box and (not queue_fwd or queue_fwd[0][0] + queue_back[0][0] >= best_total_dist):
            # Reconstruct path
            path = []
            
            # Reconstruct forward path
            current = meeting_box
            while current in forward_prev:
                path.insert(0, detail_points_fwd[current])
                current = forward_prev[current]
            path.insert(0, source_point)
            
            # Reconstruct backward path
            current = meeting_box
            path.append(detail_points_back[current])
            while current in backward_prev:
                current = backward_prev[current]
                path.append(detail_points_back[current])
            path.append(destination_point)
            
            return path, list(visited)
    
    print("No path!")
    return [], list(visited)


