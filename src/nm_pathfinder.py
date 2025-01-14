
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
    
    # Initialize data structures
    detail_points = {source_box: source_point}
    forward_dist = {source_box: 0}
    forward_prev = {}
    visited = set([source_box])
    
    # Priority queue for A* search
    queue = []
    heappush(queue, (0, source_box))
    
    while queue:
        _, current_box = heappop(queue)
        
        if current_box == dest_box:
            # Reconstruct path
            path = [destination_point]
            current = current_box
            while current in forward_prev:
                path.insert(0, detail_points[current])
                current = forward_prev[current]
            path.insert(0, source_point)
            return path, list(visited)
        
        current_point = detail_points[current_box]
        
        # Explore neighbors
        for next_box in mesh['adj'][current_box]:
            visited.add(next_box)
            
            # Calculate entry point into next box
            x1, x2, y1, y2 = next_box
            entry_x = min(max(x1, current_point[0]), x2)
            entry_y = min(max(y1, current_point[1]), y2)
            entry_point = (entry_x, entry_y)
            
            # Calculate new distance
            new_dist = forward_dist[current_box] + distance(current_point, entry_point)
            
            if next_box not in forward_dist or new_dist < forward_dist[next_box]:
                detail_points[next_box] = entry_point
                forward_dist[next_box] = new_dist
                forward_prev[next_box] = current_box
                
                # A* estimate to goal
                h = distance(entry_point, destination_point)
                heappush(queue, (new_dist + h, next_box))
    
    print("No path!")
    return [], list(visited)


