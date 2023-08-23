import numpy as np

def calculate_minor_arc_coordinates(start, end, center, radius, clockwise=True, num_points=100):
    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
    clockwise = False
    if clockwise:
        if start_angle < end_angle:
            start_angle += 2 * np.pi
    else:
        if start_angle > end_angle:
            end_angle += 2 * np.pi  
    angles = np.linspace(start_angle, end_angle, num_points)
    arc_coordinates = []
    
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        arc_coordinates.append((x, y))
    
    return arc_coordinates

# Define the endpoints and the center
start_point = (1, 1)
end_point = (4, 3)
center = (2, 2)
radius = np.linalg.norm(np.array(start_point) - np.array(center))
num_points = 20  # Number of points along the arc

# Calculate the coordinates of the minor arc
arc_coordinates = calculate_minor_arc_coordinates(start_point, end_point, center, radius, num_points=num_points)

# # Print the coordinates
# for point in arc_coordinates:
#     print(point)

