import serial
from math import sin, cos
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("tofff_radar.xyz", format="xyz")

points = np.loadtxt('tofff_radar.xyz', delimiter=' ')  # Adjust delimiter if necessary

# Convert the numpy array of points into Open3D's point cloud format
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Determine unique X values and how many points per layer there are
unique_x_values = np.unique(points[:, 0])
points_per_layer = np.sum(points[:, 0] == unique_x_values[0])

# Initialize an empty list to hold line segments
lines = []

# First, create the horizontal lines within each layer
for x_val in unique_x_values:
    layer_indices = np.arange(len(points))[points[:, 0] == x_val]
    
    # Assuming that points are already in the correct order to form the loops
    for i in range(points_per_layer):
        # Connect each point to the next, wrapping around to the start to close the loop
        lines.append([layer_indices[i], layer_indices[(i + 1) % points_per_layer]])

# Next, create the vertical lines between layers
for i in range(len(unique_x_values) - 1):
    upper_layer_indices = np.arange(len(points))[points[:, 0] == unique_x_values[i]]
    lower_layer_indices = np.arange(len(points))[points[:, 0] == unique_x_values[i + 1]]
    
    # Assuming corresponding points are at the same positions in the ordered list
    for j in range(points_per_layer):
        # Connect vertical lines from upper to lower layer directly below each point
        lines.append([upper_layer_indices[j], lower_layer_indices[j]])

# Create a LineSet object to hold the lines
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines)
)

# Optionally, set colors for the lines for better visualization
colors = [[1, 0, 0] for i in range(len(lines))]  # Red color for all lines
line_set.colors = o3d.utility.Vector3dVector(colors)

# Visualize the point cloud and the lines together
o3d.visualization.draw_geometries([pcd, line_set])
