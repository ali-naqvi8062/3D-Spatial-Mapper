
#   Currently set COM4 as serial port at 115.2kbps 8N1

import serial
from math import sin, cos
import open3d as o3d
import numpy as np

f = open("coordinates.xyz", "w")    #create a new file for writing

# open the serial port
s = serial.Serial(port='COM4', baudrate=115200, timeout=10)

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
print("Sending 's' to MCU")
s.write(b's')
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission

print("Confirming configuration with MCU")
for i in range(8):
    # read the data from the UART
    data = s.readline()
    # decode the data from bytes to string
    data = data.decode()
    # print the data
    print("Data received: " + data)

rotation_angle_deg = 11.25 ## for 32 measurements
rotation_angle_rad = (rotation_angle_deg * 3.14) / 180


# recieve 10 measurements from UART of MCU
for x in range(0,6600,300): #30cm step measurement (i.e for each 32 scan move the sensor up 30cm)
    print("Receiving 32 measurements from MCU")
    for i in range(32):
        # Read the data from the UART
        data = s.readline()
        # Decode the data from bytes to string
        data = data.decode()
        print("Data received: " + data)
    
        # Split the data by comma
        data = data.split(",")
    
        # Validate that data contains the expected number of elements and is not empty

	#data acquisition error try/except case:

        if len(data) > 0 and data[0] != '':
            try:
                distance = int(data[0])
                # Translate measurements to x,y,z coordinate system
                y = distance * cos(rotation_angle_rad * i)
                z = distance * sin(rotation_angle_rad * i)
                # Write the data to the file
                f.write('{0:d} {1:.2f} {2:.2f}\n'.format(x, y, z))
            except ValueError as e:
                print(f"Error converting data to integer: {e}")
                # Optionally, handle the error (e.g., by continuing to the next iteration)
                continue
        else:
            print("Invalid or incomplete data received.")

       
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"
    
#close the port
print("Closing: " + s.name)
s.close()
    
f.close()   #there should now be a file containing 8 vertex coordinates

# ------------------------ OPEN3D visualization ----------------------

pcd = o3d.io.read_point_cloud("coordinates.xyz", format="xyz")

points = np.loadtxt('coordinates.xyz', delimiter=' ')  # Adjust delimiter if necessary

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



