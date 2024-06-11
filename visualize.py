import os
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Function to extract pose and convert quaternion to euler angles
def extract_pose(msg):
    position = msg.pose.position
    orientation = msg.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ])
    return position.x, position.y, position.z, roll, pitch, yaw

# Function to apply a transformation and normalize yaw
def apply_transformation(data):
    dx = data['x'][0]
    dy = data['y'][0]
    dz = data['z'][0]
    dyaw = data['yaw'][0]

    transformed_data = {'time': data['time'], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': []}

    for i in range(len(data['x'])):
        # Translate the points
        x_new = data['x'][i] - dx
        y_new = data['y'][i] - dy
        z_new = data['z'][i] - dz

        # Rotate the points around the z-axis (yaw)
        cos_yaw = np.cos(-dyaw)
        sin_yaw = np.sin(-dyaw)
        x_rot = cos_yaw * x_new - sin_yaw * y_new
        y_rot = sin_yaw * x_new + cos_yaw * y_new

        # Adjust and normalize the yaw
        yaw_new = data['yaw'][i] - dyaw
        yaw_new = (yaw_new + np.pi) % (2 * np.pi) - np.pi

        transformed_data['x'].append(x_rot)
        transformed_data['y'].append(y_rot)
        transformed_data['z'].append(z_new)
        transformed_data['roll'].append(data['roll'][i])
        transformed_data['pitch'].append(data['pitch'][i])
        transformed_data['yaw'].append(yaw_new)

    return transformed_data

# Initialize lists to hold the data
pose_data_1 = {'time': [], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': []}
pose_data_optitrack = {'time': [], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': []}
pose_data_sources = {
    '/down/ov_msckf/poseimu': {'time': [], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': []},
    '/right/ov_msckf/poseimu': {'time': [], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': []},
    '/left/ov_msckf/poseimu': {'time': [], 'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': []}
}

# Print the list of .bag files in the current directory
bag_files = [f for f in os.listdir('.') if f.endswith('.bag')]
print("Available .bag files:")
for i, file in enumerate(bag_files):
    print(f"{i + 1}. {file}")

# Prompt the user to input the bag file name
bag_file_index = int(input("Please select the .bag file by entering the corresponding number: ")) - 1
bag_file = bag_files[bag_file_index]

# Open the bag file and read the messages
bag = rosbag.Bag(bag_file)
for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose', '/optitrack/vrpn_client_node/SQUID/pose', 
                                               '/down/ov_msckf/poseimu', '/right/ov_msckf/poseimu', '/left/ov_msckf/poseimu']):
    if topic == '/mavros/local_position/pose':
        x, y, z, roll, pitch, yaw = extract_pose(msg)
        pose_data_1['time'].append(t.to_sec())
        pose_data_1['x'].append(x)
        pose_data_1['y'].append(y)
        pose_data_1['z'].append(z)
        pose_data_1['roll'].append(roll)
        pose_data_1['pitch'].append(pitch)
        pose_data_1['yaw'].append(yaw)
    elif topic == '/optitrack/vrpn_client_node/SQUID/pose':
        x, y, z, roll, pitch, yaw = extract_pose(msg)
        pose_data_optitrack['time'].append(t.to_sec())
        pose_data_optitrack['x'].append(x)
        pose_data_optitrack['y'].append(y)
        pose_data_optitrack['z'].append(z)
        pose_data_optitrack['roll'].append(roll)
        pose_data_optitrack['pitch'].append(pitch)
        pose_data_optitrack['yaw'].append(yaw)
    elif topic in pose_data_sources:
        x, y, z, roll, pitch, yaw = extract_pose(msg.pose)
        pose_data_sources[topic]['time'].append(t.to_sec())
        pose_data_sources[topic]['x'].append(x)
        pose_data_sources[topic]['y'].append(y)
        pose_data_sources[topic]['z'].append(z)
        pose_data_sources[topic]['roll'].append(roll)
        pose_data_sources[topic]['pitch'].append(pitch)
        pose_data_sources[topic]['yaw'].append(yaw)
bag.close()

# Apply the transformation to each dataset
pose_data_1 = apply_transformation(pose_data_1)
pose_data_optitrack = apply_transformation(pose_data_optitrack)
for topic in pose_data_sources:
    pose_data_sources[topic] = apply_transformation(pose_data_sources[topic])

# 3D plot of all trajectories with equal scaling
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(pose_data_1['x'], pose_data_1['y'], pose_data_1['z'], label='/mavros/local_position/pose')
ax.plot(pose_data_optitrack['x'], pose_data_optitrack['y'], pose_data_optitrack['z'], label='/optitrack/vrpn_client_node/SQUID/pose')
for topic, data in pose_data_sources.items():
    ax.plot(data['x'], data['y'], data['z'], label=topic)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('3D Trajectory Plot')

# Ensure the scales for x, y, z are the same
max_range = np.array([max(pose_data_1['x']) - min(pose_data_1['x']),
                      max(pose_data_1['y']) - min(pose_data_1['y']),
                      max(pose_data_1['z']) - min(pose_data_1['z'])]).max() / 2.0

mid_x = (max(pose_data_1['x']) + min(pose_data_1['x'])) * 0.5
mid_y = (max(pose_data_1['y']) + min(pose_data_1['y'])) * 0.5
mid_z = (max(pose_data_1['z']) + min(pose_data_1['z'])) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()

# 2D plots for every pose attribute
attributes = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
for attr in attributes:
    plt.figure()
    plt.plot(pose_data_1['time'], pose_data_1[attr], label='/mavros/local_position/pose')
    plt.plot(pose_data_optitrack['time'], pose_data_optitrack[attr], label='/optitrack/vrpn_client_node/SQUID/pose')
    for topic, data in pose_data_sources.items():
        plt.plot(data['time'], data[attr], label=topic)
    plt.xlabel('Time (s)')
    plt.ylabel(attr.capitalize())
    plt.legend()
    plt.title(f'{attr.capitalize()} over Time')
    plt.show()