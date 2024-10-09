
import rosbag2_py
import pandas as pd
import csv

# Paths to your ROS bag file and output CSV file
bag_file = 'rosbag2_2024_09_11-12_15_06'
output_file = 'poses.csv'

def read_rosbag(bag_file):
    reader = rosbag2_py.SequentialReader()
    
    # Set up storage and converter options
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_file,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    # Open the bag file
    reader.open(storage_options, converter_options)

    data = {'timestamp': [], 'x_ground_truth': [], 'y_ground_truth': [], 'x_amcl': [], 'y_amcl': []}

    for topic, msg, t in reader.messages():
        timestamp = t.time_since_epoch().nanoseconds / 1e9
        if topic == '/odom/pose/pose/position':
            x_gt = msg.pose.position.x
            y_gt = msg.pose.position.y
            data['timestamp'].append(timestamp)
            data['x_ground_truth'].append(x_gt)
            data['y_ground_truth'].append(y_gt)
        elif topic == '/amcl_pose/pose/pose/position':
            x_amcl = msg.pose.position.x
            y_amcl = msg.pose.position.y
            data['x_amcl'].append(x_amcl)
            data['y_amcl'].append(y_amcl)
    return data

def save_to_csv(data, output_file):
    with open(output_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['timestamp', 'x_ground_truth', 'y_ground_truth', 'x_amcl', 'y_amcl'])
        for i in range(len(data['timestamp'])):
            csv_writer.writerow([
                data['timestamp'][i],
                data['x_ground_truth'][i] if i < len(data['x_ground_truth']) else '',
                data['y_ground_truth'][i] if i < len(data['y_ground_truth']) else '',
                data['x_amcl'][i] if i < len(data['x_amcl']) else '',
                data['y_amcl'][i] if i < len(data['y_amcl']) else '',
            ])

data = read_rosbag(bag_file)
save_to_csv(data, output_file)
