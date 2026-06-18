from rosbags.highlevel import AnyReader
from pathlib import Path
import csv

# --- CONFIGURATION ---
# 1. Change this to the actual folder name created by 'ros2 bag record'
# (It usually looks like 'rosbag2_2026_06_15-12_00_00')
bag_path = Path('./record_hands') 

# 2. Output CSV filename
csv_filename = 'raw_landmarks.csv'
# ---------------------

print(f"Reading bag from: {bag_path}")

# Open the bag file using AnyReader (works for both .db3 and .mcap)
with AnyReader([bag_path]) as reader:
    # We only want messages from the /raw_landmarks topic
    connections = [c for c in reader.connections if c.topic == '/raw_landmarks']
    
    if not connections:
        print("Error: Could not find topic '/raw_landmarks' in this bag file.")
    else:
        # Open CSV file for writing
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_sec', 'data']) # Header row
            
            print("Extracting messages...")
            # Loop through the messages
            for connection, timestamp, raw in reader.messages(connections):
                # ROS bag timestamps are in nanoseconds. Convert to seconds.
                time_sec = timestamp / 1e9
                
                # Deserialize the raw bytes into a Python object
                # Since it's std_msgs/msg/String, it will have a .data attribute
                msg = reader.deserialize(raw, connection.msgtype)
                
                # Write the time and the string data to the CSV
                writer.writerow([time_sec, msg.data])

print(f"Done! Successfully saved to {csv_filename}")