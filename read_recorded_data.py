import torch
import random

# Load the recorded data
file_path = "recorded_data.pt"

try:
    recorded_data = torch.load(file_path)
except FileNotFoundError:
    print(f"The file {file_path} does not exist.")
    exit()

# Ensure there are enough records to pick from
num_records = len(recorded_data["object_position"])
if num_records < 2:
    print("Not enough records to pick two unique sets.")
    exit()

# Randomly pick two unique indices
indices = random.sample(range(num_records), 100)

# Function to display position and angle data at a given index
def display_position_and_angle(data, index):
    print(f"\nData set at index {index}:")
    print("Object Position:", data["object_position"][index])
    print("Object Angle:", data["object_angle"][index])
    print("Disc Position:", data["disc_position"][index])
    print("Disc Angle:", data["disc_angle"][index])
    print("Joint Angle:", data["joint_angles"][index])

# Display the selected position and angle data for the two sets
display_position_and_angle(recorded_data, indices[0])
display_position_and_angle(recorded_data, indices[1])
