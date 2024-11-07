import os
import torch

# Load the original data file
input_file = "recorded_data_4500.pt"
output_directory = "recorded_data"
os.makedirs(output_directory, exist_ok=True)

# Define the chunk size
chunk_size = 100

# Load the data
recorded_data = torch.load(input_file)

# Get the total number of records
total_records = len(recorded_data["object_position"])

# Split and save each chunk
for i in range(0, total_records, chunk_size):
    # Create a chunk of data
    chunk_data = {
        "object_position": recorded_data["object_position"][i:i + chunk_size],
        "object_angle": recorded_data["object_angle"][i:i + chunk_size],
        "disc_position": recorded_data["disc_position"][i:i + chunk_size],
        "disc_angle": recorded_data["disc_angle"][i:i + chunk_size],
        "joint_angles": recorded_data["joint_angles"][i:i + chunk_size],
    }

    # Calculate the file index (e.g., 100, 200, ...)
    file_index = (i // chunk_size + 1) * 100

    # Define the output file path
    output_file = os.path.join(output_directory, f"recorded_data_{file_index}.pt")

    # Save the chunk to the new file
    torch.save(chunk_data, output_file)
    print(f"Saved {output_file} with {len(chunk_data['object_position'])} records.")
