import os
import torch

# Define the directory containing the recorded data files
directory = "recorded_data"

# Iterate over all .pt files in the directory
for filename in os.listdir(directory):
    if filename.startswith("recorded_data_") and filename.endswith(".pt"):
        file_path = os.path.join(directory, filename)
        
        # Load the .pt file
        try:
            recorded_data = torch.load(file_path)
            
            # Check the length of recorded_data["object_position"]
            if len(recorded_data["object_position"]) < 40:
                # If the length is less than 40, delete the file
                os.remove(file_path)
                print(f"Deleted {file_path} (length of object_position was less than 40)")
                print("length of object_position: ", len(recorded_data["object_position"]))
            else:
                print(f"Kept {file_path} (length of object_position: {len(recorded_data['object_position'])})")
        
        except Exception as e:
            print(f"Error loading {file_path}: {e}")
