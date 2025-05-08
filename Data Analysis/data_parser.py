import os
import time

input_file = "/Users/gilles/Documents/Github/fsb/Data Analysis/Logs/data1.txt"
output_files = {
    "LEFT": "/Users/gilles/Documents/Github/fsb/Data Analysis/Logs/left_data.csv",
    "RIGHT": "/Users/gilles/Documents/Github/fsb/Data Analysis/Logs/right_data.csv",
    "BACK": "/Users/gilles/Documents/Github/fsb/Data Analysis/Logs/back_data.csv"
}


# Prepare file handles
outputs = {key: open(path, "w") for key, path in output_files.items()}
for f in outputs.values():
    f.write("timestamp,value\n")  # Write headers

timestamp = 0
sampling_interval = 0.05  # 50ms between readings based on the sensor script

with open(input_file, "r") as f:
    for line in f:
        line = line.strip()
        
        # Skip empty lines
        if not line:
            continue
            
        # Parse the sensor data
        parts = line.split(":")
        if len(parts) == 2:
            sensor_type = parts[0].strip().lower()  # Convert to lowercase
            
            # Extract the sensor direction (left, right, back)
            if "left" in sensor_type:
                direction = "LEFT"
            elif "right" in sensor_type:
                direction = "RIGHT"
            elif "back" in sensor_type:
                direction = "BACK"
            else:
                continue  # Skip lines that don't match our sensors
                
            # Extract the value and write to the appropriate file
            try:
                value = int(parts[1].strip())
                outputs[direction].write(f"{timestamp:.2f},{value}\n")
                
                # Only increment timestamp after a complete set of readings
                # This assumes the data comes in sets of 3 (left, right, back)
                if direction == "BACK":
                    timestamp += sampling_interval
            except ValueError:
                # Skip lines where the value isn't a valid integer
                continue

# Close all output files
for f in outputs.values():
    f.close()

print(f"Data parsing complete. Processed data up to timestamp {timestamp:.2f} seconds.")
