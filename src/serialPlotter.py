import random

# User input for minimum and maximum values
min_value = int(input("Enter the minimum value: "))
max_value = int(input("Enter the maximum value: "))

# User input for the number of "Low" pixels between "High" values
low_pixels_between_highs = int(input("Enter the number of 'Low' pixels between 'High' values: "))

# Initialize variables
sensorData = []  # To store the generated pattern
current_value = random.randint(min_value, max_value)  # Initialize with a random value

# Generate the pattern
while len(sensorData) < 2048:
    sensorData.append(current_value)

    # Toggle between "Low" and "High" values
    if current_value == min_value:
        current_value = max_value
    else:
        current_value = min_value

    # Add the specified number of "Low" pixels
    for _ in range(low_pixels_between_highs):
        sensorData.append(min_value)

# Ensure the pattern length is exactly 2048 by filling with "Low" pixels
sensorData.extend([min_value] * (2048 - len(sensorData)))

# Create a C-style uint16_t array string
c_style_array = ', '.join(map(str, sensorData))

# Define the output file path
output_file_path = 'sensorData.txt'

# Write the C-style array to a text file
with open(output_file_path, 'w') as file:
    file.write(f"uint16_t sensorData[2048] = {{{c_style_array}}};")

print(f"Array written to {output_file_path}")
