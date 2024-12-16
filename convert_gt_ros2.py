import pandas as pd
import os
import sys

def modify_data(input_file):
    # Read the data from the CSV file
    #file_path = '/home/user/nvidia_hawk_camera_tests/suv_side_cam_mapping2/GT_raw.txt'  # Replace with your actual file path
    
    df = pd.read_csv(input_file, header=None)

    #df[0] = df[0].astype(str) + '.' + df[1].astype(str.rjust(9, "0"))
    df[0] = df[0].astype(str) + '.' + df[1].astype(str).str.rjust(9, "0")

    # Drop the second and third columns
    df.drop(columns=[1, 2], inplace=True)

    # Reset the column indices to reflect changes
    df.columns = range(df.shape[1])

    # Convert DataFrame to string with space-separated values
    space_separated = df.apply(lambda row: ' '.join(row.astype(str)), axis=1)

    # Print the modified data
    print(space_separated.to_string(index=False))

    # Get the directory and filename of the input file
    dir_name = os.path.dirname(input_file)
    base_name = os.path.basename(input_file)
    
    # Create the output filename
    output_file = os.path.join(dir_name, base_name.replace('.txt', '_modified.tum'))
    
    with open(output_file, 'w') as f:
        f.write('\n'.join(space_separated))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python modify_data.py <input_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    modify_data(input_file)