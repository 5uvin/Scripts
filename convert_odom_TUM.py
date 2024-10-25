####################################################################################################################
#   if you've used `rostopic echo -b bag -p /topic > odom.txt` command to write out odom msgs from rosbag 
#   then, your odom.txt is not in TUM format, use this script to remove '%time' , frame_id, seq number and other columns  
#   this script also converts the timestamp from nsecs to secs which is how kdVisual does it 
#
#   Usage : python3 convert_gt.py <input_file>
###################################################################################################################

import pandas as pd
import os
import sys

def modify_data(input_file):
    # Load the data
    data = pd.read_csv(input_file)

    # Drop the unwanted columns
    data = data.drop(columns=['%time', 'field.header.frame_id', 'field.header.seq', 'field.child_frame_id', 'field.twist.twist.linear.x', 'field.twist.twist.linear.y', 'field.twist.twist.linear.z', 'field.twist.twist.angular.x', 'field.twist.twist.angular.y', 'field.twist.twist.angular.z'])

    for x in range(36):
        data = data.drop(columns=['field.pose.covariance{}'.format(x), 'field.twist.covariance{}'.format(x)])

    # Divide the 'field.header.stamp' column by 10^9
    data['field.header.stamp'] = data['field.header.stamp'] / 10**9

    new_headers = ['#timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
    data.columns = new_headers
    print(data)

    # Convert the DataFrame to a CSV string, replace commas with spaces
    csv_data = data.to_csv(index=False).replace(',', ' ')

    # Get the directory and filename of the input file
    dir_name = os.path.dirname(input_file)
    base_name = os.path.basename(input_file)
    
    # Create the output filename
    output_file = os.path.join(dir_name, base_name.replace('.txt', '_modified.tum'))

    # Write the modified data to the output file
    with open(output_file, 'w') as file:
        file.write(csv_data)

    print(f'Modified file saved as: {output_file}')

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python modify_data.py <input_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    modify_data(input_file)