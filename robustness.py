import pandas as pd

def calculate_lost_percentage(file_path):
    # Load the data from the CSV file
    df = pd.read_csv(file_path)

    # Calculate the total number of entries
    total_entries = len(df)
    
    # Calculate the number of times "lost" appears in the Tracking Status column
    lost_entries = len(df[df['Tracking Status'] == 'lost'])
    
    # Calculate the percentage of "lost" entries
    lost_percentage = (lost_entries / total_entries) * 100
    
    # Check if 2 seconds worth of statuses at the beginning shows 'lost'
    two_seconds_ns = 2 * 10**9
    start_time = df['Timestamp (ns)'].iloc[0]
    end_time = start_time + two_seconds_ns
    
    # Filter the dataframe for the first 2 seconds
    first_two_seconds_df = df[df['Timestamp (ns)'] <= end_time]
    
    # Check if all entries in the first 2 seconds have 'lost' status
    all_lost_in_first_two_seconds = all(first_two_seconds_df['Tracking Status'] == 'lost')
    
    return lost_percentage, all_lost_in_first_two_seconds

if __name__ == "__main__":
    file_path = '/home/user/Downloads/KdV3_test_m1_2.csv'  # Replace with your actual file path
    lost_percentage, all_lost_in_first_two_seconds = calculate_lost_percentage(file_path)
    print(f"The 'Tracking Status' shows 'lost' {lost_percentage:.2f}% of the time.")
    if all_lost_in_first_two_seconds:
        print("The first 2 seconds worth of statuses all show 'lost'.")
    else:
        print("The first 2 seconds worth of statuses do not all show 'lost'.")


