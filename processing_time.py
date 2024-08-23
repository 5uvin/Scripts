import pandas as pd

def check_processing_time(file_path, fps):
    # Load the data from the CSV file
    df = pd.read_csv(file_path)
    
    # Calculate the threshold for processing time
    threshold = 1 / fps
    
    # Convert the processing time from nanoseconds to seconds
    df['Processing Time (s)'] = df['Processing Time (ns)'] / 1e9
    
    # Calculate the number of entries with processing time greater than the threshold
    exceedances = df[df['Processing Time (s)'] > threshold]
    
    # Calculate the percentage of entries with processing time greater than the threshold
    exceedance_percentage = (len(exceedances) / len(df)) * 100
    
    # Check if more than 2.5% of the entries exceed the threshold
    if exceedance_percentage > 2.5:
        print("fail")
    else:
        print("pass")
    
    return exceedance_percentage

if __name__ == "__main__":
    file_path = '/home/user/Downloads/KdV3_test_m1_2'  # Replace with your actual file path
    fps = 30  # Replace with the input frame rate
    exceedance_percentage = check_processing_time(file_path, fps)
    print(f"The percentage of log entries with processing time higher than the threshold is {exceedance_percentage:.2f}%.")

