import pandas as pd

# Load the CSV file with tab separator and skip the first 10 rows
file_path = '/home/user/Downloads/2024-06-14-AMR-Tests/GT files/2024-06-14-13-01-KdV3-Test-M11-Gemini.csv'  # Replace with your file path
df = pd.read_csv(file_path, delim_whitespace=True, skiprows=10)

# Remove row 11 (10-indexed)
df = df.drop(index=[11])

# Delete columns 1 and the last column
df = df.drop(columns=[df.columns[0], df.columns[-1]])

# Change header names to 'timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'
new_headers = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
df.columns = new_headers

# Save the modified DataFrame to a new CSV file
output_file_path = '/home/user/Downloads/2024-06-14-AMR-Tests/GT files/2024-06-14-13-01-KdV3-Test-M1-GT'
df.to_csv(output_file_path, index=False)

print(f"Modified file saved to: {output_file_path}")

