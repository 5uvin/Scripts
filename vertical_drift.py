import pandas as pd

def calculate_exceedance_percentage(file_path):
    # Load the data from the CSV file with space as delimiter
    df = pd.read_csv(file_path, delim_whitespace=True, header=None)

    # Extract the 4th column
    fourth_column = df.iloc[:, 3]

    # Calculate the number of values greater than 0.1
    exceedances = fourth_column[fourth_column > 0.1]

    # Calculate the percentage of values greater than 0.1
    exceedance_percentage = (len(exceedances) / len(fourth_column)) * 100

    # Check if more than 10% of the values exceed 0.1
    if exceedance_percentage > 10:
        print("fail")
    else:
        print("pass")
    
    return exceedance_percentage

if __name__ == "__main__":
    file_path = '/home/user/scripts/KdV3_test_m1_3'  # Replace with your actual file path
    exceedance_percentage = calculate_exceedance_percentage(file_path)
    print(f"The percentage of values in the 4th column that are above 0.1 m is {exceedance_percentage:.2f}%.")

