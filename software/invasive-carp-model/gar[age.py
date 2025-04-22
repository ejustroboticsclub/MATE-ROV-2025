import pandas as pd

# Data extracted from the image
data = {
    "Year": [2016, 2017, 2018, 2019, 2020, 2021, 2022, 2023, 2024, 2025],
    "Region 1": ["N", "Y", "Y", "Y", "Y", "Y", "Y", "Y", "Y", "Y"],
    "Region 2": ["N", "N", "N", "N", "N", "Y", "Y", "Y", "Y", "Y"],
    "Region 3": ["N", "N", "N", "N", "Y", "Y", "Y", "Y", "Y", "Y"],
    "Region 4": ["N", "N", "N", "N", "N", "N", "Y", "Y", "Y", "Y"],
    "Region 5": ["N", "N", "N", "N", "N", "N", "N", "N", "N", "N"],
}

# Create a DataFrame
df = pd.DataFrame(data)

# Save the DataFrame to a csv file
df.to_csv("regions_data.csv", index=False)