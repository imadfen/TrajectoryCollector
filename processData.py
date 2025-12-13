import pandas as pd

# Load data
df = pd.read_csv("results/data_car_15.csv")

# Create Labels (e.g., Position 2 seconds into the future)
T_seconds = 2.0 
# Find how many rows correspond to T seconds (assuming 0.1s step)
rows_shift = int(T_seconds / 0.1) 

# Create the label columns by shifting data BACKWARDS
df['Future_X'] = df['X'].shift(-rows_shift)
df['Future_Y'] = df['Y'].shift(-rows_shift)

# Drop the last rows (which now have NaN because they have no future)
df = df.dropna()

# Save for Training
df.to_csv("training_dataset.csv", index=False)
