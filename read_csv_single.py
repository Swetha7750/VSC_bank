import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Close existing plots
plt.close('all')

# Define file path
file_path = 'flight_2_catch.csv'

# Check if file exists
if not os.path.exists(file_path):
    print(f"Error: File '{file_path}' not found!")
else:
    try:
        # Load only X,Y,Z columns (skip time)
        df = pd.read_csv(file_path, usecols=["pos/x6_x", "pos/x7_y", "pos/x8_z"])
        
        # Clean data
        df = df.dropna()  # Remove missing values
        df = df.drop_duplicates()  # Remove duplicates
        
        # Create 3D plot
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        ax.plot(
            df["pos/x7_y"],  # Y-axis
            df["pos/x6_x"],  # X-axis
            df["pos/x8_z"],  # Z-axis
            linewidth=1.5, 
            color='green', 
            label='Flight Path'
        )
        
        # Mark start/end points
        ax.scatter(df["pos/x7_y"].iloc[0], df["pos/x6_x"].iloc[0], df["pos/x8_z"].iloc[0],
                   color='red', s=100, label='Start', marker='o')
        ax.scatter(df["pos/x7_y"].iloc[-1], df["pos/x6_x"].iloc[-1], df["pos/x8_z"].iloc[-1],
                   color='blue', s=100, label='End', marker='s')
        
        # Labels and title
        ax.set_xlabel('Y (m)', fontsize=12)
        ax.set_ylabel('X (m)', fontsize=12)
        ax.set_zlabel('Z (m)', fontsize=12)
        ax.set_title('3D Flight Trajectory (X,Y,Z Only)', fontsize=14)
        
        # Legend and grid
        ax.legend(fontsize=10)
        ax.grid(True, linestyle='--', alpha=0.7)
        
        # Set viewing angle
        ax.view_init(elev=20, azim=45)
        
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"Error: {e}")