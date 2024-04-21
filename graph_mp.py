import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_data(csv_file):
    # Load data from a CSV file
    data = pd.read_csv(csv_file)
    
    # Generate time array (assuming data points are 10 ms apart)
    time = np.arange(0, len(data) * 0.01, 0.01)  # start, stop, step

    # Set a style
    plt.style.use('seaborn-v0_8-darkgrid')

    # Define column names and titles for plots
    columns = [
        ("x", "Position X (meters)"),
        ("y", "Position Y (meters)"),
        ("theta", "Orientation (radians)"),
        ("linearVel", "Linear Velocity (m/s)"),
        ("angularVel", "Angular Velocity (rad/s)"),
        ("linearAccel", "Linear Acceleration (m/s²)"),
        ("angularAccel", "Angular Acceleration (rad/s²)")
    ]

    # Calculate number of rows needed for a 2-column layout
    nrows = (len(columns) + 1) // 2

    # Create a figure and a grid of subplots
    fig, axes = plt.subplots(nrows=nrows, ncols=2, figsize=(15, 10))  # Adjust figsize as needed
    axes = axes.flatten()  # Flatten the array to make indexing easier

    # Color palette
    colors = plt.cm.viridis(np.linspace(0, 1, len(columns)))

    # Plot each column
    for ax, (col, title), color in zip(axes, columns, colors):
        ax.plot(time, data[col], color=color, label=col.replace('Accel', 'Acceleration').replace('Vel', 'Velocity'))  # Use time as x-axis, set color, and add label for legend
        ax.set_title(title, fontsize=14)
        ax.set_xlabel('Time (seconds)', fontsize=12)
        ax.set_ylabel('Value', fontsize=12)
        ax.legend()

    # If the number of plots is odd, hide the last ax if unused
    if len(columns) % 2 != 0:
        axes[-1].set_visible(False)

    # Adjust layout
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py <path_to_csv_file>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    plot_data(csv_file)
