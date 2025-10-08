import pandas as pd
import numpy as np  # Import numpy for grid calculations
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import LogNorm


# --- HELPER FUNCTIONS from before ---
def load_obstacles_from_csv(csv_file):
    # (This function is unchanged)
    try:
        df = pd.read_csv(csv_file)
        obstacles = []
        for index, row in df.iterrows():
            obstacles.append({
                'center': (row['x'], row['y']),
                'radius': row['radius']
            })
        print(f"Successfully loaded {len(obstacles)} obstacles from '{csv_file}'.")
        return obstacles
    except FileNotFoundError:
        print(f"Error: The obstacle file '{csv_file}' was not found.")
        return []
    except KeyError:
        print(f"Error: The CSV file '{csv_file}' must contain 'x', 'y', and 'radius' columns.")
        return []

def load_scenario_config(config_file):
    # (This function is unchanged)
    config = {}
    try:
        with open(config_file, 'r') as f:
            for line in f:
                if ':' in line:
                    key, value = line.split(':', 1)
                    config[key.strip()] = float(value.strip())
        start_pos = (config['start_x'], config['start_y'])
        goal_pos = (config['goal_x'], config['goal_y'])
        print(f"Successfully loaded scenario from '{config_file}'.")
        return start_pos, goal_pos
    except FileNotFoundError:
        print(f"Error: The config file '{config_file}' was not found.")
        return None, None
    except (KeyError, ValueError) as e:
        print(f"Error: Config file '{config_file}' is missing a key or has an invalid value. {e}")
        return None, None# Add this import at the top of the python script if you haven't already
from matplotlib.colors import LogNorm

# --- UPDATED FUNCTION TO CALCULATE AND PLOT THE POTENTIAL FIELD HEATMAP ---
def plot_potential_heatmap(ax, goal_pos, obstacles_info, density=200):
    """Calculates and plots the potential field heatmap using a grid of a specified density."""
    # Define the boundaries of the plot area
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    
    # Create a grid of points using linspace for a fixed density
    x = np.linspace(xlim[0], xlim[1], density)
    y = np.linspace(ylim[0], ylim[1], density)
    xx, yy = np.meshgrid(x, y)

    # --- Potential Field Parameters ---
    zeta = 1 / 1000  # Attractive potential gain
    eta = 500       # Repulsive potential gain
    d_star = 50     # Obstacle zone of influence radius

    # 1. Calculate Attractive Potential
    dist_goal_sq = (xx - goal_pos[0])**2 + (yy - goal_pos[1])**2
    potential = 0.5 * zeta * dist_goal_sq

    # 2. Calculate Repulsive Potential for each obstacle
    for obs in obstacles_info:
        center_x, center_y = obs['center']
        radius = obs['radius']
        
        dist_obs = np.sqrt((xx - center_x)**2 + (yy - center_y)**2) - radius
        dist_obs = np.maximum(dist_obs, 0.01) # Avoid division by zero

        mask = dist_obs <= d_star
        repulsive_potential = 0.5 * eta * (1.0 / dist_obs - 1.0 / d_star)**2
        potential[mask] += repulsive_potential[mask]
    
    # Clip the potential for better visualization
    potential = np.clip(potential, 0, np.quantile(potential, 0.98))

    # Plot the heatmap with a logarithmic scale for better color contrast
    im = ax.imshow(potential, cmap='viridis', norm=LogNorm(), 
                   extent=[xlim[0], xlim[1], ylim[0], ylim[1]], 
                   origin='lower', alpha=0.7)
    return im

def plot_simulation_results(csv_file, obstacles_info, start_pos, goal_pos):
    """
    Reads simulation data and plots the USV path, velocity, and yaw angle.
    Now includes a heatmap of the potential field.
    """
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file}' was not found.")
        return
    if data.empty:
        print(f"Warning: The data file '{csv_file}' is empty. Nothing to plot.")
        return
        
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 14), gridspec_kw={'height_ratios': [4, 1, 1]})
    fig.tight_layout(pad=5.0)

    # --- Panel 1: Generated Path with Heatmap ---
    ax1.set_title("Generated Path and Potential Field", fontsize=14)
    ax1.set_xlabel("X / pixels")
    ax1.set_ylabel("Y / pixels")
    ax1.set_aspect('equal', adjustable='box')
    
    # First, plot the path and points to establish the plot boundaries
    ax1.plot(data['x'].to_numpy(), data['y'].to_numpy(), 'r-', linewidth=2, label='USV Path', zorder=3)
    ax1.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='Start Point', zorder=4)
    ax1.plot(goal_pos[0], goal_pos[1], 'b*', markersize=12, label='Goal', zorder=4)

    # Now, add the heatmap based on those boundaries
    heatmap_image = plot_potential_heatmap(ax1, goal_pos, obstacles_info, density=200)
    fig.colorbar(heatmap_image, ax=ax1, shrink=0.7, label='Potential U')

    # Then, draw the obstacles on top of everything
    for obs in obstacles_info:
        circle = Circle(obs['center'], obs['radius'], color='black', alpha=0.9, zorder=5)
        ax1.add_patch(circle)
        
    ax1.grid(True, linestyle='--', alpha=0.4, zorder=0)
    ax1.legend()
    # The origin='lower' in imshow handles the y-axis direction correctly.
    # ax1.invert_yaxis() # You may or may not need this depending on your preference.

    # --- Panel 2: Yaw Angle Profile ---
    ax2.set_title("Yaw Angle", fontsize=12)
    ax2.plot(data['step'].to_numpy(), data['yaw_deg'].to_numpy(), color='darkslategray')
    ax2.set_xlabel("Sample Steps")
    ax2.set_ylabel("Yaw Angle / °")
    ax2.grid(True, linestyle='--', alpha=0.6)

    # --- Panel 3: Velocity Profile ---
    ax3.set_title("Velocity", fontsize=12)
    ax3.plot(data['step'].to_numpy(), data['velocity'].to_numpy(), color='darkslategray')
    ax3.set_xlabel("Sample steps")
    ax3.set_ylabel("Velocity / (pixel per step)")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.set_ylim(bottom=0)
    
    plt.show()

# --- Main Execution Block ---
if __name__ == '__main__':
    # Define file paths
    OBSTACLE_FILE = 'obstacles.csv'
    SCENARIO_FILE = 'scenario.txt'
    PATH_DATA_FILE = 'path_data.csv'
    
    # Load all scenario data
    scenario_start, scenario_goal = load_scenario_config(SCENARIO_FILE)
    scenario_obstacles = load_obstacles_from_csv(OBSTACLE_FILE)
    
    # Plot results if everything loaded successfully
    if scenario_start and scenario_goal and scenario_obstacles:
        plot_simulation_results(
            PATH_DATA_FILE, 
            scenario_obstacles, 
            scenario_start, 
            scenario_goal
        )