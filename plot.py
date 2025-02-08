import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

def get_max_allowed_torques(model_id: int):
    '''Function to get the max allowed torques for the given model_id'''

    # Define the mappings for max_allowed_torques based on model_id
    max_allowed_torques_mapping = {
        3: [133, 133, 107, 68.5, 68.5, 68.5],
        2: [133, 133, 107, 68.5, 68.5, 68.5],
        1: [133, 133, 107, 68.5, 68.5, 68.5]
    }

    # Return the corresponding max allowed torques for the model_id
    return max_allowed_torques_mapping.get(model_id, None)  # Returns None if model_id is not found


def plot_torques(model_id: int, static: bool):
    '''Funciton to plot the torrque values for all the motors'''
    
    if static:
        torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/static_torques_data.csv"
        plot_path = f"C:/Users/smart/Downloads/desc/plots/{model_id}_robot_model/static_torques_violin_plot.png"
    else:
        torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/dynamic_torques_data.csv"
        plot_path = f"C:/Users/smart/Downloads/desc/plots/{model_id}_robot_model/dynamic_torques_violin_plot.png"

    # Load torques data from the CSV file
    loaded_torques_data = pd.read_csv(torques_data_file)

    # Convert the data to a numpy array for easier processing (if needed)
    torques_array = loaded_torques_data.values

    # Create a violin plot
    plt.figure(figsize=(10, 6))

    # Plot a violin plot for each torque value in the data (columns in the torques_array)
    sns.violinplot(data=torques_array, inner="stick")

    # Add labels and title
    plt.title('Violin Plot of Torques for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Torque')

    # Save the plot to a file
    plt.savefig(plot_path)
    plt.close()


def plot_cheat_torques(model_id: int, static: bool):
    '''Funciton to plot the torrque values for all the motors except for the 1st motor'''

    if static:
        torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/static_torques_data.csv"
        plot_path = f"C:/Users/smart/Downloads/desc/plots/{model_id}_robot_model/cheat_static_torques_violin_plot.png"
    else:
        torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/dynamic_torques_data.csv"
        plot_path = f"C:/Users/smart/Downloads/desc/plots/{model_id}_robot_model/cheat_dynamic_torques_violin_plot.png"

    # Load torques data from the CSV file
    loaded_torques_data = pd.read_csv(torques_data_file)

    # Convert the data to a numpy array for easier processing (if needed)
    torques_array = loaded_torques_data.values
    torques_array_filtered = np.delete(torques_array, [0], axis=1)

    # Create a violin plot
    plt.figure(figsize=(10, 6))

    # Plot a violin plot for each torque value in the data (columns in the torques_array)
    sns.violinplot(data=torques_array_filtered, inner="stick")

    # Add labels and title
    plt.title('Violin Plot of Torques for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Torque')

    # Save the plot to a file
    plt.savefig(plot_path)
    plt.close()


def plot_torques_analytic(model_id: int, static: bool):
        
    '''Function to plot the torque values for all the motors as well as the max allowed torques'''

    if static:
        torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/static_torques_data.csv"
        plot_path = f"C:/Users/smart/Downloads/desc/plots/{model_id}_robot_model/analytical_static_torques_violin_plot.png"
    else:
        torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/dynamic_torques_data.csv"
        plot_path = f"C:/Users/smart/Downloads/desc/plots/{model_id}_robot_model/analytical_dynamic_torques_violin_plot.png"

    # Load torques data from the CSV file
    loaded_torques_data = pd.read_csv(torques_data_file)

    # Convert the data to a numpy array for easier processing (if needed)
    torques_array = loaded_torques_data.values
    torques_array_filtered = np.delete(torques_array, [0], axis=1)

    # Define the max allowed torques for each motor (example data)
    max_allowed_torques = get_max_allowed_torques(model_id)

    # Create a violin plot
    plt.figure(figsize=(10, 6))

    # Plot a violin plot for each torque value in the data (columns in the torques_array)
    sns.violinplot(data=torques_array_filtered, inner="stick")

    # Add horizontal lines for max allowed torque for each motor
    for i, max_torque in enumerate(max_allowed_torques):
        plt.axhline(y=max_torque, color='r', linestyle='--', label=f'Max Allowed Torque Motor {i+1}' if i == 0 else "")

    # Add labels and title
    plt.title('Violin Plot of Torques for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Torque')

    # Add a legend to indicate the max allowed torque lines
    plt.legend()

    # Save the plot to a file
    plt.savefig(plot_path)
    plt.close()