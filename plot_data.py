import matplotlib.pyplot as plt
import pandas as pd

def plot_data(file_path):
    # Load data from file
    data = pd.read_csv(file_path)  # or any appropriate method to read your data
    
    # Plotting
    plt.figure()
    plt.plot(data['time'], data['ground_truth'], label='Ground Truth')
    plt.plot(data['time'], data['amcl'], label='AMCL')
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.legend()
    plt.title('Ground Truth vs AMCL Position')
    plt.savefig('plot.png')
    plt.show()

if __name__ == "__main__":
    output_file = 'data.csv'  # Define your output file or path here
    plot_data(output_file)
