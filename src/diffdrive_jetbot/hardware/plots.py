#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Plot PID tuning results from a CSV file.')
    parser.add_argument('csv_file', type=str, help='Path to the input CSV file.')
    args = parser.parse_args()

    # Check if file exists
    if not os.path.exists(args.csv_file):
        print(f"Error: File not found at '{args.csv_file}'")
        return

    # Load data
    try:
        data = pd.read_csv(args.csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # Create plots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('PID Tuning Results', fontsize=16)

    # Group data by test_id to plot each run separately
    for test_id, group in data.groupby('test_id'):
        # Speed response
        axes[0,0].plot(group['timestamp'], group['actual_speed_left'], 'b-')
        axes[0,0].plot(group['timestamp'], group['actual_speed_right'], 'r-')
        axes[0,0].plot(group['timestamp'], group['target_left'], 'k--')

        # Error plot
        axes[0,1].plot(group['timestamp'], group['error_left'], 'b-')
        axes[0,1].plot(group['timestamp'], group['error_right'], 'r-')

        # Position plot
        axes[1,0].plot(group['timestamp'], group['pos_left'], 'b-')
        axes[1,0].plot(group['timestamp'], group['pos_right'], 'r-')

        # Synchronization error
        sync_error = abs(group['actual_speed_left'] - group['actual_speed_right'])
        axes[1,1].plot(group['timestamp'], sync_error, 'g-')

    # Add labels and titles after the loop to avoid repetition in the legend
    # Speed response
    axes[0,0].plot([], [], 'b-', label='Left Actual')
    axes[0,0].plot([], [], 'r-', label='Right Actual')
    axes[0,0].plot([], [], 'k--', label='Target')
    axes[0,0].set_title('Speed Response')
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Speed')
    axes[0,0].legend()
    axes[0,0].grid(True)

    # Error plot
    axes[0,1].plot([], [], 'b-', label='Left Error')
    axes[0,1].plot([], [], 'r-', label='Right Error')
    axes[0,1].set_title('Tracking Error')
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Error')
    axes[0,1].legend()
    axes[0,1].grid(True)

    # Position plot
    axes[1,0].plot([], [], 'b-', label='Left Position')
    axes[1,0].plot([], [], 'r-', label='Right Position')
    axes[1,0].set_title('Wheel Positions')
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Position')
    axes[1,0].legend()
    axes[1,0].grid(True)

    # Synchronization error
    axes[1,1].plot([], [], 'g-', label='Sync Error')
    axes[1,1].set_title('Synchronization Error')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('|Left - Right| Speed')
    axes[1,1].legend()
    axes[1,1].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save and show plot
    output_filename = os.path.splitext(args.csv_file)[0] + '_plots.png'
    plt.savefig(output_filename, dpi=300)
    print(f"Plot saved to '{output_filename}'")
    
    plt.show()

if __name__ == '__main__':
    main()
