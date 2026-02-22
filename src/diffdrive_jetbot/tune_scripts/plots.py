#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

def calculate_metrics_for_run(group):
    """Calculates a performance score for a single test run (one test_id). Lower is better."""
    if group.empty:
        return np.inf

    group = group.sort_values('timestamp').reset_index(drop=True)
    duration = group['timestamp'].max()
    target_speed = group['target_left'].iloc[0]
    if abs(target_speed) < 1e-3:
        return np.inf

    # --- Rise Time (10% to 90%) ---
    try:
        t_10 = group[group['actual_speed_left'].abs() >= 0.1 * target_speed]['timestamp'].iloc[0]
        t_90 = group[group['actual_speed_left'].abs() >= 0.9 * target_speed]['timestamp'].iloc[0]
        rise_time_left = t_90 - t_10
    except IndexError:
        rise_time_left = duration  # Penalize if it never reaches 90%

    try:
        t_10_r = group[group['actual_speed_right'].abs() >= 0.1 * target_speed]['timestamp'].iloc[0]
        t_90_r = group[group['actual_speed_right'].abs() >= 0.9 * target_speed]['timestamp'].iloc[0]
        rise_time_right = t_90_r - t_10_r
    except IndexError:
        rise_time_right = duration

    rise_time = (rise_time_left + rise_time_right) / 2.0

    # --- Overshoot ---
    overshoot_left = max(0, (group['actual_speed_left'].abs().max() - target_speed) / target_speed)
    overshoot_right = max(0, (group['actual_speed_right'].abs().max() - target_speed) / target_speed)
    overshoot = (overshoot_left + overshoot_right) / 2.0

    # --- Steady State Error (last 10% of data) ---
    ss_group = group.tail(max(1, int(len(group) * 0.1)))
    ss_error_left = (target_speed - ss_group['actual_speed_left']).abs().mean()
    ss_error_right = (target_speed - ss_group['actual_speed_right']).abs().mean()
    steady_state_error = (ss_error_left + ss_error_right) / 2.0

    # --- Settling Time (time to enter and stay in 5% band of final value) ---
    final_val_l = group['actual_speed_left'].iloc[-1]
    tolerance_l = abs(final_val_l * 0.05)
    unsettled_l = group[(group['actual_speed_left'] - final_val_l).abs() > tolerance_l]
    settling_time_l = unsettled_l['timestamp'].max() if not unsettled_l.empty else group['timestamp'].min()

    final_val_r = group['actual_speed_right'].iloc[-1]
    tolerance_r = abs(final_val_r * 0.05)
    unsettled_r = group[(group['actual_speed_right'] - final_val_r).abs() > tolerance_r]
    settling_time_r = unsettled_r['timestamp'].max() if not unsettled_r.empty else group['timestamp'].min()
    
    settling_time = (settling_time_l + settling_time_r) / 2.0

    # --- Synchronization Error ---
    sync_error = (group['actual_speed_left'] - group['actual_speed_right']).abs().mean()

    # --- Overall Score (weights from C++ code) ---
    score = (0.15 * rise_time +
             0.20 * overshoot * 100.0 +
             0.30 * steady_state_error +
             0.25 * settling_time +
             0.10 * sync_error)
    return score

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

    # Find best PID by calculating score for each
    scores = {}
    for pid_tuple, pid_group in data.groupby(['pid_p', 'pid_i', 'pid_d']):
        run_scores = [calculate_metrics_for_run(run_group) for _, run_group in pid_group.groupby('test_id')]
        if run_scores:
            scores[pid_tuple] = np.mean(run_scores)

    best_pid = min(scores, key=scores.get) if scores else None
    if best_pid:
        print(f"Best PID found (Score: {scores[best_pid]:.3f}): P={best_pid[0]:.3f}, I={best_pid[1]:.3f}, D={best_pid[2]:.3f}")
    else:
        print("Could not determine best PID.")

    # Generate title with PID values
    unique_pids = data[['pid_p', 'pid_i', 'pid_d']].drop_duplicates()
    title_lines = ['PID Tuning Results']
    for _, row in unique_pids.iterrows():
        pid_tuple = (row['pid_p'], row['pid_i'], row['pid_d'])
        score_str = f"(Score: {scores.get(pid_tuple, 'N/A'):.2f})" if pid_tuple in scores else ""
        pid_str = f"P: {row['pid_p']:.3f}, I: {row['pid_i']:.3f}, D: {row['pid_d']:.3f} {score_str}"
        if pid_tuple == best_pid:
            pid_str = f"-> {pid_str} [BEST]"
        title_lines.append(pid_str)
    title = '\n'.join(title_lines)

    # Create plots
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(title, fontsize=10)

    # Setup colors
    unique_test_ids = sorted(data['test_id'].unique())
    num_tests = len(unique_test_ids)
    blues = plt.cm.get_cmap('Blues', num_tests + 4)
    reds = plt.cm.get_cmap('Reds', num_tests + 4)
    greens = plt.cm.get_cmap('Greens', num_tests + 4)
    test_id_to_color_idx = {tid: i for i, tid in enumerate(unique_test_ids)}

    # Group data by test_id to plot each run separately
    for test_id, group in data.groupby('test_id'):
        pid_tuple = (group['pid_p'].iloc[0], group['pid_i'].iloc[0], group['pid_d'].iloc[0])
        is_best = (pid_tuple == best_pid)
        linewidth = 2.5 if is_best else 1.0
        alpha = 1.0 if is_best else 0.6
        zorder = 10 if is_best else 1

        color_idx = test_id_to_color_idx[test_id]
        color_blue = blues(color_idx + 3)
        color_red = reds(color_idx + 3)
        color_green = greens(color_idx + 3)

        # Speed response
        axes[0,0].plot(group['timestamp'], group['actual_speed_left'], color=color_blue, linewidth=linewidth, alpha=alpha, zorder=zorder)
        axes[0,0].plot(group['timestamp'], group['actual_speed_right'], color=color_red, linewidth=linewidth, alpha=alpha, zorder=zorder)
        axes[0,0].plot(group['timestamp'], group['target_left'], 'k--', alpha=0.5)

        # Error plot
        axes[0,1].plot(group['timestamp'], group['error_left'], color=color_blue, linewidth=linewidth, alpha=alpha, zorder=zorder)
        axes[0,1].plot(group['timestamp'], group['error_right'], color=color_red, linewidth=linewidth, alpha=alpha, zorder=zorder)

        # Position plot
        axes[1,0].plot(group['timestamp'], group['pos_left'], color=color_blue, linewidth=linewidth, alpha=alpha, zorder=zorder)
        axes[1,0].plot(group['timestamp'], group['pos_right'], color=color_red, linewidth=linewidth, alpha=alpha, zorder=zorder)

        # Synchronization error
        sync_error = abs(group['actual_speed_left'] - group['actual_speed_right'])
        axes[1,1].plot(group['timestamp'], sync_error, color=color_green, linewidth=linewidth, alpha=alpha, zorder=zorder)

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

    plt.tight_layout(rect=[0, 0.03, 1, 0.90])
    
    # Save and show plot
    output_filename = os.path.splitext(args.csv_file)[0] + '_plots.png'
    plt.savefig(output_filename, dpi=300)
    print(f"Plot saved to '{output_filename}'")
    
    plt.show()

if __name__ == '__main__':
    main()
