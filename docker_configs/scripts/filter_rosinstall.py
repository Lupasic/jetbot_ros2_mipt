#!/usr/bin/env python3
import os
import sys
import yaml

ROS_DISTRO = os.environ.get("ROS_DISTRO", "humble")
ROS_INSTALL_PATH = f"/opt/ros/{ROS_DISTRO}/install/share"

def get_installed_packages():
    try:
        return set(os.listdir(ROS_INSTALL_PATH))
    except FileNotFoundError:
        print(f"ERROR: ROS not installed at {ROS_INSTALL_PATH}")
        sys.exit(1)

def filter_rosinstall(input_path, output_path):
    with open(input_path, 'r') as infile:
        rosinstall = yaml.safe_load(infile)

    installed = get_installed_packages()

    filtered = []
    for entry in rosinstall:
        name = list(entry.values())[0].get("local-name").rpartition('/')[-1]
        if name and name in installed:
            print(f"Skipping installed package: {name}")
        else:
            filtered.append(entry)

    with open(output_path, 'w') as outfile:
        yaml.dump(filtered, outfile, sort_keys=False)

    print(f"Filtered rosinstall written to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: ./filter_rosinstall.py input.rosinstall output.rosinstall")
        sys.exit(1)
    filter_rosinstall(sys.argv[1], sys.argv[2])
