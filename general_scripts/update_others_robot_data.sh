#!/bin/bash

# Script to update configuration and restart Docker containers on a remote JetBot.
#
# Usage: ./update_others_robot_data.sh <ssh_target>
# Example: ./update_others_robot_data.sh jetbot@192.168.1.219

# Check if SSH target is provided
if [ -z "$1" ]; then
  echo "Error: SSH target not provided."
  echo "Usage: $0 <ssh_target>"
  echo "Example: $0 jetbot@192.168.1.219"
  exit 1
fi

SSH_TARGET=$1
PASSWORD="jetbot"

# Get the absolute path to the script's directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_DIR=$(dirname "$SCRIPT_DIR")

LOCAL_CONFIG_PATH="/home/jetbot/Programs/jetbot_ros2_docker/src/jetbot_bringup/config/nav2_default_params.yaml"
REMOTE_CONFIG_PATH=$LOCAL_CONFIG_PATH
REMOTE_DOCKER_PATH="/home/jetbot/Programs/jetbot_ros2_docker"

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass not found. Please install it to continue."
    echo "For Debian/Ubuntu based systems: sudo apt-get install sshpass"
    exit 1
fi

# 1. Transfer the configuration file using scp
echo "Transferring configuration file to ${SSH_TARGET}..."
sshpass -p "$PASSWORD" scp -o StrictHostKeyChecking=no "$LOCAL_CONFIG_PATH" "${SSH_TARGET}:${REMOTE_CONFIG_PATH}"

if [ $? -ne 0 ]; then
  echo "Failed to transfer the file. Please check the SSH target and network connection."
  exit 1
fi
echo "Configuration file transferred successfully."

# 2. Execute commands on the remote robot
echo "Executing Docker commands on ${SSH_TARGET}..."
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$SSH_TARGET" "
    echo 'Connected to ${SSH_TARGET}';
    cd ${REMOTE_DOCKER_PATH} || exit;
    echo 'Stopping Docker containers...';
    docker compose down;
    echo 'Building Docker image...';
    docker build -t jetbot_full_ros2 .;
    echo 'Starting Docker containers...';
    docker compose up -d;
    echo 'Commands executed. Disconnecting.';
"

if [ $? -ne 0 ]; then
  echo "Failed to execute remote commands."
  exit 1
fi

echo "Script finished successfully."
