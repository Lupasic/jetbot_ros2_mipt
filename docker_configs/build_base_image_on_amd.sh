#!/bin/bash

# Function to show usage
show_usage() {
    echo "Usage:"
    echo "  $0 tar <filename>     - Build and save to tar file"
    echo "  $0 push <tagname>     - Build and push to repository"
    echo ""
    echo "Examples:"
    echo "  $0 tar jetbot_ros2_base.tar"
    echo "  $0 push v1.0.0"
}

# Check if enough arguments provided
if [ $# -lt 2 ]; then
    echo "Error: Insufficient arguments provided"
    show_usage
    exit 1
fi

MODE=$1
ARGUMENT=$2

# Check if Dockerfile_base exists
if [ ! -f "Dockerfile_base" ]; then
    echo "Error: Dockerfile_base not found in current directory"
    exit 1
fi

# Validate mode and argument
case $MODE in
    "tar")
        if [ -z "$ARGUMENT" ]; then
            echo "Error: Filename not provided for tar mode"
            show_usage
            exit 1
        fi
        
        # Check if file already exists
        if [ -f "$ARGUMENT" ]; then
            echo "Warning: File $ARGUMENT already exists. It will be overwritten."
            read -p "Continue? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Operation cancelled"
                exit 0
            fi
        fi
        
        echo "Building and saving to tar file: $ARGUMENT"
        docker buildx build --output type=docker,dest="$ARGUMENT" \
        -t jetbot_ros2_base \
        --platform=linux/arm64 \
        --progress=auto \
        -f Dockerfile_base \
        ..
        ;;
        
    "push")
        if [ -z "$ARGUMENT" ]; then
            echo "Error: Tag name not provided for push mode"
            show_usage
            exit 1
        fi
        
        # Validate tag name format (basic validation)
        if [[ ! $ARGUMENT =~ ^[a-zA-Z0-9._-]+$ ]]; then
            echo "Error: Invalid tag name format. Use only alphanumeric characters, dots, underscores, and hyphens"
            exit 1
        fi
        
        echo "Building and pushing to repository with tag: $ARGUMENT"
        docker buildx build \
        -t lupasic/jetbot_ros_humble_base:"$ARGUMENT" \
        --platform=linux/arm64 \
        --progress=auto \
        --push \
        -f Dockerfile_base \
        ..
        ;;
        
    *)
        echo "Error: Invalid mode '$MODE'. Use 'tar' or 'push'"
        show_usage
        exit 1
        ;;
esac

echo "Operation completed successfully"