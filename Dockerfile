# Base ROS 2 desktop image
FROM rwthika/ros2:humble-desktop-full-arm64

# GUI settings for RViz
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Allow access to serial ports
RUN usermod -aG dialout root

# Update and install required dependencies
RUN apt-get update && \
    apt-get install -y \
        python3-venv \
        python3-pip \
        python3-dev \
        sudo \
        nano \
        ros-humble-tf-transformations \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-joint-state-broadcaster \
        ros-humble-diff-drive-controller \
        ros-humble-joint-trajectory-controller \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

