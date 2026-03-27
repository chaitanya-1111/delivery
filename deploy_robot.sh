#!/bin/bash
set -e  # Exit immediately if a command fails

echo "🤖 STARTING ROBOT PRODUCTION INSTALLATION (FULL PERCEPTION STACK)..."

# --- 1. System Updates ---
echo "📦 Updating System..."
sudo apt update && sudo apt upgrade -y

# --- 2. Install Audio & Video Drivers (Hardware Layer) ---
echo "🎤 Installing Audio/Video Drivers..."
sudo apt install -y \
    python3-pyaudio \
    portaudio19-dev \
    libgstreamer1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-alsa \
    v4l-utils \
    espeak \
    libespeak-dev \
    ffmpeg  # Required for audio file conversion

# --- 3. Install ROS 2 Build Tools ---
echo "🛠️ Installing Build Tools..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# --- 4. Install Python Libraries (Brain Layer) ---
echo "🐍 Installing Python Dependencies..."
# Note: numpy<2.0 is strict requirement for ROS 2 Humble
pip3 install "numpy<2.0" \
    opencv-python \
    opencv-contrib-python \
    pyaudio \
    pyttsx3 \
    SpeechRecognition \
    setuptools==58.2.0  # Downgrade fixes common build warnings in Humble

# --- 5. Fix USB/Hardware Permissions ---
echo "🔒 Setting up Hardware Permissions..."
# Allows robot to access Camera (video), Mic (audio), and ESP32 (dialout)
sudo usermod -aG audio $USER
sudo usermod -aG video $USER
sudo usermod -aG dialout $USER

# Reload rules to apply changes immediately
sudo udevadm control --reload-rules && sudo udevadm trigger

# --- 6. Initialize ROS Dependencies ---
echo "📚 Checking ROS Dependencies..."
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    sudo rosdep init
fi
rosdep update
# Install any missing standard ROS packages
rosdep install --from-paths src --ignore-src -r -y

# --- 7. Clean & Build the Robot ---
echo "🏗️ Building the Workspace..."
# We remove old builds to ensure a clean slate
rm -rf build install log
colcon build --symlink-install

# --- 8. Setup Environment ---
echo "🌍 Setting up Environment..."
# Check if source command exists in bashrc
if ! grep -q "source $HOME/delivery_bot_ws/install/setup.bash" ~/.bashrc; then
    echo "source $HOME/delivery_bot_ws/install/setup.bash" >> ~/.bashrc
    echo "✅ Added workspace sourcing to ~/.bashrc"
else
    echo "✅ Workspace sourcing already exists in ~/.bashrc"
fi

echo "=========================================="
echo "✅ ROBOT DEPLOYMENT COMPLETE!"
echo "👉 To apply changes: source ~/.bashrc"
echo "👉 To start robot: ros2 launch robot_bringup_pkg perception.launch.py"
echo "=========================================="