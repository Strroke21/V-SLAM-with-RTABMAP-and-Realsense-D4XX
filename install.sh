
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y build-essential cmake pkg-config libjpeg-dev libtiff-dev libpng-dev libgtk-3-dev libcanberra-gtk* libatlas-base-dev gfortran python3-dev

# Download OpenCV
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.8.0.zip
unzip opencv.zip
cd opencv-4.8.0

# Build OpenCV
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src

git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros2

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/realsense_ws
colcon build --symlink-install

source ~/realsense_ws/install/setup.bash

echo "source ~/realsense_ws/install/setup.bash" >> ~/.bashrc

sudo apt install -y ros-humble-rtabmap-ros