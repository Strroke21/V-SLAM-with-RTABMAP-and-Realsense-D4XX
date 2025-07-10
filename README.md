# ROS 2 Humble Setup Guide (Ubuntu 22.04 LTS)

## I. ROS 2 Humble Setup or run

```bash
    ./install.sh
```

### **1. Set Locale**

Before installing ROS 2, ensure your system locale is set to UTF-8.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### **2. Setup Sources**

To add the ROS 2 apt repository, enable the Ubuntu Universe repository.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### **3. Install ROS 2 Packages**

Update your apt repository caches and install ROS 2 packages.

```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
```

Install development tools.

```bash
sudo apt install ros-dev-tools
```

### **4. Environment Setup**

To start working with ROS 2, source the setup script in each terminal session.

```bash
source /opt/ros/humble/setup.bash
```

### **5. Try Some Examples**

**Talker-Listener Example**

Open a terminal and start a talker node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py talker
```

Open another terminal and start a listener node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

## II. OpenCV 4.8.0+ Installation for Librealsense

```bash
# Install dependencies
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
```

## III. Camera Node Setup

### **1. Create a Workspace**

```bash
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
```

### **2. Clone the `realsense_ros` Repository**

```bash
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros2

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **3. Build the Workspace**

```bash
cd ~/realsense_ws
colcon build --symlink-install
```

Source the newly built workspace:

```bash
source ~/realsense_ws/install/setup.bash
```

To make it persistent, add it to your `.bashrc`:

```bash
echo "source ~/realsense_ws/install/setup.bash" >> ~/.bashrc
```

### **4. Launch Camera Node**

To test the installation, launch the RealSense camera node:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  depth_module.depth_profile:=640,480,60 \
  rgb_camera.color_profile:=640,480,60
  align_depth:=true

```

## IV. RTAB-Map Installation

### **1. Install RTAB-Map**

```bash
sudo apt install -y ros-humble-rtabmap-ros
```

### **2. Launch RTAB-Map**

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link \
  use_sim_time:=true \
  approx_sync:=true \
  qos:=2 \
  rviz:=false \
  queue_size:=100 
  wait_for_transform:=true \
  transform_timeout:=2.0
```

### **3. Ardupilot Parameter setup**

```bash
SERIAL1_PROTOCOL = 2 (MAVLink2).

SERIAL1_BAUD = 115 (115200 baud)

VISO_TYPE = 1 (mavlink)

EK3_SRC1_POSXY = 6 (ExternalNav)

EK3_SRC1_VELXY = 6 (ExternalNav)

EK3_SRC1_POSZ = 1 (Baro which is safer)

EK3_SRC1_VELZ = 6 (you can set it to 6 if 1st flight test looks good)

EK3_SRC1_YAW = 6 (ExternalNav)


```

### **4. Run Slam node (ardupilot)**

```bash
#for python
python3 slam_localization.py 

# for cpp run
colcon build --packages-select slam_localization --symlink-install #once
source install/setup.bash
ros2 run slam_localization slam_node
 
```

### **5. To see Saved maps Run

```bash
rtabmap ~/.ros/rtabmap.db
```
