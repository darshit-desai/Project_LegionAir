# VOXL 2 Drone Documentation

## 1. System Information
| Component | Version |
|-----------|---------|
| System Image | 1.7.1-M0054-14.1a-perf-nightly-20231025 |
| Kernel | 4.19.125 |
| Hardware Version | M0054 |
| VOXL Suite | 1.1.2 |

## 2. Software Components

### Core Libraries
| Library | Version |
|---------|---------|
| libmodal-cv | 0.4.0 |
| libmodal-exposure | 0.1.0 |
| libmodal-journal | 0.2.2 |
| libmodal-json | 0.4.3 |
| libmodal-pipe | 2.9.2 |
| libqrb5165-io | 0.4.2 |
| libvoxl-cci-direct | 0.2.1 |
| libvoxl-cutils | 0.1.1 |

### System Services
| Service | Version |
|---------|---------|
| qrb5165-bind | 0.1-r0 |
| qrb5165-dfs-server | 0.2.0 |
| qrb5165-imu-server | 1.0.1 |
| qrb5165-rangefinder-server | 0.1.1 |
| qrb5165-system-tweaks | 0.2.3 |
| qrb5165-tflite | 2.8.0-2 |

### Camera and Vision
| Component | Version |
|-----------|---------|
| voxl-camera-calibration | 0.5.3 |
| voxl-camera-server | 1.8.9.1 |
| voxl-feature-tracker | 0.3.2 |
| voxl-flow-server | 0.3.3 |
| voxl-gphoto2-server | 0.0.10 |
| voxl-jpeg-turbo | 2.1.3-5 |
| voxl-lepton-server | 1.2.0 |
| voxl-libgphoto2 | 0.0.4 |
| voxl-libuvc | 1.0.7 |
| voxl-opencv | 4.5.5-2 |
| voxl-tag-detector | 0.0.4 |
| voxl-tflite-server | 0.3.2 |
| voxl-uvc-server | 0.1.6 |
| voxl-vision-hub | 1.7.3 |

### Flight Control and Navigation
| Component | Version |
|-----------|---------|
| voxl-px4 | 1.14.0-2.0.63 |
| voxl-px4-imu-server | 0.1.2 |
| voxl-px4-params | 0.3.3 |
| voxl-qvio-server | 1.0.0 |

### Communication and Telemetry
| Component | Version |
|-----------|---------|
| voxl-mavlink | 0.1.1 |
| voxl-mavlink-server | 1.3.2 |
| voxl-microdds-agent | 2.4.1-0 |
| voxl-modem | 1.0.8 |
| voxl-remote-id | 0.0.9 |

### ROS Integration
| Component | Version |
|-----------|---------|
| voxl-mpa-to-ros | 0.3.7 |
| voxl-mpa-to-ros2 | 0.0.2 |
| voxl-ros2-foxy | 0.0.1 |

### Utilities and Tools
| Component | Version |
|-----------|---------|
| voxl-bind-spektrum | 0.1.0 |
| voxl-configurator | 0.4.8 |
| voxl-cpu-monitor | 0.4.7 |
| voxl-docker-support | 1.3.0 |
| voxl-elrs | 0.1.3 |
| voxl-esc | 1.3.7 |
| voxl-logger | 0.3.5 |
| voxl-mavcam-manager | 0.5.3 |
| voxl-mpa-tools | 1.1.3 |
| voxl-neopixel-manager | 0.0.3 |
| voxl-portal | 0.6.3 |
| voxl-streamer | 0.7.4 |
| voxl-utils | 1.3.3 |

### System Components
| Component | Version |
|-----------|---------|
| mv-voxl | 0.1-r0 |
| voxl2-system-image | 1.7.1-r0 |
| voxl2-wlan | 1.0-r0 |

## 3. Package Repository Information
- **Repository URL**: http://voxl-packages.modalai.com/ ./dists/qrb5165/sdk-1.1/binary-arm64/
- **Last Updated**: 2024-05-07 05:41:15


## 4. Initial Setup Commands
Before editing any configuration files, ensure that the necessary services are set up and active.

### 4.1. Activate VOXL TFLite Server
```bash
voxl-configure-tflite
```

### 4.2. Activate VOXL Vision Hub
```bash
voxl-configure-vision-hub
```

### 4.3. Install and Configure ROS 2 Bridge (Optional)
1. Install ROS 2 Foxy:
   ```bash
   sudo apt-get install voxl-ros2-foxy
   ```
2. Install the MPA to ROS 2 Bridge:
   ```bash
   sudo apt-get install voxl-mpa-to-ros2
   ```
3. Configure the bridge:
   ```bash
   voxl-configure-mpa-to-ros2
   ```

---

## 5. VOXL TFLite Server Configuration

### Configuration File
**Path**: `/etc/modalai/voxl-tflite-server.conf`

**Default Content**:
```json
{
    "skip_n_frames": 0,
    "model": "/usr/bin/dnn/yolov5_float16_quant.tflite",
    "input_pipe": "/run/mpa/hires_small_grey/",
    "delegate": "gpu",
    "allow_multiple": false,
    "output_pipe_prefix": "yolov5"
}
```

### Steps to Modify
1. **Download Custom YOLOv5 Model**:
   - Model file: [YOLOv5n Model](https://drive.google.com/file/d/1vo8GY54IPJN9HCBmCY1uFKZN9OyJOBbt/view?usp=drive_link)
   - Place the model in `/usr/bin/dnn/` as `yolov5_float16_quant.tflite`.

2. **Modify Labels File**:
   - Download the custom labels file: [YOLOv5 Labels](https://drive.google.com/file/d/15HLPd0u_9tR3kiNVji2dv6fAaJNaa91T/view?usp=drive_link)
   - Place it in `/usr/bin/dnn/` as `yolov5_label.txt`.

---

## 6. VOXL Vision Hub Configuration

### Configuration File
**Path**: `/etc/modalai/voxl-vision-hub.conf`

### Steps to Modify
1. **Disable Automatic Flight Pattern**:
   - Change the `offboard_mode` parameter to `"off"` to prevent the automatic figure-8 flight pattern:
     ```json
     {
         "offboard_mode": "off"
     }
     ```

2. **Restart the Vision Hub Service**:
   ```bash
   systemctl restart voxl-vision-hub
   ```

---

## 7. VOXL Camera Server Configuration

### Configuration File
**Path**: `/etc/modalai/voxl-camera-server.conf`

### Key Settings
1. **PMD ToF Camera FPS**:
   - Update the frame rate to 15 FPS if not already set.

2. **Set CPU Performance Mode**:
   ```bash
   voxl-set-cpu-mode performance
   ```

---

## 8. Multi-Drone Namespace Configuration

### Steps to Configure
1. **Download the Swarm Configuration Script**:
   ```bash
   wget https://drive.google.com/file/d/12gKvCccvBPMrTLuNX7L1iwBryBanQBTf/view?usp=drive_link -O swarm_script.sh
   ```

2. **Install the Script**:
   ```bash
   sudo mv swarm_script.sh /home/root/.profile.d/
   sudo chmod +x /home/root/.profile.d/swarm_script.sh
   ```

3. **Edit the Script**:
   - Open the script:
     ```bash
     sudo nano /home/root/.profile.d/swarm_script.sh
     ```
   - Set the namespace for your drone (e.g., `uav_1`):
     ```bash
     export UAV_NAMESPACE=uav_1
     ```

4. **Namespace-Specific Commands**:
   - **Run MPA to ROS 2 Bridge**:
     ```bash
     ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node --ros-args -r __ns:=/$UAV_NAMESPACE
     ```
   - **Run Docker with Namespace**:
     ```bash
     docker run -it --net=host --ipc=host --pid=host \
         -v /home/root:/root/yoctohome:rw \
         -e UAV_NAMESPACE=$UAV_NAMESPACE \
         my_custom_image:v1
     ```

---

## 9. Docker Installation and Image Loading

### Docker Installation
Refer to the [ModalAI Docker Guide](https://docs.modalai.com/voxl-docker-and-cross-installation/) for installation steps.

### Load Preconfigured Docker Image
1. **Download the Docker Image**:
   - Image file: [Docker Image](https://drive.google.com/file/d/1LCj9wKJXs4X5EDfoHEo6Jccn-JK_5Lxn/view?usp=drive_link)

2. **Load the Image**:
   ```bash
   docker load < [downloaded-tar-filename]
   ```
3. **Workspace Setup and Repository Configuration**:
   
   First, we need to set up a ROS 2 workspace inside the Docker container. Since we've mounted the VOXL2's home directory to `/root/yoctohome` in the Docker container, we'll create our workspace there.

   a. **Create Workspace Structure**:
   ```bash
   # Navigate to the mounted home directory
   cd /root/yoctohome

   # Create the ROS 2 workspace and its source directory
   mkdir -p n_ros2_ws/src
   ```

   b. **Configure Git** (Required only once):
   Before cloning the repository, you'll need to configure Git with your credentials. This is necessary for authentication:
   ```bash
   git config --global user.name "Your Name"
   git config --global user.email "your.email@example.com"
   ```

   c. **Clone Repository**:
   Navigate to the src directory and clone the Project_LegionAir repository:
   ```bash
   cd n_ros2_ws/src
   git clone https://github.com/darshit-desai/Project_LegionAir.git -b control_vision_update
   ```

   d. **Build the Workspace**:
   The Docker image comes pre-installed with ROS 2 Humble and necessary dependencies. Build the workspace using colcon:
   ```bash
   cd ..  # Return to workspace root
   colcon build
   ```

   e. **Source the Workspace**:
   After building, source the workspace to make the packages available:
   ```bash
   source install/setup.bash
   ```

   Note: You may want to add the source command to your `.bashrc` inside the Docker container for persistence:
   ```bash
   echo "source /root/yoctohome/n_ros2_ws/install/setup.bash" >> ~/.bashrc
   ```



## 10. ROS 2 Integration Overview
- **Bridge Topics**:
  - PX4/MPA topics appear as:
    - Publishers: `/fmu/out/*`
    - Subscribers: `/fmu/in/*`
- Uses `uXRCE-DDS` middleware for data exchange.

For additional details, refer to the [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide).
