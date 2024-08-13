# Video Demo

<p align="center">
  <img src="./img/1.webp" autoplay loop="loop" style="width: 50%" controls>
</p>

# Introduction

This repository implements teleoperation of the humanoid robot Unitree H1 using Azure Kinect DK camera.

# Prerequisites

We tested our code on Ubuntu 20.04, other operating systems may be configured differently.

For more information, you can refer to [Official Documentation](https://support.unitree.com/home/zh/Teleoperation/kinect_teleoperate) and  [Azure Kinect DK Code Samples](https://github.com/microsoft/Azure-Kinect-Samples).

## Azure Kinect configuration

The Azure Kinect DK camera driver consists of two parts:  **camera SDK** and **body tracking SDK**.

### 1. camera SDK

Developers can manually download the latest version 1.4.1 SDK installation package from the [Microsoft packages website](https://packages.microsoft.com/ubuntu/18.04/) : `libk4a1.4_1.4.1_amd64.deb` , `libk4a1.4-dev_1.4.1_amd64.deb` and `k4a-tools_1.4.1_amd64.deb`。

For convenience, use the `wget` command to download directly. After the download is complete, proceed with the installation in sequence :

```bash
cd ~/Downloads
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb
sudo dpkg -i libk4a1.4_1.4.1_amd64.deb
sudo dpkg -i libk4a1.4-dev_1.4.1_amd64.deb
sudo dpkg -i k4a-tools_1.4.1_amd64.deb
```

Note that in order to use the Azure Kinect SDK on the device without needing to be a "root" user, [you need to set udev rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup). The command is as follows :

```bash
cd ~
wget https://github.com/microsoft/Azure-Kinect-Sensor-SDK/raw/develop/scripts/99-k4a.rules
sudo mv ~/99-k4a.rules /etc/udev/rules.d/
# Disconnect and reconnect the Azure Kinect device (if it was connected during this process).
```

After completing the above steps and connecting the camera hardware, use the following command to open the image visualization interface of the Azure Kinect camera :

```
k4aviewer
```

The result is shown in the image below :

<center>
<img src="https://oss-global-cdn.unitree.com/static/b664417592fc49dd9a5c875e2cd82e7a_1691x1041.png"  width=85%>
</center>




### 2. body tracking SDK

This example is tested on the Ubuntu 20.04 operating system. Developers can manually download the latest version 1.1.2 installation package from the [Microsoft packages website](https://packages.microsoft.com/ubuntu/20.04/) : `libk4abt1.1_1.1.2_amd64.deb` and `libk4abt1.1-dev_1.1.2_amd64.deb` 。

If you are using the Ubuntu 18.04 operating system, you can switch to the `/ubuntu/18.04/` path to download the corresponding installation package.

For convenience, use the `wget` command to download directly. After the download is complete, proceed with the installation in sequence :

```bash
# Some dependencies that might be needed:
sudo apt-get install libxi-dev
sudo apt-get install libxinerama-dev
sudo apt-get install libxcursor-dev
sudo apt-get install libsoundio1
# Begin the installation of the Body Tracking SDK.
cd ~/Downloads
wget https://packages.microsoft.com/ubuntu/20.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb
wget https://packages.microsoft.com/ubuntu/20.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb
sudo dpkg -i libk4abt1.1_1.1.2_amd64.deb
sudo dpkg -i libk4abt1.1-dev_1.1.2_amd64.deb
# After completion, you can use the `locate` command to check if the CMake configuration files exist in the path.
locate k4abtConfig.cmake
# The output result of a successful installation is as follows:
/usr/lib/cmake/k4abt/k4abtConfig.cmake
```

After completing the above steps, connect the camera device using the **USB 3.0** interface. Use the following command to open the Body Tracking visualization interface of the camera (Note: In the visualization interface, press the `h` key to display the help information in the command line) :

```
k4abt_simple_3d_viewer
```

The result is shown in the image below :

<center>
<img src="https://oss-global-cdn.unitree.com/static/ad01fc07be9a4607a2285614327fc7d3_1028x972.png" width=35%>
</center>

### 3. firmware version (optional)

The firmware version information of the Azure Kinect DK camera during this test is as follows:

```bash
# Note: You can view the command usage information by directly entering the `AzureKinectFirmwareTool` command.
AzureKinectFirmwareTool -q
# The command output information is shown below:
== Azure Kinect DK Firmware Tool == 
Device Serial Number: 000348604312
Current Firmware Versions:
  RGB camera firmware:      1.6.110
  Depth camera firmware:    1.6.80
  Depth config file:        6109.7
  Audio firmware:           1.6.14
  Build Config:             Production
  Certificate Type:         Microsoft
```

If developers want to update their camera device to the latest firmware version, they can follow these steps :

```bash
# Download the latest firmware binary files from this link: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/firmware
cd ~/Downloads
wget https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/firmware/AzureKinectDK_Fw_1.6.110080014.bin
# With the device connected, use the `AzureKinectFirmwareTool` command to load the binary files and update the device firmware.
AzureKinectFirmwareTool -u AzureKinectDK_Fw_1.6.110080014.bin
```

For more information, please refer to [Update Azure Kinect DK firmware](https://learn.microsoft.com/en-us/azure/kinect-dk/update-device-firmware).

## Environment configuration

The code in this repository is modified from the [Azure Kinect DK Code Samples](https://github.com/microsoft/Azure-Kinect-Samples).

This repository uses the [Mujoco](https://mujoco.org/) physics engine for simulation testing, with version [3.1.5](https://github.com/google-deepmind/mujoco/releases/download/3.1.5/mujoco-3.1.5-linux-x86_64.tar.gz). 

For convenience, the Mujoco repository files have already been placed in the `src/mujoco-3.1.5` directory of this example, so developers do not need to download them manually.

The Unitree H1 `XML` model file needed for the Mujoco simulation environment has been placed in the `src/unitree_h1` directory of this repository.

If developers want to visualize the model file using Mujoco for research purposes, they can proceed with the following command :

```bash
# Install the Python version of Mujoco:
pip install mujoco
# Open the Mujoco viewer:
python -m mujoco.viewer 
```

After dragging the MJCF model file (`src/unitree_h1/mjcf/scene.xml`) into the MuJoCo viewer window with your mouse, the result is shown in the image below :

<center>
<img src="https://doc-cdn.unitree.com/static/2024/7/25/b3b26e125996448ea8e850e8516689de_1726x1015.png" width=720>
</center>


This example uses CMake + [ninja](https://github.com/ninja-build/ninja) to build the project. Therefore, in addition to installing CMake, you also need to install ninja. You can do this by running the following command :

```bash
sudo apt-get install build-essential
sudo apt-get install cmake
sudo apt-get install ninja-build
```

Additionally, you need to install the [Eigen3](https://eigen.tuxfamily.org/index.php) library to handle some mathematical computations.

```bash
sudo apt install libeigen3-dev
```

Clone this repository and compile the code :

```bash
cd ~
git clone https://github.com/unitreerobotics/kinect_teleoperate.git
cd kinect_teleoperate
mkdir build
cd build
cmake .. -GNinja
# Use ninja to compile the program:
ninja
```

After successful compilation, you can find the generated executable file `kinect_teleoperate` in the `build` directory.

# Usage

Please read the [Official Documentation ](https://support.unitree.com/home/zh/Teleoperation/kinect_teleoperate)at least once before starting this program.

Start the teleoperation executable program in the `kinect_teleoperate/build` path :

```bash
./kinect_teleoperate

# The following is the output information after the program is executed
 Basic Navigation:

 Rotate: Rotate the camera by moving the mouse while holding mouse left button
 Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button
 Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel
 Select Center: Center the scene based on a detected joint by right clicking the joint with mouse

 Key Shortcuts

 ESC: quit
 h: help
 b: body visualization mode
 k: 3d window layout

control loop start...
MujocoRender loop start...
KinectRender loop start...
Please use the wake-up action to start or stop the TeleOperation...
```

After the program is started, two visualization windows pop up on the desktop, as shown below :

<center>
<img src="https://oss-global-cdn.unitree.com/static/681684ac9f2e42ba896b9ea1b1edc754_2386x912.png" width=720>
</center>


## Wake-up action

If you execute the teleoperation function immediately without any preparation, it may cause accidents and dangers. 

To avoid this, the corresponding **wake-up action** is required after the program is started to start and stop the teleoperation function.

- **Wake-up action description**: The teleoperator stands naturally after facing the Azure Kinect DK camera, with the upper arm hanging naturally, the forearm facing the front and the upper arm at a vertical angle of about 90 degrees. Keep this state for a few seconds to switch the teleoperation start or stop state.

- **Wake-up action diagram**:

  Please refer to the corresponding section in the [Official Documentation](https://support.unitree.com/home/zh/Teleoperation/kinect_teleoperate).

When the program successfully recognizes the wake-up action and outputs `START.` in the command line terminal, the function is turned on and teleoperation can be performed at this time.

When the wake-up action is performed again and successfully recognized, the command line terminal outputs `END.`, and the teleoperation function is disabled at this time.

# Codebase Tutorial

```
├── CMakeLists.txt       [cmake config]
├── lib                  [Backup of relevant library files]
└── src
    ├── CMakeLists.txt
    ├── extern           [GLFW and other third-party libraries]
    ├── kinect_teleoperate_robot
    │   ├── CMakeLists.txt
    │   ├── include           
    │   │   ├── jointRetargeting.hpp      [used to retarget skeletal joint angles to motor joints]
    │   │   ├── math_tool.hpp             [filters and quaternion conversion tools]
    │   │   └── StartEndPoseDetector.hpp  [wake-up action detection code]
    │   └── main.cpp                      [main program code]
    ├── mujoco-3.1.5                      [Mujoco library]
    ├── sample_helper_includes
    ├── sample_helper_libs
    ├── unitree_g1                        [Unitree g1 URDF]
    └── unitree_h1                        [Unitree h1 URDF]
```

# Acknowledgement

1. https://github.com/microsoft/Azure-Kinect-Samples
2. https://github.com/microsoft/Azure-Kinect-Sensor-SDK
3. https://ninja-build.org/
4. https://mujoco.org/
5. https://eigen.tuxfamily.org
6. https://github.com/glfw/glfw