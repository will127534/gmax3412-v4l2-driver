# Minimum Driver for GMAX3412

GMAX3412 requires external triger to generate the frame, such that it can't really drive itself like most of the MIPI camera sensors, as such the driver here is missing critical V4L2 controls like V4L2_CID_VBLANK, V4L2_CID_HBLANK and V4L2_CID_EXPOSURE and the expect is that the users will generate the pulses externally via other means.  
The way this driver currently setup is with external exposure, which means exposure and framerate is determined by a external pulse frequency and the level high duration, and the driver won't do anything about the controls for V4L2_CID_VBLANK, V4L2_CID_HBLANK and V4L2_CID_EXPOSURE. 
It will still expose them to the upper layer because some applications (rpicam and libcaemra) requires these minimum controls.  
  
The driver requires 4-lane MIPI, as I did not see a way to use 2 or 1 lane from the leaked(?) datasheet.
Additionally it only supports 12bit 4K (4096x3072) mode, if anyone has a more up to date version of the datasheet, feel free to open an issue and send the datasheet.  

Overall this is a extension from my gmax4002 basic minimum driver - just modified for gmax3412.  

## Working platform
The code is tested with RPI5 with either Analog discovery 2 or MCU as pulse generator, see the camera board repo [here](https://github.com/will127534/GlobalEye) for MCU code and camera board. The libcamera support has been added to my libcamera fork [here](https://github.com/will127534/libcamera).  

<img width="1280" alt="image" src="https://github.com/user-attachments/assets/53eb4a42-8ea5-4f12-b764-6d9b37767cd4" />
<img width="1280" alt="image" src="https://github.com/user-attachments/assets/5b95bd54-53f1-42e3-ba69-38ca70e62af9" />

See it in action here: [Youtube](https://www.youtube.com/watch?v=J_Mvx6Y6Drg).  
The board is tested with Raspberry Pi 5.  

The trigger signal looks like this:  
<img width="1280" alt="image" src="https://github.com/user-attachments/assets/56f11915-6237-4873-89a1-e6653319db5b" />
Yellow is the TEXP, and blue one is the TDIG output from the sensor showing "Frame Overhead Time".


## Prerequisites

Before you begin the installation process, please ensure the following prerequisites are met:

- **Kernel version**: You should be running on a Linux kernel version 6.12 or newer. You can verify your kernel version by executing `uname -r` in your terminal.

- **Development tools**: Essential tools such as `gcc`, `dkms`, and `linux-headers` are required for compiling a kernel module. If not already installed, these can be installed using the package manager with the following command:
  
   ```bash 
   sudo apt install linux-headers dkms git
   ```
   
## Installation Steps

### Setting Up the Tools

First, install the necessary tools (`linux-headers`, `dkms`, and `git`) if you haven't done so:

```bash 
sudo apt install linux-headers dkms git
```

### Fetching the Source Code

Clone the repository to your local machine and navigate to the cloned directory:

```bash
git clone https://github.com/will127534/gmax3412-v4l2-driver.git
cd gmax3412-v4l2-driver/
```

### Compiling and Installing the Kernel Driver

To compile and install the kernel driver, execute the provided installation script:

```bash 
./setup.sh
```

### Updating the Boot Configuration

Edit the boot configuration file using the following command:

```bash
sudo nano /boot/config.txt
```

In the opened editor, locate the line containing `camera_auto_detect` and change its value to `0`. Then, add the line `dtoverlay=gmax3412`. So, it will look like this:

```
camera_auto_detect=0
dtoverlay=gmax3412
```

After making these changes, save the file and exit the editor.

Remember to reboot your system for the changes to take effect.

## dtoverlay options

### cam0

If the camera is attached to cam0 port, append the dtoverlay with `,cam0` like this:  
```
camera_auto_detect=0
dtoverlay=gmax3412,cam0
```

### always-on

If you want to keep the camera power always on (Useful for debugging HW issues, specifically this will set CAM_GPIO to high constantly), append the dtoverlay with `,always-on` like this:  
```
camera_auto_detect=0
dtoverlay=gmax3412,always-on
```
