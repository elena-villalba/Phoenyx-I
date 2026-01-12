# 🍓 Raspberry Pi Setup Guide  
**Phoenyx I – On-board Computer Installation**

This guide explains how to set up the **Raspberry Pi** used on **Phoenyx I**, starting from a **clean SD card** up to **remote development via Visual Studio Code**, without needing a screen, keyboard, or mouse connected to the Raspberry Pi.

The target system is optimized for **ROS 2 Humble** running on **Ubuntu Server 22.04 LTS**.

---

## 📋 Requirements

### Hardware
- Raspberry Pi 4B (≥ 4 GB RAM recommended)
- microSD card (≥ 32 GB recommended)
- Stable internet connection (Ethernet or WiFi)

### Host Computer
- Windows / Linux / macOS
- Visual Studio Code installed

---

## 1. Install the Operating System

### 1.1. Raspberry Pi Imager

1. Download and install **Raspberry Pi Imager** from the official website:  
   https://www.raspberrypi.com/software/

2. Insert the microSD card into your computer.

3. Open **Raspberry Pi Imager** and configure:
   - **Device**: Raspberry Pi 4
   - **Operating System**:  
     `Other general-purpose OS → Ubuntu → Ubuntu Server 22.04.05 LTS (64-bit)`
   - **Storage**: your microSD card

---

### 1.2 Enable SSH & User Configuration (⚠️IMPORTANT⚠️)

Before flashing the SD card, you must configure the Raspberry Pi **advanced options**.  
This step is critical to enable **headless operation** (no screen, keyboard, or mouse).

1. In **Raspberry Pi Imager**, press **`Ctrl + Shift + X`** to open **Advanced Options**.
2. Configure the following settings:
   - ✅ **Set hostname**  
     Hostname: `phoenyxI`
   - ✅ **Set username and password**  
     Username: `phoenyxI`  
     Password: *(choose a secure password)*
   - ✅ **Configure wireless LAN** 
     - SSID: *your WiFi network name*  
     - Password: *your WiFi password*
   - ✅ **Set locale settings**  
     - Time zone: *your local timezone*  
     - Keyboard layout: *your preferred layout*
   - ✅ **Enable SSH**  
     - Select **Enable SSH**
     - Authentication: **Password authentication**

3. Click **Save**, then **Write** to flash the SD card with the configured settings.

---

### 1.3 First Boot

1. Insert the microSD card into the Raspberry Pi.
2. Power it on.
3. Wait ~1–2 minutes until the system boots.

No screen or peripherals are required.

---

## 2. Connect to the Raspberry Pi via SSH

Make sure your **host computer** is on the same network.

```bash
$ ssh phoenyxI@phoenyxI.local
```

## 3. Remote Development with Visual Studio Code

To avoid working directly in a terminal, we strongly recommend using **Visual Studio Code Remote SSH**.

---

### 3.1. Install VS Code Extension

1. Open **Visual Studio Code**.
2. Go to **Extensions**.
3. Search and install: **Remote - SSH**


---

### 3.2. Configure a New Remote Connection

1. Once installed, a new **Remote Explorer** icon appears on the **left sidebar**.
2. Click it.
3. Press the **`+`** button to add a new remote connection.
4. Enter the SSH target:

   ```text
   phoenyxI@phoenyxI.local
    ```
5. Select the default SSH config file when prompted.

---

### 3.3. Connect to the Raspberry Pi

1. In the Remote Explorer panel, click phoenyxI@phoenyxI.local.
2. When asked, enter the password, the same you put during the SO instalation
3. After a short setup process, VS Code will open a remote session directly on the Raspberry Pi.

---

##  4. Install Phoenyx I Software Stack

At this point, the Raspberry Pi is fully configured and accessible via SSH and Visual Studio Code.

The complete software stack (ROS 2, system dependencies, drivers, and workspace setup) is installed by running a single script. 

1. Download the installation script
   ```bash
   $ wget https://raw.githubusercontent.com/PUCRA/Phoenyx-I/refs/heads/main/scripts/install_rpi.sh
   ```
2. Make the installation script executable:
   ```bash
   $ chmod +x install_rpi.sh
   ```
3. Run the script
   ```bash
   $ ./scripts/install_rpi.sh
   ```
   This script automatically:

   - Installs ROS 2 Humble
   - Installs all required system and ROS dependencies
   - Creates the ROS 2 workspace
   - Clones the Phoenyx I repository
   - Configures hardware drivers and permissions

   ⏳ The installation may take several minutes and may require a system reboot once completed.

## 5. Setting Up Serial and I2C Communication on the Raspberry Pi

Phoenyx I communicates with the motor controllers and sensors via **UART (serial)** and **I2C** buses.  
These interfaces must be enabled and properly configured on the Raspberry Pi.

---

### 5.1. Enable Serial and I2C Interfaces

1. Install and launch the Raspberry Pi configuration tool:

   ```bash
   $ sudo apt-get install raspi-config
   $ sudo raspi-config
   ```

2. Navigate through the menu: `Interface Options → Serial Port`

   You will be asked:

   - Would you like a login shell to be accessible over serial?

      → Select 'No' using the arrow keys and 'tab' and 'enter' keys on your keyboard

   - Would you like the serial port hardware to be enabled?
      
      → Select 'Yes'

3. Enable I2C: `Interface Options → I2C → Enable`

   If prompted to reboot, select 'Yes'.

   After rebooting, reconnect to the Raspberry Pi via SSH.

---

### 5.2. Add User to Required System Groups
1. Add the current user to the required groups:
   ```bash
   $ sudo adduser $USER tty
   $ sudo adduser $USER dialout
   $ sudo adduser $USER input
   ```

   If the `dialout` group does not exist, create it first:
   ```bash
   $ sudo groupadd dialout
   ```
2. Reboot for group changes to take effect.

   ```bash
   $ sudo reboot
   ```

3. Verify serial devices

   After logging back in, verify that the serial devices are available:
   ```bash
   $ ls -l /dev/serial*
   ```

   You should see at least: `/dev/serial0 -> ttyS0`. This is the primary UART interface used to communicate with the Roboclaw motor controllers via GPIO pins.

   If you see: `/dev/serial1 -> ttyAMA0`. This corresponds to a secondary, software-defined serial interface (often used for Bluetooth).
   The exact mapping may vary depending on the Raspberry Pi model and configuration.

---
➡️ After the script finishes, follow the instructions in the main README.md to run the system in simulation or on the real robot.
