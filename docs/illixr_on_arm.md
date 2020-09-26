# ILLIXR on ARM

ILLIXR runs on Ubuntu 18.04, while the most recent publicly available Ubuntu for ARM64 architecture is 16.04.<br>
Therefore, we will cross-compile an Ubuntu 18.04 for ARM and run ILLIXR on it. This instruction mainly contains the following parts:
- Cross-compile Ubuntu 18.04 for ARM64
- Install and setup the operating system
- Install and run ILLIXR

## Cross-compile Ubuntu 18.04

We name the host machine where we cross-compile the operating system and the target machine where the ILLIXR runs.<br>
The following are the details about our configuration:

- Host Machine:
    - OS: Ubuntu 18.04
    - ISA: x86-64
- Target Machine:
    - OS: Ubuntu 18.04
    - ISA: ARM64

---
#### * *Run the following commands on the host machine (x86-64)*:
---

1. Install the packages required for the build:
    ```
    sudo apt-get install -y ccache python-pip build-essential kernel-package fakeroot libncurses5-dev libssl-dev gcc git-core gnupg binfmt-support qemu qemu-user-static debootstrap simg2img
    ```
2. Clone repo and run the build script:
    ```
    cd ~/
    git clone https://github.com/Bigcountry907/Bionic-Builder.git
    cd ~/Bionic-Builder
    sudo ./BB.sh
    ```
    
Now you will see the main menu of the `Bionic-Builder System`.

3. Config and build:
    - (1) Create minimal base root filesystem:
        - Select option 1 by input `1`.
        - Set username and password for logging into the compiled system.
        - Select the first mirror to download packages.
        - Set access point name and password for the internet connection.
    - (2) Build Kernel Linux v4.9.78:
        - Select option 2 by input `2`.
        - Select option B to automatically perform all kernel build options.
        - After the finish of build, make sure the kernel `Image-hikey970-v4.9.gz` and the device tree `kirin970-hikey970.dtb` in the following path:
            ```~/Bionic-Builder/Install/kernel-install/```
    - (3) Copy and install kernels:
        - Select option 3 by input `3`.
    - (4) Generate flashable and compressed images:
        - Select option 4 by input `4`.
    - (5) Exit `Bionic-Builder System`:
        - Select option 99 by input `99`.

## Install the Operating System
4. Flash the compiled Ubuntu 18.04 to an ARM board:
    - (1) Set the switches on the dip block to the following status: 
        |   1   |   2   |   3   |   4   |
        |  ---- | ----  | ----  | ----  |
        |   ON  |  OFF  |  ON   |  OFF  |
    - (2) Connect the USB-C cable to the port next to the HDMI port.
    - (3) Power on the board.
    - (4) Change directory of the host machine to `~/Bionic-Builder/Install`.
    - (5) Make sure the board is correctly detected (expected output contains device ID and `fastboot`):
        ```
            sudo fastboot devices
        ```
    - (6) Install necessary tools:
        ```
            sudo apt-get install android tools-adb android-tools-fastboot
        ```
    - (7) Flash the board:
        ```
            sudo ./update_Hikey970.sh
        ```
    - (8) Power off the board when flashing is finished.

---
#### * *Run the following commands on the target machine (ARM64)*:
---

5. Post-install setup:
    - (1) Set the switches on the dip block to the following states: 
        |   1   |   2   |   3   |   4   |
        |  ---- | ----  | ----  | ----  |
        |   ON  |  OFF  |  OFF  |  OFF |
    - (2) Connect the board to a monitor by the HDMI cable.
    - (3) Power on the board, input the username and password you set when building the kernel.
    - (4) List and connect to an available access point:
        ```
            nmcli dev wifi list
            nmcli d wifi connect <essid> password <password>
        ```
    - (5) Run the following commands to finish setup:
        ```
            sudo /etc/./init.sh
        ```
        Please ignore any error and select `Yes` during the setup.<br>
        Make sure to select `Yes` for installing `Ubuntu-desktop`, otherwise the Ubuntu will not have graphics mode.

    - (6) Switch to the graphics mode.
        ``` 
            sudo service lightdm start
        ```

<!--
## Setup the Operating System
1. Update Python3 to Python3.8
    - (1) Install Python3.8:
        ```
            sudo apt update -y
            sudo apt install python3.8
        ```
    - (2) Point Python3 to Python3.8:
        ```
            sudo rm /usr/bin/python3
            sudo ln -s python3.8 /usr/bin/python3
        ```
    
2. Update CMake to CMake3.18
    - (1) Download the source file:
        ```
            cd ~/Download
            wget https://github.com/Kitware/CMake/releases/download/v3.18.3/cmake-3.18.3.tar.gz
        ```
    - (2) Unzip the file:
        ```
            tar -xzvf cmake-3.18.3.tar.gz
        ```
    - (3) Install CMake3.18:
        ```
            ./bootstrap && make && sudo make install
        ```

3. Install dependencies
    - (1) Install pip3 for Python3.8:
        ```
            sudo apt install pip3
        ```
    - (2) Use pip3 to install dependencies listed [here](https://github.com/ILLIXR/ILLIXR/blob/issue-136-illixr-on-arm/runner/environment.yml)
        ```
            pip3 install <package_name>
        ```
-->

## Install and Run ILLIXR
1. Clone the repository:
    ```
        git clone --recursive --branch issue-136-illixr-on-arm https://github.com/ILLIXR/ILLIXR
    ```

2. Update the submodules. Submodules are git repositories inside a git repository that needs to be
   pulled down separately:
    ```
        git submodule update --init --recursive
    ```

3. Install dependencies. This script installs some Ubuntu/Debian packages and builds a specific version of OpenCV from source:
    ``` 
        ./install_deps.sh
    ```

4. Inspect `configs/native.yaml`. The schema definition (with documentation inline) is in `runner/config_schema.yaml`.


5. Build and run ILLIXR standalone:
    ```
        ./runner.sh configs/native.yaml
    ```
