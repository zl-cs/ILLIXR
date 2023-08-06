# Building ILLIXR on Jetson
This documentation has been validated on the offload\_rebase and vulkan-offload branch of ILLIXR as of Aug 5, 2023 with Jetson Xavier AGX board.

## Tools you will need:
1. Nvidia SDK Manager [https://developer.nvidia.com/sdk-manager][1] (tested version 1.9.3) on a separate machine
2. Nvidia JetPack (tested version 5.1.2)
if you want run Vulkan-based on ILLIXR:
3. Google Shaderc [https://github.com/google/shaderc][2] 
4. Vulkan validation layers




## Steps  
1. On a separate machine, install Nvidia SDK Manager 
2. Log into SDK Manger and flash Jetson with JetPack 
3. Log into your Jetson, install library dependencies for ILLIXR (best to install on SDCard since Jetson will quickly ran out of storage space)
4. Modify the following plugins:
    - ground\_truth\_slam:
        -Add LDFLAGS=-lstdc++fs
        - 
    - 
