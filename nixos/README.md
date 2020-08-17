# ILLIXR NixOS

### Building QEMU image
Use the following command to build the QEMU image:
```
nix-build '<nixpkgs/nixos>' -A config.system.build.qcow2 --arg configuration "{ imports = [ ./build-qcow2.nix ]; }"
```

### Running QEMU image
Use the following command to run the QEMU image:
```
sudo qemu-system-x86_64 \
  -m 4G \     
  -vga virtio \
  -show-cursor \
  -usb \
  -device usb-tablet \
  -enable-kvm \
  -drive file=nixos.qcow2,if=virtio \
  -cpu host
```
