#!/bin/bash
echo "Setting up udev rules for Serial Ports..."
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Done! Permissions for /dev/ttyUSB* are now 666 (Read/Write for all)."
echo "You may need to unplug and replug the USB devices."
