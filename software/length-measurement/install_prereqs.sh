#!/usr/bin/env bash
set -e

# install_prereqs.sh
# Script to install development prerequisites for zed-open-capture examples

sudo apt-get update

sudo apt-get install -y build-essential

sudo apt-get install -y cmake

sudo apt-get install -y libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev

sudo apt-get install -y libopencv-dev libopencv-viz-dev

cd udev
bash install_udev_rule.sh
cd ..

echo "All prerequisites installed successfully."

