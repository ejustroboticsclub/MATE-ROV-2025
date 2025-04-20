#!/usr/bin/env bash
set -e

# build_and_run_receiver.sh
# Builds, installs, and runs the length measurement receiver tool

cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig

zed_open_capture_depth_tune_stereo
