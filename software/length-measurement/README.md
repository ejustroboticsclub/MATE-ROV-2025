# Length Measurement Receiver

This repository contains the *length-measurement* module which uses a ZED 2i Stereolabs stereo camera to measure distances in real time over a network stream using Gstreamer.

## Prerequisites

Before building or running the examples, ensure you have:

- **USB3 Stereolabs Stereo camera**: ZED 2i, ZED 2, ZED, or ZED Mini
- **Linux OS** (Ubuntu 18.04+, Debian, etc.)
- **GCC** (v7.5 or newer)
- **CMake** (v3.1 or newer)
- **OpenCV** (v3.4.0 or newer)&#x20;

## Installation of System Dependencies

To install all required system libraries, run the helper script:

```bash
./install_prereqs.sh
```

---

## Running the Sender (Raspberry Pi)

On your Raspberry Pi (connected to the ZED camera), stream the stereo feed via GStreamer:

```bash
./rtsp_stream.sh <RECEIVER_IP>
```

- Replace `<RECEIVER_IP>` with the IP address of the PC that will run the receiver

---

## Building and Running the Receiver (PC)

On your PC, build and install the length-measurement receiver tool:

```bash
./build_and_run_receiver.sh
```

---

## Usage

1. **Start the sender** on the Pi: `./rtsp_stream.sh 192.168.1.50`
2. **Run the receiver** on the PC: `./build_and_run_receiver.sh`
3. Click two points in the video window to measure distance.

---

