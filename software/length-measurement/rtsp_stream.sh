#!/bin/bash

# === SETTINGS ===
WIDTH=1344
HEIGHT=376
FPS=30
PORT=5000

# === STREAM COMMAND ===
gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
"video/x-raw,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1" ! \
videoconvert ! x264enc tune=zerolatency bitrate=5000 speed-preset=superfast ! \
rtph264pay config-interval=1 pt=96 ! \
udpsink host=$1 port=$PORT
