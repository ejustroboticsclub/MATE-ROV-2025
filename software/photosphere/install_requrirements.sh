#!/bin/bash

# Install system dependencies
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-tk python3-venv

# Upgrade pip inside venv
pip install --upgrade pip

# Install project dependencies
pip install -r requirements.txt
