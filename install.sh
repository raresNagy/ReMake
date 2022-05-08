#!/bin/bash
echo "Installing python dependencies"
sudo apt update
sudo apt install python3-dev
python -m pip install smbus 
python -m pip install -e .
pip install numpy opencv-python yaml picamera netiface


