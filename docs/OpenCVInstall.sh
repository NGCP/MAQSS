#!/usr/bin/env bash

# Installs OpenCV 3.0 and Rapicam Library
# Assumes Raspbian Jessie has been installed
# removes Wolfram engine to free up harddisk memory

echo "Installing OpenCV and Raspicam"

# remove Wolfram engine
echo "Removing Wolfram Engine..."
sudo apt-get purge wolfram-engine

# install dependencies
echo "Installing Dependencies..."
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev, image IO
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev, video IO
sudo apt-get install libgtk2.0-dev, highgui support
sudo apt-get install libatlas-base-dev gfortran, misc dependencies

# downloads OpenCV
echo "Downloading OpenCV..."
cd ~/Downloads 
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.1.0.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.1.0.zip
unzip opencv_contrib.zip

# extract and make
echo "Extracting OpenCV..."
cd ~/Downloads/opencv-3.1.0/
mkdir build
cd build

echo "Configuring cmake..."
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=OFF -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib-3.1.0/modules -D BUILD_EXAMPLES=ON ..

echo "Building project..."
make -j4
sudo make install
sudo ldconfig
echo "OpenCV built and installed"

# Download and install Raspicam
echo "Installing Raspicam Library..."
wget https://sourceforge.net/projects/raspicam/files/raspicam-0.1.3.zip/download
unzip raspicam-0.1.3.zip
cd raspicam-0.1.3
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig


