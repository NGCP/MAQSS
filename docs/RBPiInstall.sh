#!/usr/bin/env bash

# Installs programs necessary to run and maintain MAQSS onboard the Raspberry Pi 3B
# Assumes Raspbian Jessie has been installed
# WILL NOT install OpenCV or Raspicam

clear
echo "Installing Programs onto Raspberry Pi...."
echo "Installing..."

echo "cmake..."
echo "git..."
echo "vim..."
echo "minicom..."
echo "wget..."
echo "feh..."
echo "boost..."
sudo apt-get install cmake git vim minicom wget feh libboost1.55-all-dev

echo "Eigen..."
cd ~/Downloads
pwd
wget -O eigen3.2.10.tar.bz2 http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
tar -xvjf eigen3.2.10.tar.bz2
cd eigen-eigen-b9cd8366d4e8
sudo mv Eigen /usr/local/include

echo "Raspberry Pi Programs Installed"
