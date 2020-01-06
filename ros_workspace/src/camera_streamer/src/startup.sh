#!/bin/bash

# Script to be run at EVERY boot

# Attempt to pull latest repo
cd /home/jhsrobo/rpicamera/ || exit
git pull

# Start up camera streamer and restart it if it exits
while true; do
  sudo python3 /home/jhsrobo/rpicamera/streamer/streamer.py
  done

exit 0
