#!/bin/bash

# chmod +x visquad_clear_disk.sh #Enable execute
# sudo visquad_clear_disk.sh #Run with sudo

# Free up ROS logs and cache
cd /root/.ros
rm -rf log/*
# rm -rf cache/*

# Free up var logs and cache
cd /var
rm -rf log/*
rm -rf cache/*

# Clean apt downloaded files
apt-get autoclean

# Clear trash
rm -rf ~/.local/share/Trash/*
