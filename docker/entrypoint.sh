#!/usr/bin/env bash

sudo chown "$USER" /isaac-sim/kit/cache -R
sudo chown "$USER" /isaac-sim/kit/data -R
sudo chown "$USER" /isaac-sim/kit/logs -R
source /opt/ros/humble/setup.bash
bash
