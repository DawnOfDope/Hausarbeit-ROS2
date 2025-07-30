#!/bin/bash

# --- Set Path to your Keystore ---
export ROS_SECURITY_KEYSTORE=~/Hausarbeit-ROS2/keystore

# --- Enable Security ---
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Permissive
export ROS_DOMAIN_ID=0

echo "ROS 2 Security is ON (Strategy: Permissive)"