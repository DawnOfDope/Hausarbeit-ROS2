#!/bin/bash

# --- Disable Security ---
export ROS_SECURITY_ENABLE=false

# --- Unset other security variables for a clean environment ---
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_STRATEGY

echo "ROS 2 Security is OFF"