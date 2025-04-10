#!/bin/bash
IFACE="can0"
can2=0
# Scan For Device
if lsusb | grep -i "16d0:117e MCS CANable2 b158aa7 github.com/normaldotcom/canable2.git" > /dev/null; then
    echo "CAN2 Device Connected...Attempting Alternate Setup"
    can2=1
else 
    echo "Attempting to Connect to CAN Device"
fi

if [ "$can2" -eq 0 ]; then
    # Check if interface exists
    if ip link show "$IFACE" > /dev/null 2>&1; then
        echo "Interface $IFACE exists."

        # Check if it's already up
        if ip link show "$IFACE" | grep -q "state UP"; then
            echo "Interface $IFACE is already up."

        else
            echo "Bringing $IFACE up..."
            sudo ip link set "$IFACE" up
        fi

    else
        echo "Starting $IFACE now"
        "$(pwd)/scripts/can-setup.sh"
    fi

elif ls /dev | grep ttyACM0 > /dev/null; then
    sudo slcand -o -c -s6 /dev/ttyACM0 can0
    sudo ifconfig can0 up
    sudo ifconfig can0 txqueuelen 1000
    echo "found Can2 Device at ACM0"

elif ls /dev | grep ttyACM1 > /dev/null; then
    sudo slcand -o -c -s6 /dev/ttyACM1 can0
    sudo ifconfig can0 up
    sudo ifconfig can0 txqueuelen 1000
    echo "found Can2 Device at ACM1"

else
    echo "Not Can Device Found"
    exit 1
fi

source "$(pwd)/install/local_setup.sh"
ros2 launch "$(pwd)/launch/main_launch.py"