~/ros2-ws-2025/scripts/can-setup.sh
source ~/ros2-ws-2025/install/local_setup.sh
ros2 run autonomous auto &
ros2 run sensors sensor &
ros2 run db_broker broker --ros-args -p can_id:=15 &
ros2 run db_broker broker --ros-args -p can_id:=16 &
ros2 run db_broker broker --ros-args -p can_id:=17 &
ros2 run db_broker broker --ros-args -p can_id:=18 
