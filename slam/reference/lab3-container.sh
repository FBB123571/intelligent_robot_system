#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=handex-lab2-robot
ros() {
  local command
  local -a terminal=(-i)
  if [[ -t 0 && -t 1 ]]; then terminal=(-it); fi
  printf -v command '%q ' "$@"
  podman exec "${terminal[@]}" "$CONTAINER" bash -lc "source /root/catkin_ws/devel/setup.bash; exec $command"
}
action="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
  prepare)
    bash "$ROOT/../lab2/start_robot.sh"
    bash "$ROOT/lab3.sh" sync
    ;;
  sync)
    # Preserve any previous container copy before synchronizing.
    ros bash -c 'if [ -d /root/catkin_ws/src/path_planning ]; then backup=$(mktemp -d /root/lab3-source-backup.XXXXXX); cp -a /root/catkin_ws/src/path_planning "$backup/"; fi'
    podman cp "$ROOT/path_planning" "$CONTAINER:/root/catkin_ws/src/"
    ros bash -c 'cd /root/catkin_ws && catkin_make -j2'
    ;;
  mapping|navigation)
    nodes="$(ros rosnode list)"
    if printf '%s\n' "$nodes" | grep -Eq '^/(slam_gmapping|amcl|move_base|lab3_depth_scan|rectangle|circle|marker_follower|ar_location|turtlebot_teleop_keyboard)$'; then
      echo '已有建图、导航或运动节点；请先结束对应控制终端。' >&2
      exit 1
    fi
    ros python -c 'import rospy, tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
rospy.init_node("lab3_preflight", anonymous=True)
for topic, kind in [("/odom", Odometry), ("/camera/depth/image_raw", Image), ("/camera/depth/camera_info", CameraInfo)]:
    a=rospy.wait_for_message(topic, kind, timeout=10)
    b=rospy.wait_for_message(topic, kind, timeout=10)
    assert b.header.stamp > a.header.stamp, topic + " timestamps not advancing"
    assert abs((rospy.Time.now()-b.header.stamp).to_sec()) < 2, topic + " stale"
    print(topic + " OK")
listener=tf.TransformListener()
listener.waitForTransform("odom", "camera_depth_frame", rospy.Time(0), rospy.Duration(10))
print("TF odom -> camera_depth_frame OK")'
    if [[ "$action" == navigation ]]; then
      [[ $# -eq 1 && "$1" =~ ^[A-Za-z0-9_-]+$ ]] || { echo '用法: navigation 地图名'; exit 2; }
      ros test -f "/root/lab3_maps/$1.yaml"
      ros roslaunch path_planning navigation.launch "map_file:=/root/lab3_maps/$1.yaml"
    else
      ros roslaunch path_planning mapping.launch
    fi
    ;;
  view)
    ros roslaunch turtlebot_rviz_launchers view_navigation.launch
    ;;
  teleop)
    podman exec -it "$CONTAINER" bash -lc 'source /root/catkin_ws/devel/setup.bash; exec roslaunch turtlebot_teleop keyboard_teleop.launch'
    ;;
  save)
    name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
    [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || { echo '地图名仅可使用字母、数字、下划线和连字符'; exit 2; }
    [[ ! -e "$ROOT/maps/$name.yaml" && ! -e "$ROOT/maps/$name.pgm" ]] || { echo '本地同名地图已存在'; exit 1; }
    ros bash -c 'mkdir -p /root/lab3_maps; test ! -e "$1.yaml" && test ! -e "$1.pgm"' _ "/root/lab3_maps/$name"
    ros bash -c 'cd /root/lab3_maps && timeout 30 rosrun map_server map_saver -f "$1"' _ "$name"
    mkdir -p "$ROOT/maps"
    podman cp "$CONTAINER:/root/lab3_maps/$name.pgm" "$ROOT/maps/"
    podman cp "$CONTAINER:/root/lab3_maps/$name.yaml" "$ROOT/maps/"
    ;;
  goal)
    ros rosrun path_planning navigation_goal.py "$@"
    ;;
  *)
    echo '用法: bash lab3/lab3.sh {prepare|sync|mapping|view|teleop|save [地图名]|navigation 地图名|goal --goal X Y 朝向度数 [--goal ...]}'
    ;;
esac
