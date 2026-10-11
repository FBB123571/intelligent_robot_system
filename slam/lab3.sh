#!/usr/bin/env bash
# Run directly inside an Ubuntu 16.04 / ROS Kinetic VM.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$action" == help ]]; then
  echo '用法: bash slam/lab3.sh {sync|base|camera|mapping|view|teleop|save [地图名]|navigation 地图名|goal --goal X Y 朝向度数}'
  exit 0
fi
source /opt/ros/kinetic/setup.bash
WS="${LAB3_CATKIN_WS:-$HOME/catkin_ws}"
export TURTLEBOT_BASE="${TURTLEBOT_BASE:-kobuki}"
export TURTLEBOT_STACKS="${TURTLEBOT_STACKS:-hexagons}"
export TURTLEBOT_3D_SENSOR="${TURTLEBOT_3D_SENSOR:-kinect}"
export TURTLEBOT_BATTERY="${TURTLEBOT_BATTERY:-None}"
export TURTLEBOT_SIMULATION=false
export TURTLEBOT_SERIAL_PORT="${TURTLEBOT_SERIAL_PORT:-/dev/kobuki}"
if [[ "$action" == sync ]]; then
  mkdir -p "$WS/src"
  if [[ -e "$WS/src/path_planning" || -L "$WS/src/path_planning" ]]; then
    if [[ "$(readlink -f "$WS/src/path_planning")" != "$ROOT/path_planning" ]]; then
      echo "已有其他 path_planning 包: $WS/src/path_planning；请先备份或设置 LAB3_CATKIN_WS。" >&2
      exit 1
    fi
  else
    ln -s "$ROOT/path_planning" "$WS/src/path_planning"
  fi
  cd "$WS"
  catkin_make -j2
  exit 0
fi
if [[ ! -f "$WS/devel/setup.bash" ]]; then
  echo '请先运行 bash slam/lab3.sh sync' >&2
  exit 1
fi
source "$WS/devel/setup.bash"
case "$action" in
  base) exec roslaunch turtlebot_bringup minimal.launch ;;
  camera) exec roslaunch openni_launch openni.launch publish_tf:=false ;;
  mapping|navigation)
    nodes="$(rosnode list)"
    if printf '%s\n' "$nodes" | grep -Eq '^/(slam_gmapping|amcl|move_base|lab3_depth_scan|rectangle|circle|marker_follower|ar_location|turtlebot_teleop_keyboard)$'; then
      echo '已有建图、导航或运动节点；请先结束对应控制终端。' >&2
      exit 1
    fi
    python -c 'import rospy, tf
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
      [[ -f "$ROOT/maps/$1.yaml" ]] || { echo "地图不存在: $ROOT/maps/$1.yaml" >&2; exit 1; }
      exec roslaunch path_planning navigation.launch "map_file:=$ROOT/maps/$1.yaml"
    fi
    exec roslaunch path_planning mapping.launch
    ;;
  view) exec roslaunch turtlebot_rviz_launchers view_navigation.launch ;;
  teleop) exec roslaunch turtlebot_teleop keyboard_teleop.launch ;;
  save)
    name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
    [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || { echo '地图名仅可使用字母、数字、下划线和连字符'; exit 2; }
    mkdir -p "$ROOT/maps"
    [[ ! -e "$ROOT/maps/$name.yaml" && ! -e "$ROOT/maps/$name.pgm" ]] || { echo '同名地图已存在'; exit 1; }
    cd "$ROOT/maps"
    exec timeout 30 rosrun map_server map_saver -f "$name"
    ;;
  goal) exec rosrun path_planning navigation_goal.py "$@" ;;
  *) echo "未知命令: $action" >&2; exit 2 ;;
esac
